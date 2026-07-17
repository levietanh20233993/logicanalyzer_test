"""
main.py  -  Logic Analyzer  |  RP2040 -> COM7 -> pygame
=======================================================
Kien truc 2 tien trinh (multiprocessing):

  Tien trinh 1  (data_process)
    -> Doc COM7  ->  tach CH1/CH2  ->  ghi vao shared ring buffer
       (di chuyen write_ptr)

  Tien trinh 2  (main / display_process)
    -> Doc ring buffer  ->  ve cham "." cao/thap theo bit  ->  pygame
       (di chuyen read_ptr)

THAY DOI THEO YEU CAU:
  - Moi bit duoc ve thanh 1 dau "." :
        bit = 1 -> dau "." ve o vi tri CAO
        bit = 0 -> dau "." ve o vi tri THAP
  - 2 con tro write_ptr / read_ptr:
        + write_ptr: tien trinh 1 (data_process) tang khi nhan byte moi tu COM7
        + read_ptr : tien trinh 2 (display_process) tang khi ve data ra man hinh
    Khi 2 con tro CHAM NHAU (read_ptr == write_ptr, khong con data moi de doc,
    hoac buffer day khong con cho de ghi) -> TAM DUNG (khong tien len) cho den
    khi co du lieu / cho moi.
  - Toc do doc/ve man hinh CO DINH 30 FPS (khong con tu dong theo SAMPLE_RATE).
  - Co the ZOOM (PageUp/PageDown hoac scroll chuot) de xem chi tiet toi don vi ms.
  - Truc thoi gian ben duoi, thoi gian bat dau = 0 (t=0 la thoi diem bat dau chuong trinh).
  - Nhan "Z" de DUNG/TIEP TUC (pause/resume) viec doc va ve du lieu moi.

Cai dat:
    pip install pygame pyserial
Chay:
    python main.py
"""

import time, math
import multiprocessing as mp
import ctypes
import pygame
import serial

# ===============================================================
#  CAU HINH
# ===============================================================
COM_PORT       = "COM7"
BAUD_RATE      = 921600
SAMPLE_RATE_HZ = 921600      # phai khop SAMPLE_FREQ_HZ trong main.c (RP2040)

FPS            = 45          # co dinh 30 FPS theo yeu cau

# ---- Cau hinh decode UART (tuy chon, bat/tat bang phim U) ----
UART_BAUD       = 115200
SAMPLES_PER_BIT = SAMPLE_RATE_HZ // UART_BAUD   # = 8 (so nguyen, khong lech)
BIT_MID         = SAMPLES_PER_BIT // 2          # vi tri lay mau giua bit (mau thu 4/8)

UART_IDLE, UART_START, UART_DATA, UART_STOP = range(4)


class UartDecoder:
    """May trang thai decode UART (8N1, LSB truoc), chay tren tung MAU 1-bit.

    feed(bit, ts) tra ve (byte, t_start, t_end) neu vua hoan thanh 1 khung
    hop le (bao gom timestamp bat dau/ket thuc de ve overlay dung vi tri),
    nguoc lai tra ve None.
    """

    def __init__(self):
        self.state = UART_IDLE
        self.prev_bit = 1        # duong truyen idle muc cao
        self.bit_pos = 0         # vi tri mau trong bit hien tai (0..SAMPLES_PER_BIT-1)
        self.data_bit_idx = 0
        self.rx_byte = 0
        self.t_start = 0.0
        self.frame_error_count = 0

    def feed(self, bit, ts):
        result = None

        if self.state == UART_IDLE:
            if self.prev_bit == 1 and bit == 0:   # canh xuong -> start bit
                self.state = UART_START
                self.bit_pos = 1
                self.t_start = ts

        elif self.state == UART_START:
            if self.bit_pos == BIT_MID:
                if bit != 0:                       # glitch, khong phai start bit that
                    self.state = UART_IDLE
                    self.prev_bit = bit
                    return None
            self.bit_pos += 1
            if self.bit_pos == SAMPLES_PER_BIT:
                self.state = UART_DATA
                self.bit_pos = 0
                self.data_bit_idx = 0
                self.rx_byte = 0

        elif self.state == UART_DATA:
            if self.bit_pos == BIT_MID:
                self.rx_byte |= (bit << self.data_bit_idx)   # LSB truoc
            self.bit_pos += 1
            if self.bit_pos == SAMPLES_PER_BIT:
                self.bit_pos = 0
                self.data_bit_idx += 1
                if self.data_bit_idx == 8:
                    self.state = UART_STOP

        elif self.state == UART_STOP:
            if self.bit_pos == BIT_MID:
                if bit == 1:                       # stop bit hop le -> byte hoan chinh
                    result = (self.t_start, ts, self.rx_byte)
                else:
                    self.frame_error_count += 1     # khung loi, bo qua
            self.bit_pos += 1
            if self.bit_pos == SAMPLES_PER_BIT:
                self.state = UART_IDLE
                self.bit_pos = 0

        self.prev_bit = bit
        return result

WIN_W, WIN_H   = 1400, 700
PANEL_W        = 260

# Zoom: so micro-giay hien thi tren 1 pixel cua vung ve song
# Gia tri nho hon => zoom sau hon (nhin duoc tung ms / us)
ZOOM_LEVELS_US_PER_PX = [
    0.1,
    0.2,
    0.5,
    1,      # 1 us/px  - zoom sau nhat (nhin tung mau ~10us)
    2, 5, 10, 20, 50, 100, 200, 500,
    1000,   # 1 ms/px
    2000, 5000, 10000,
]
DEFAULT_ZOOM_INDEX = 9   # ~200 us/px

# ===============================================================
#  SHARED RING BUFFER (multiprocessing shared memory)
#
#  Moi kenh co 1 ring buffer rieng dung Array(ctypes.c_uint8).
#  2 con tro write_ptr / read_ptr dung Value(c_int64), TANG DAN VO HAN
#  (khong wrap thu cong) - chi so thuc trong array = ptr & MASK.
#
#  Tien trinh 1 -> ghi qua write_ptr (tien len khi co data moi)
#  Tien trinh 2 -> doc qua read_ptr  (tien len khi ve ra man hinh)
#
#  "Cham nhau":
#    - write_ptr cham read_ptr + RING (buffer day) -> tien trinh 1 TAM DUNG
#      (khong ghi them, cho tien trinh 2 doc bot)
#    - read_ptr cham write_ptr (khong con data moi) -> tien trinh 2 TAM DUNG
#      (khong ve them, cho tien trinh 1 ghi them)
# ===============================================================
# RING du lon de chua nhieu giay du lieu (tang len 60s cho dung luong lon hon)
_raw   = int(60 * SAMPLE_RATE_HZ) + 8192
RING   = 2 ** math.ceil(math.log2(_raw))
MASK   = RING - 1


def make_shared_buffers():
    """Tao shared memory dung chung giua 2 tien trinh."""
    ch1_bits = mp.Array(ctypes.c_uint8, RING, lock=False)
    ch2_bits = mp.Array(ctypes.c_uint8, RING, lock=False)
    # Timestamp luu dang float64 (giay, tinh tu thoi diem start_time = 0)
    sample_ts = mp.Array(ctypes.c_double, RING, lock=False)

    write_ptr = mp.Value(ctypes.c_int64, 0)   # tien trinh 1 ghi (tang dan)
    read_ptr  = mp.Value(ctypes.c_int64, 0)   # tien trinh 2 doc (tang dan)

    # Thong ke chia se
    stat_bytes   = mp.Value(ctypes.c_int64,  0,     lock=True)
    stat_samples = mp.Value(ctypes.c_int64,  0,     lock=True)
    stat_hz      = mp.Value(ctypes.c_double, 0.0,   lock=True)
    # stat_conn: 0=dang ket noi, 1=connected, 2=loi
    stat_conn    = mp.Value(ctypes.c_int8,   0,     lock=True)
    stat_err     = mp.Array(ctypes.c_char,   256)
    start_time   = mp.Value(ctypes.c_double, time.time())

    # Co bao "buffer day" / "het data" de hien thi trang thai PAUSE tren UI
    stat_write_paused = mp.Value(ctypes.c_int8, 0, lock=True)  # 1 = write bi cham buffer day
    stat_read_paused  = mp.Value(ctypes.c_int8, 0, lock=True)  # 1 = read bi cham (het data moi)

    return (ch1_bits, ch2_bits, sample_ts,
            write_ptr, read_ptr,
            stat_bytes, stat_samples, stat_hz,
            stat_conn, stat_err, start_time,
            stat_write_paused, stat_read_paused)


# ===============================================================
#  TIEN TRINH 1  :  data_process
#  Nhan data tu COM7, tach CH1/CH2, ghi vao ring buffer
#  -> di chuyen write_ptr. Neu buffer day (write_ptr cham read_ptr
#     + RING) thi TAM DUNG khong ghi them.
# ===============================================================
def data_process(shared):
    (ch1_bits, ch2_bits, sample_ts,
     write_ptr, read_ptr,
     stat_bytes, stat_samples, stat_hz,
     stat_conn, stat_err, start_time,
     stat_write_paused, stat_read_paused) = shared

    def buf_free():
        avail = write_ptr.value - read_ptr.value
        return RING - 1 - avail

    rate_n, rate_t = 0, time.perf_counter()

    stat_conn.value = 0  # 0 = connecting

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        stat_conn.value = 1          # 1 = connected
        stat_err.value  = b""
    except Exception as e:
        stat_conn.value = 2          # 2 = loi
        stat_err.value = str(e).encode()[:255]
        return

    try:
        while True:
            raw = ser.read(512)
            if not raw:
                continue

            stat_bytes.value += len(raw)
            now = time.perf_counter()
            n   = len(raw)

            for i, byte in enumerate(raw):
    # Mot byte = 4 sample (autopush 8 bit = 4 lan "in pins,2")
    # MSB-first (shift left): bit7,6 = sample CU NHAT trong byte;
    #                          bit1,0 = sample MOI NHAT trong byte
                for sub in range(4):
        # Cho neu buffer day -> 2 con tro "cham nhau" o phia ghi
                    while buf_free() < 1:
                        stat_write_paused.value = 1
                        time.sleep(0.001)
                    stat_write_paused.value = 0

                    shift = (3 - sub) * 2          # 6,4,2,0
                    pair  = (byte >> shift) & 0b11
                    ch1   = pair & 1
                    ch2   = (pair >> 1) & 1

                    # vi tri sample nay trong qua khu so voi sample cuoi cung
                    # tong so sample con lai sau byte hien tai (tinh ca sub-sample):
                    samples_remaining = (n - i - 1) * 4 + (3 - sub)
                    ts = (time.time() - start_time.value) - samples_remaining / SAMPLE_RATE_HZ

                    idx = write_ptr.value & MASK
                    ch1_bits[idx]  = ch1
                    ch2_bits[idx]  = ch2
                    sample_ts[idx] = ts
                    write_ptr.value += 1

                stat_samples.value += n * 4

            rate_n += n
            dt = now - rate_t
            if dt >= 0.5:
                stat_hz.value  = rate_n / dt
                rate_n, rate_t = 0, now

    except Exception as e:
        stat_conn.value = 2          # 2 = loi/mat ket noi
        stat_err.value  = str(e).encode()[:255]
    finally:
        try: ser.close()
        except: pass


# ===============================================================
#  HAM LAY SAMPLE MOI DE VE  (tien trinh 2 dung)
#
#  Co che:
#    - Tien trinh 2 muon ve cua so [t_left, t_right] (theo zoom hien tai)
#    - Doc cac sample tu read_ptr den write_ptr, day vao history buffer
#      (list Python, giu toan bo de co the zoom ra xem lai)
#    - read_ptr TANG khi sample da duoc "tieu thu" (dua vao history)
#    - Neu read_ptr == write_ptr (khong con sample moi) -> TAM DUNG,
#      khong tang read_ptr, danh dau stat_read_paused = 1
# ===============================================================
def pull_new_samples(shared, history_ch1, history_ch2, history_ts, max_history):
    (ch1_bits, ch2_bits, sample_ts,
     write_ptr, read_ptr, *_rest) = shared
    stat_read_paused = shared[12]

    wp = write_ptr.value
    rp = read_ptr.value

    if wp == rp:
        # Khong con data moi -> 2 con tro cham nhau o phia doc -> tam dung
        stat_read_paused.value = 1
        return 0
    stat_read_paused.value = 0

    avail = wp - rp
    # Gioi han so sample xu ly moi frame de tranh giat (nhung van theo kip)
    n = min(avail, 200000)

    for k in range(n):
        idx = (rp + k) & MASK
        history_ch1.append(int(ch1_bits[idx]))
        history_ch2.append(int(ch2_bits[idx]))
        history_ts.append(float(sample_ts[idx]))

    read_ptr.value = rp + n

    # Cat history neu qua dai (giu toi da max_history sample gan nhat)
    drop = 0
    if len(history_ts) > max_history:
        drop = len(history_ts) - max_history
        del history_ch1[:drop]
        del history_ch2[:drop]
        del history_ts[:drop]

    return drop


# ===============================================================
#  VE DANG SONG BANG DAU "."  (cham cao / cham thap)
# ===============================================================
def draw_waveform_dots(surf, history_bits, history_ts, x0, y0, w, h,
                        color, ch_label, font, font_xs,
                        t_left, t_right,
                        decode_mode=False, decoded_events=None,
                        frame_error_count=0):
    # Nen + vien
    pygame.draw.rect(surf, (16, 19, 28), (x0, y0, w, h))
    pygame.draw.rect(surf, (50, 55, 72), (x0, y0, w, h), 1)

    # Ten kenh
    surf.blit(font.render(ch_label, True, color), (x0 + 8, y0 + 8))

    lbl_w  = 70
    draw_x = x0 + lbl_w
    draw_w = w - lbl_w - 8

    margin = 14
    y_high = y0 + margin
    y_low  = y0 + h - margin

    # Nhan muc
    surf.blit(font_xs.render("1", True, (120, 125, 140)), (x0 + lbl_w - 18, y_high - 6))
    surf.blit(font_xs.render("0", True, (120, 125, 140)), (x0 + lbl_w - 18, y_low - 6))

    # Duong tham chieu mo
    pygame.draw.line(surf, (30, 35, 48), (draw_x, y_high), (draw_x + draw_w, y_high), 1)
    pygame.draw.line(surf, (30, 35, 48), (draw_x, y_low), (draw_x + draw_w, y_low), 1)

    # Duong doc phan cach label
    pygame.draw.line(surf, (50, 55, 72), (draw_x - 1, y0), (draw_x - 1, y0 + h), 1)

    if len(history_ts) < 1:
        msg = font_xs.render("Cho tin hieu tu " + COM_PORT + "...", True, (70, 75, 90))
        surf.blit(msg, (draw_x + draw_w // 2 - 80, y0 + h // 2 - 7))
        return

    t_span = t_right - t_left
    if t_span <= 0:
        return

    def t_to_x(ts):
        return draw_x + (ts - t_left) / t_span * draw_w

    # Binary-search de chi quet vung can thiet (history_ts tang dan)
    n = len(history_ts)
    lo, hi = 0, n
    while lo < hi:
        mid = (lo + hi) // 2
        if history_ts[mid] < t_left:
            lo = mid + 1
        else:
            hi = mid
    start_idx = lo

    lo, hi = 0, n
    while lo < hi:
        mid = (lo + hi) // 2
        if history_ts[mid] <= t_right:
            lo = mid + 1
        else:
            hi = mid
    end_idx = lo

    # So pixel tren moi sample (10us tuong ung 1 sample o 100kHz)
    px_per_sample = draw_w / t_span * (1.0 / SAMPLE_RATE_HZ)

    if px_per_sample >= 2 and (end_idx - start_idx) > 0:
        # ===== ZOOM SAU: ve dang song "bac thang" ro tung bit =====
        dim_color = (color[0] // 2, color[1] // 2, color[2] // 2)
        prev_x, prev_y = None, None
        for i in range(start_idx, end_idx):
            x = t_to_x(history_ts[i])
            y = y_high if history_bits[i] == 1 else y_low
            if prev_x is not None:
                # duong ngang giu muc cu toi vi tri hien tai
                pygame.draw.line(surf, color, (prev_x, prev_y), (x, prev_y), 2)
                # duong doc transition (neu doi muc)
                if prev_y != y:
                    pygame.draw.line(surf, color, (x, prev_y), (x, y), 2)
            prev_x, prev_y = x, y
        # keo dai muc cuoi den canh phai vung ve
        if prev_x is not None:
            pygame.draw.line(surf, color, (prev_x, prev_y),
                              (draw_x + draw_w, prev_y), 2)

        # ve them dau "." tai vi tri tung sample de de doc gia tri tung bit
        dot_radius = 3
        for i in range(start_idx, end_idx):
            x = t_to_x(history_ts[i])
            if history_bits[i] == 1:
                pygame.draw.circle(surf, color, (int(x), y_high), dot_radius)
            else:
                pygame.draw.circle(surf, dim_color, (int(x), y_low), dot_radius)

    else:
        # ===== ZOOM XA: group theo pixel, ve cham dai dien =====
        pixel_has1 = [False] * (draw_w + 1)
        pixel_has0 = [False] * (draw_w + 1)

        for i in range(start_idx, end_idx):
            ts = history_ts[i]
            px = int((ts - t_left) / t_span * draw_w)
            if 0 <= px <= draw_w:
                if history_bits[i] == 1:
                    pixel_has1[px] = True
                else:
                    pixel_has0[px] = True

        dot_radius = 2 if t_span > 0.01 else 3

        for px in range(draw_w + 1):
            cx = draw_x + px
            if pixel_has1[px]:
                pygame.draw.circle(surf, color, (cx, y_high), dot_radius)
            if pixel_has0[px]:
                dim_color = (color[0] // 2, color[1] // 2, color[2] // 2)
                pygame.draw.circle(surf, dim_color, (cx, y_low), dot_radius)

    # ===== OVERLAY DECODE UART (tuy chon) =====
    # Ve 1 dai mau tai vi tri tung khung UART da decode duoc, chong len
    # dung doan bit [t_start, t_end] cua khung do, kem chu hex gia tri byte.
    if decode_mode and decoded_events:
        band_h = 18
        band_y = (y_high + y_low) // 2 - band_h // 2

        for (ev_t_start, ev_t_end, ev_byte) in decoded_events:
            if ev_t_end < t_left or ev_t_start > t_right:
                continue   # khung nam ngoai vung dang xem -> bo qua

            xs = t_to_x(max(ev_t_start, t_left))
            xe = t_to_x(min(ev_t_end, t_right))
            bw = max(1, xe - xs)

            overlay = pygame.Surface((int(bw), band_h), pygame.SRCALPHA)
            overlay.fill((255, 190, 40, 170))   # mau vang cam ban trong suot
            surf.blit(overlay, (int(xs), band_y))
            pygame.draw.rect(surf, (255, 210, 90), (int(xs), band_y, int(bw), band_h), 1)

            # Chi ve chu hex neu du rong de khong bi de len nhau
            label = f"{ev_byte:02X}"
            lbl_surf = font_xs.render(label, True, (20, 15, 5))
            if lbl_surf.get_width() + 4 <= bw:
                lx = int(xs + (bw - lbl_surf.get_width()) / 2)
                surf.blit(lbl_surf, (lx, band_y + 3))

        # Nhan trang thai decode + so khung loi (neu co)
        status = "DECODE UART: BAT"
        if frame_error_count:
            status += f"   loi khung: {frame_error_count}"
        surf.blit(font_xs.render(status, True, (255, 210, 90)), (x0 + w - 8 - 260, y0 + 8))

    # Cursor tai mep phai
    pygame.draw.line(surf, (color[0] // 2, color[1] // 2, color[2] // 2),
                     (draw_x + draw_w - 1, y0 + 4),
                     (draw_x + draw_w - 1, y0 + h - 4), 1)

# ===============================================================
#  VE TRUC THOI GIAN (t=0 la luc bat dau chuong trinh)
# ===============================================================
def draw_time_axis(surf, font_xs, x0, y0, w, t_left, t_right):
    pygame.draw.rect(surf, (14, 16, 24), (x0, y0, w, 30))
    pygame.draw.line(surf, (50, 55, 72), (x0, y0), (x0 + w, y0), 1)

    lbl_w  = 70
    draw_x = x0 + lbl_w
    draw_w = w - lbl_w - 8

    t_span = t_right - t_left
    if t_span <= 0:
        return

    us_per_px = (t_span * 1_000_000) / draw_w

    if us_per_px <= 2.0:
        if us_per_px < 1.0:
            # ===== ZOOM SIEU SAU: ve tick moi 0.1 us =====
            step_native = 0.1
            ns_left  = math.floor(t_left  * 10_000_000)  # don vi 0.1us
            ns_right = math.ceil(t_right * 10_000_000)

            min_label_gap_px = 36
            px_per_native = draw_w / t_span * (step_native / 1_000_000.0)
            label_step_units = max(1, math.ceil(min_label_gap_px / px_per_native))

            for n in range(int(ns_left), int(ns_right) + 1):
                t = n * step_native / 1_000_000.0
                px = draw_x + int((t - t_left) / t_span * draw_w)
                if draw_x <= px <= draw_x + draw_w:
                    pygame.draw.line(surf, (50, 55, 72), (px, y0), (px, y0 + 8), 1)
                    if n % label_step_units == 0:
                        val = n * step_native
                        lbl = f"{val:.1f} us"
                        t_surf = font_xs.render(lbl, True, (100, 105, 120))
                        surf.blit(t_surf, (px - t_surf.get_width() // 2, y0 + 10))
        else:
            # ===== ZOOM SAU: ve tick moi 1 us =====
            us_left  = math.floor(t_left * 1_000_000)
            us_right = math.ceil(t_right * 1_000_000)

            min_label_gap_px = 36
            label_step_us = max(1, math.ceil(min_label_gap_px * us_per_px))

            for us in range(int(us_left), int(us_right) + 1):
                t = us / 1_000_000.0
                px = draw_x + int((t - t_left) / t_span * draw_w)
                if draw_x <= px <= draw_x + draw_w:
                    pygame.draw.line(surf, (50, 55, 72), (px, y0), (px, y0 + 8), 1)
                    if us % label_step_us == 0:
                        lbl = f"{us} us"
                        t_surf = font_xs.render(lbl, True, (100, 105, 120))
                        surf.blit(t_surf, (px - t_surf.get_width() // 2, y0 + 10))
    else:
        # ===== ZOOM XA: chia deu 10 moc nhu cu =====
        n_ticks = 10
        for i in range(n_ticks + 1):
            frac = i / n_ticks
            px = draw_x + int(frac * draw_w)
            t  = t_left + frac * t_span

            if t < 0:
                t = 0.0

            if t_span < 0.001:
                lbl = f"{t * 1e6:.1f} us"
            elif t_span < 1.0:
                lbl = f"{t * 1e3:.2f} ms"
            else:
                lbl = f"{t:.3f} s"

            pygame.draw.line(surf, (50, 55, 72), (px, y0), (px, y0 + 8), 1)
            t_surf = font_xs.render(lbl, True, (100, 105, 120))
            surf.blit(t_surf, (px - t_surf.get_width() // 2, y0 + 10))

    # Nhan "t=0" rieng o goc neu 0 nam trong khung nhin
    if t_left <= 0 <= t_right:
        px = draw_x + int((0 - t_left) / t_span * draw_w)
        pygame.draw.line(surf, (255, 210, 60), (px, y0), (px, y0 + 8), 2)
        t_surf = font_xs.render("t=0", True, (255, 210, 60))
        surf.blit(t_surf, (px - t_surf.get_width() // 2, y0 + 18))

# ===============================================================
#  VE THANH RING BUFFER
# ===============================================================
def draw_ring_bar(surf, font_xs, x0, y0, bw, write_ptr, read_ptr):
    pygame.draw.rect(surf, (22, 26, 36), (x0, y0, bw, 10))
    pygame.draw.rect(surf, (50, 55, 72), (x0, y0, bw, 10), 1)

    avail = write_ptr - read_ptr
    fw = int(min(max(avail, 0), RING) / RING * bw)
    if fw > 0:
        fill_color = (30, 80, 45) if fw < bw * 0.8 else (120, 50, 30)
        pygame.draw.rect(surf, fill_color, (x0, y0, fw, 10))

    wp = int((write_ptr & MASK) / RING * bw)
    rp = int((read_ptr & MASK) / RING * bw)
    pygame.draw.line(surf, (80, 220, 90), (x0 + wp, y0 - 6), (x0 + wp, y0 + 15), 2)
    pygame.draw.line(surf, (160, 110, 255), (x0 + rp, y0 - 6), (x0 + rp, y0 + 15), 2)
    surf.blit(font_xs.render("W", True, (80, 220, 90)), (x0 + wp - 4, y0 - 17))
    surf.blit(font_xs.render("R", True, (160, 110, 255)), (x0 + rp - 4, y0 + 16))

    pct = int(min(max(avail, 0), RING) / RING * 100)
    surf.blit(font_xs.render(f"{pct}%  ({avail:,}/{RING:,})",
              True, (100, 105, 120)), (x0, y0 + 28))

# ===============================================================
#  VE PANEL THONG TIN
# ===============================================================
def draw_panel(surf, H, font_b, font, font_xs, shared, elapsed,
               zoom_us_per_px, write_paused, read_paused, user_stopped,
               decode_uart_mode=False):
    (ch1_bits, ch2_bits, sample_ts,
     write_ptr, read_ptr,
     stat_bytes, stat_samples, stat_hz,
     stat_conn, stat_err, start_time,
     stat_write_paused, stat_read_paused) = shared

    pygame.draw.rect(surf, (18, 20, 30), (0, 0, PANEL_W, H))
    pygame.draw.line(surf, (50, 55, 72), (PANEL_W - 1, 0), (PANEL_W - 1, H), 1)

    pad, lh = 14, 22
    y = pad

    # -- Tieu de --
    surf.blit(font_b.render("Logic Analyzer", True, (255, 185, 0)), (pad, y)); y += 30
    surf.blit(font_xs.render("RP2040  GPIO  ->  USB CDC", True, (100, 105, 120)), (pad, y)); y += lh + 6

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Thong ke --
    hz = stat_hz.value
    hz_str = f"{hz/1000:.2f} kHz" if hz >= 1000 else f"{hz:.0f} Hz"

    if zoom_us_per_px >= 1000:
        zoom_str = f"{zoom_us_per_px/1000:.1f} ms/px"
    else:
        zoom_str = f"{zoom_us_per_px} us/px"

    rows = [
        ("Cong COM",     COM_PORT),
        ("Tan so cfg",   f"{SAMPLE_RATE_HZ//1000} kHz"),
        ("Toc do thuc",  hz_str),
        ("Zoom",         zoom_str),
        ("FPS (cfg)",    f"{FPS}"),
        ("Tong mau",     f"{stat_samples.value:,}"),
        ("Bytes RX",     f"{stat_bytes.value:,}"),
    ]
    for k, v in rows:
        surf.blit(font_xs.render(k + ":", True, (120, 125, 140)), (pad, y))
        surf.blit(font_xs.render(v, True, (210, 215, 220)), (pad + 95, y))
        y += lh

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Trang thai DUNG/CHAY (phim Z) --
    surf.blit(font_xs.render("Trang thai (Z):", True, (120, 125, 140)), (pad, y))
    if user_stopped:
        surf.blit(font.render("DA DUNG (STOPPED)", True, (255, 90, 90)), (pad + 95, y))
    else:
        surf.blit(font.render("DANG CHAY", True, (0, 210, 90)), (pad + 95, y))
    y += lh + 4

    # -- Trang thai DECODE UART (phim U) --
    surf.blit(font_xs.render("Decode UART (U):", True, (120, 125, 140)), (pad, y))
    if decode_uart_mode:
        surf.blit(font.render("BAT", True, (255, 210, 90)), (pad + 115, y))
    else:
        surf.blit(font.render("TAT", True, (100, 105, 120)), (pad + 115, y))
    y += lh + 4

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Trang thai 2 con tro --
    surf.blit(font_xs.render("-- 2 con tro --", True, (80, 85, 100)), (pad, y)); y += 18

    surf.blit(font_xs.render("Write (ghi):", True, (120, 125, 140)), (pad, y))
    wstate = "TAM DUNG (day)" if write_paused else "dang ghi"
    wcolor = (255, 140, 0) if write_paused else (0, 210, 90)
    surf.blit(font_xs.render(wstate, True, wcolor), (pad + 95, y)); y += lh

    surf.blit(font_xs.render("Read (doc):", True, (120, 125, 140)), (pad, y))
    if user_stopped:
        rstate, rcolor = "TAM DUNG (Z)", (255, 90, 90)
    elif read_paused:
        rstate, rcolor = "TAM DUNG (het data)", (160, 110, 255)
    else:
        rstate, rcolor = "dang doc", (0, 210, 90)
    surf.blit(font_xs.render(rstate, True, rcolor), (pad + 95, y)); y += lh

    if write_paused or read_paused or user_stopped:
        msg = font_xs.render("=> 2 con tro da CHAM NHAU", True, (255, 90, 90))
        surf.blit(msg, (pad, y)); y += lh

    y += 4
    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Dong ho (t=0 la luc bat dau) --
    m, s = divmod(int(elapsed), 60)
    h_val, m = divmod(m, 60)
    surf.blit(font_xs.render("Thoi gian:", True, (120, 125, 140)), (pad, y))
    surf.blit(font.render(f"{h_val:02d}:{m:02d}:{s:02d}", True, (255, 220, 80)), (pad + 90, y))
    y += lh + 4
    surf.blit(font_xs.render("(t=0 = bat dau chuong trinh)", True, (90, 95, 110)), (pad, y))
    y += lh + 4

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Trang thai ket noi --
    conn = stat_conn.value
    if conn == 1:
        sc, st = (0, 210, 90), "*  CONNECTED"
    elif conn == 0:
        sc, st = (255, 180, 0), "*  CONNECTING..."
    else:
        sc, st = (255, 70, 70), "*  ERROR / DISCONNECTED"
    surf.blit(font.render(st, True, sc), (pad, y)); y += lh + 2

    err = stat_err.value.decode(errors="replace").strip()
    if err:
        for ln in [err[i:i+28] for i in range(0, len(err), 28)][:3]:
            surf.blit(font_xs.render(ln, True, (220, 80, 80)), (pad, y)); y += 16
    y += 4

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Legend --
    surf.blit(font_xs.render("Kenh:", True, (100, 105, 120)), (pad, y)); y += 18
    pygame.draw.circle(surf, (0, 220, 130), (pad + 8, y + 6), 3)
    surf.blit(font_xs.render("CH1  GPIO pin 0", True, (0, 220, 130)), (pad + 22, y)); y += lh
    pygame.draw.circle(surf, (80, 160, 255), (pad + 8, y + 6), 3)
    surf.blit(font_xs.render("CH2  GPIO pin 1", True, (80, 160, 255)), (pad + 22, y)); y += lh + 6

    surf.blit(font_xs.render("bit 1 -> . (cham CAO)", True, (160, 165, 175)), (pad, y)); y += 16
    surf.blit(font_xs.render("bit 0 -> . (cham THAP)", True, (160, 165, 175)), (pad, y)); y += lh + 6

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Zoom control hint --
    surf.blit(font_xs.render("Zoom:", True, (100, 105, 120)), (pad, y)); y += 16
    surf.blit(font_xs.render("  Lan/cuon chuot = zoom in/out", True, (160, 165, 175)), (pad, y)); y += 15
    surf.blit(font_xs.render("  +/- hoac PgUp/PgDn", True, (160, 165, 175)), (pad, y)); y += 18
    surf.blit(font_xs.render("  Z = Dung / Tiep tuc", True, (255, 220, 80)), (pad, y)); y += 18
    surf.blit(font_xs.render("  U = Bat/Tat decode UART", True, (255, 210, 90)), (pad, y)); y += 18

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Pan (keo chuot xem lai) hint --
    surf.blit(font_xs.render("Xem bit o vi tri can:", True, (100, 105, 120)), (pad, y)); y += 16
    surf.blit(font_xs.render("  Giu + keo chuot trai = di chuyen", True, (160, 165, 175)), (pad, y)); y += 15
    surf.blit(font_xs.render("  thanh hien thi qua khu/hien tai", True, (160, 165, 175)), (pad, y)); y += 15
    surf.blit(font_xs.render("  Click phai / Home / L = ve LIVE", True, (255, 220, 80)), (pad, y)); y += 18

    pygame.draw.line(surf, (45, 50, 65), (pad, y), (PANEL_W - pad, y), 1); y += 8

    # -- Ring buffer --
    surf.blit(font_xs.render("Ring buffer:", True, (100, 105, 120)), (pad, y)); y += 16
    surf.blit(font_xs.render("* W = write_ptr (tien trinh 1)", True, (80, 220, 90)), (pad, y)); y += 15
    surf.blit(font_xs.render("* R = read_ptr  (tien trinh 2)", True, (160, 110, 255)), (pad, y)); y += 18
    draw_ring_bar(surf, font_xs, pad, y,
                  PANEL_W - pad * 2,
                  write_ptr.value, read_ptr.value)


# ===============================================================
#  VE THANH TRANG THAI TREN CUNG
# ===============================================================
def draw_topbar(surf, font_xs, font_b, wx, ww, n_samples_visible, elapsed,
                user_stopped, is_live, is_dragging):
    pygame.draw.rect(surf, (14, 16, 24), (wx, 0, ww, 36))
    pygame.draw.line(surf, (50, 55, 72), (wx, 36), (wx + ww, 36), 1)

    left = (f"  COM7   {SAMPLE_RATE_HZ//1000} kHz   "
            f"FPS: {FPS}   "
            f"Mau hien thi: {n_samples_visible:,}")
    right = ("[ESC] Thoat   [+/-] Zoom   [Scroll] Zoom   [Z] Dung/Tiep tuc   [U] Decode UART   "
             "[Keo chuot] Xem lai   [Home/L] Live")

    tl = font_xs.render(left, True, (90, 95, 110))
    tr = font_xs.render(right, True, (90, 95, 110))
    surf.blit(tl, (wx + 8, 10))
    surf.blit(tr, (wx + ww - tr.get_width() - 4, 10))

    # -- Banner "DA DUNG" hien thi noi bat khi nguoi dung nhan Z --
    if user_stopped:
        label = font_b.render("DA DUNG (STOPPED) - Nhan Z de tiep tuc", True, (255, 90, 90))
        bx = wx + ww // 2 - label.get_width() // 2
        pygame.draw.rect(surf, (40, 16, 16), (bx - 10, 4, label.get_width() + 20, 28))
        pygame.draw.rect(surf, (255, 90, 90), (bx - 10, 4, label.get_width() + 20, 28), 1)
        surf.blit(label, (bx, 8))
    else:
        # -- Bao LIVE / XEM LAI (khi nguoi dung keo chuot de xem qua khu) --
        if is_live:
            label = font_b.render("LIVE", True, (0, 220, 90))
            bx = wx + ww // 2 - label.get_width() // 2
            pygame.draw.rect(surf, (10, 30, 16), (bx - 10, 4, label.get_width() + 20, 28))
            pygame.draw.rect(surf, (0, 220, 90), (bx - 10, 4, label.get_width() + 20, 28), 1)
            surf.blit(label, (bx, 8))
        else:
            txt = "DANG KEO XEM LAI..." if is_dragging else \
                  "XEM LAI - Click phai hoac Home/L de ve LIVE"
            label = font_b.render(txt, True, (255, 200, 60))
            bx = wx + ww // 2 - label.get_width() // 2
            pygame.draw.rect(surf, (40, 32, 8), (bx - 10, 4, label.get_width() + 20, 28))
            pygame.draw.rect(surf, (255, 200, 60), (bx - 10, 4, label.get_width() + 20, 28), 1)
            surf.blit(label, (bx, 8))


# ===============================================================
#  TIEN TRINH 2  :  display_process  (main process)
# ===============================================================
def display_process(shared):
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption("Logic Analyzer  -  RP2040  GPIO")
    clock = pygame.time.Clock()

    font_b  = pygame.font.SysFont("Consolas", 17, bold=True)
    font    = pygame.font.SysFont("Consolas", 13)
    font_xs = pygame.font.SysFont("Consolas", 11)

    start_time = shared[10]
    start_ts = start_time.value

    stat_write_paused = shared[11]
    stat_read_paused  = shared[12]

    # History buffer (toan bo sample da "tieu thu" boi read_ptr, de co the
    # zoom ra xem lai). Gioi han so luong de khong tran RAM.
    MAX_HISTORY = SAMPLE_RATE_HZ * 60  # toi da 60s du lieu trong history
    history_ch1 = []
    history_ch2 = []
    history_ts  = []

    zoom_index = DEFAULT_ZOOM_INDEX

    # user_stopped: True khi nguoi dung nhan "Z" -> dung viec doc/ve du lieu
    # moi (read_ptr KHONG tien len nua). Nhan "Z" lan nua de tiep tuc.
    user_stopped = False

    # ------------------------------------------------------------
    # CHUC NANG KEO CHUOT DE XEM BIT O VI TRI CAN (PAN)
    # ------------------------------------------------------------
    # view_offset: so giay LUI VE QUA KHU so voi mau moi nhat (t_now).
    #   = 0   -> dang xem "LIVE" (du lieu moi nhat luon o mep phai man hinh)
    #   > 0   -> da keo chuot sang PHAI -> xem lai du lieu cu hon (qua khu)
    # Gia tri nay duoc cong/tru truc tiep boi do dich chuyen chuot (pixel)
    # quy doi theo ty le zoom hien tai (us_per_px), nen keo chuot = "keo"
    # waveform di theo dung huong ngon tay/chuot.
    view_offset = 0.0

    # dragging: dang giu chuot trai va keo de cuon waveform
    is_dragging = False
    # us_per_px cua frame truoc, dung de quy doi pixel chuot -> thoi gian
    last_us_per_px = ZOOM_LEVELS_US_PER_PX[zoom_index]

    # ------------------------------------------------------------
    # CHE DO DECODE UART (tuy chon, bat/tat bang phim "U")
    # ------------------------------------------------------------
    # Khi bat: tin hieu bit tren man hinh VAN hien thi binh thuong nhu cu,
    # nhung doan nao decode duoc 1 byte UART hop le se duoc chong 1 dai
    # mau + chu hex len dung doan bit tuong ung (xem draw_waveform_dots).
    decode_uart_mode = False

    decoder_ch1 = UartDecoder()
    decoder_ch2 = UartDecoder()
    decode_idx_ch1 = 0   # vi tri (chi so trong history_ch1) da dua vao decoder
    decode_idx_ch2 = 0
    decoded_events_ch1 = []   # list (t_start, t_end, byte)
    decoded_events_ch2 = []
    MAX_DECODE_EVENTS = 20000   # gioi han so event luu, tranh tran RAM

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); return
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    pygame.quit(); return
                if ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_PAGEUP):
                    zoom_index = max(0, zoom_index - 1)  # zoom in (it us/px hon)
                if ev.key in (pygame.K_MINUS, pygame.K_PAGEDOWN):
                    zoom_index = min(len(ZOOM_LEVELS_US_PER_PX) - 1, zoom_index + 1)
                if ev.key == pygame.K_z:
                    # Bam "Z" -> dao trang thai DUNG / TIEP TUC
                    user_stopped = not user_stopped
                if ev.key in (pygame.K_HOME, pygame.K_l):
                    # Bam Home hoac L -> quay ve xem LIVE (mau moi nhat)
                    view_offset = 0.0
                if ev.key == pygame.K_u:
                    # Bam "U" -> bat/tat che do decode UART (tuy chon)
                    decode_uart_mode = not decode_uart_mode
            if ev.type == pygame.MOUSEWHEEL:
                if ev.y > 0:
                    zoom_index = max(0, zoom_index - 1)
                elif ev.y < 0:
                    zoom_index = min(len(ZOOM_LEVELS_US_PER_PX) - 1, zoom_index + 1)

            # -- Bat dau keo chuot: nhan giu nut chuot TRAI tren vung song --
            if ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    is_dragging = True
                elif ev.button == 3:
                    # Click chuot PHAI -> quay nhanh ve xem LIVE
                    view_offset = 0.0

            if ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    is_dragging = False

            # -- Dang keo chuot: dich chuyen "thanh hien thi" (vung xem) --
            # Keo chuot sang PHAI -> xem lai du lieu CU hon (lui ve qua khu)
            # Keo chuot sang TRAI -> tien gan ve hien tai (toi da = LIVE)
            if ev.type == pygame.MOUSEMOTION and is_dragging:
                dx_px = ev.rel[0]
                view_offset += dx_px * (last_us_per_px / 1_000_000.0)
                if view_offset < 0:
                    view_offset = 0.0


        W, H = screen.get_size()
        screen.fill((11, 13, 20))

        # -- Lay sample moi tu ring buffer (di chuyen read_ptr) --
        # Neu nguoi dung da nhan "Z" (user_stopped = True) thi KHONG goi
        # pull_new_samples -> read_ptr dung yen, man hinh dong bang lai
        # (du tien trinh 1 van tiep tuc thu data vao buffer phia sau).
        if not user_stopped:
            drop = pull_new_samples(shared, history_ch1, history_ch2, history_ts, MAX_HISTORY)
            if drop:
                # History vua bi cat bot phia dau -> lui lai con tro decode
                # tuong ung de khong bi lech chi so (khong decode lai/nham).
                decode_idx_ch1 = max(0, decode_idx_ch1 - drop)
                decode_idx_ch2 = max(0, decode_idx_ch2 - drop)

            if decode_uart_mode:
                # Chi decode CAC MAU MOI duoc them vao tu lan truoc den gio
                while decode_idx_ch1 < len(history_ch1):
                    b = history_ch1[decode_idx_ch1]
                    t = history_ts[decode_idx_ch1]
                    r = decoder_ch1.feed(b, t)
                    if r is not None:
                        decoded_events_ch1.append(r)
                    decode_idx_ch1 += 1

                while decode_idx_ch2 < len(history_ch2):
                    b = history_ch2[decode_idx_ch2]
                    t = history_ts[decode_idx_ch2]
                    r = decoder_ch2.feed(b, t)
                    if r is not None:
                        decoded_events_ch2.append(r)
                    decode_idx_ch2 += 1

                # Gioi han so event luu lai (giu cac event gan nhat)
                if len(decoded_events_ch1) > MAX_DECODE_EVENTS:
                    del decoded_events_ch1[:len(decoded_events_ch1) - MAX_DECODE_EVENTS]
                if len(decoded_events_ch2) > MAX_DECODE_EVENTS:
                    del decoded_events_ch2[:len(decoded_events_ch2) - MAX_DECODE_EVENTS]

        write_paused = bool(stat_write_paused.value)
        read_paused  = bool(stat_read_paused.value)

        elapsed = time.time() - start_ts

        # -- Layout --
        wx   = PANEL_W + 6
        ww   = W - wx - 6
        top  = 38
        bot  = 36
        gap  = 6
        ch_h = (H - top - bot - gap) // 2
        ch1_y = top
        ch2_y = top + ch_h + gap

        # -- Tinh khung thoi gian hien thi theo zoom --
        us_per_px = ZOOM_LEVELS_US_PER_PX[zoom_index]
        last_us_per_px = us_per_px  # luu lai de quy doi pixel chuot ky tiep
        draw_w = ww - 70 - 8  # giong lbl_w/margin trong ham ve
        span_sec = (us_per_px * draw_w) / 1_000_000.0

        if history_ts:
            t_now = history_ts[-1]
        else:
            t_now = 0.0

        # Gioi han view_offset: khong vuot qua du lieu xa nhat dang co
        # (t_now - span_sec), va khong nho hon 0 (= LIVE)
        max_offset = max(0.0, t_now - span_sec)
        if view_offset > max_offset:
            view_offset = max_offset
        if view_offset < 0:
            view_offset = 0.0

        # mep phai cua "thanh hien thi" = t_now lui ve view_offset giay
        t_right = max(t_now - view_offset, span_sec)  # khong de t_right < span (tranh am)
        t_left  = t_right - span_sec
        if t_left < 0:
            t_left = 0.0
            t_right = span_sec

        is_live = view_offset <= 1e-9

        n_visible = sum(1 for t in history_ts if t_left <= t <= t_right)

        # -- Ve panel trai --
        draw_panel(screen, H, font_b, font, font_xs, shared, elapsed,
                   us_per_px, write_paused, read_paused, user_stopped,
                   decode_uart_mode=decode_uart_mode)

        # -- Ve thanh top --
        draw_topbar(screen, font_xs, font_b, wx, ww, n_visible, elapsed, user_stopped,
                    is_live, is_dragging)


        # -- Ve 2 kenh (dang cham "." cao/thap) --
        draw_waveform_dots(screen, history_ch1, history_ts, wx, ch1_y, ww, ch_h,
                            (0, 220, 130), "CH1", font, font_xs, t_left, t_right,
                            decode_mode=decode_uart_mode, decoded_events=decoded_events_ch1,
                            frame_error_count=decoder_ch1.frame_error_count)
        draw_waveform_dots(screen, history_ch2, history_ts, wx, ch2_y, ww, ch_h,
                            (80, 160, 255), "CH2", font, font_xs, t_left, t_right,
                            decode_mode=decode_uart_mode, decoded_events=decoded_events_ch2,
                            frame_error_count=decoder_ch2.frame_error_count)

        # -- Truc thoi gian (t=0 = bat dau) --
        draw_time_axis(screen, font_xs, wx, H - bot - 2, ww, t_left, t_right)

        pygame.display.flip()

        # -- Co dinh 30 FPS theo yeu cau --
        clock.tick(FPS)


# ===============================================================
#  ENTRY POINT
# ===============================================================
if __name__ == "__main__":
    mp.freeze_support()

    shared = make_shared_buffers()

    p1 = mp.Process(target=data_process, args=(shared,), daemon=True, name="DataProcess")
    p1.start()
    print(f"[OK] Tien trinh 1 (DataProcess) PID={p1.pid} dang chay -> {COM_PORT}")

    print(f"[OK] Tien trinh 2 (DisplayProcess) PID={mp.current_process().pid} dang chay -> pygame")
    display_process(shared)

    p1.terminate()
    p1.join()
    print("[OK] Da dung.")