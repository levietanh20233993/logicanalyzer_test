"""
Logic Analyzer — 8 kênh, cửa sổ 10 giây trượt từ trái sang phải
  CH1→bit0, CH2→bit1, ..., CH8→bit7 (8 bit liên tiếp trong mỗi word 32-bit)
  Mỗi pixel = (SAMPLE_RATE × WINDOW_S / wave_width) mẫu
  Tín hiệu mới xuất hiện bên phải, cũ trôi sang trái.

Cài:  pip install pygame pyserial
Chạy thật:  python main1.py
Chạy demo:  python main1.py --demo
"""

import sys, struct, threading, time
from collections import deque
import pygame

# ══════════════════════════════════════
#  CẤU HÌNH
# ══════════════════════════════════════
SAMPLE_RATE = 1_000_000        # Hz
BUFFER_SIZE = 1024             # word 32-bit, khớp firmware
WINDOW_S    = 10.0             # giây hiển thị trên 1 màn hình
WIN_W       = 1280
WIN_H       = 680
NUM_CH      = 8          # số kênh hiển thị (CH1–CH8 = bit0–bit7)
CH_GAP      = 4          # khoảng cách giữa các hàng sóng (px)
FPS         = 60

BG           = ( 13,  17,  23)
GRID_C       = ( 30,  41,  59)
COLOR_HIGH   = ( 34, 197,  94)
COLOR_LOW    = (100, 116, 139)
COLOR_EDGE   = ( 71,  85, 105)
COLOR_TEXT   = (148, 163, 184)
COLOR_BORDER = ( 51,  65,  85)

PAD_L = 60
PAD_R = 16
PAD_T = 24
PAD_B = 42   # chỗ cho nhãn thời gian + footer

CH_NAMES = [f"CH{i+1}  bit{i}" for i in range(NUM_CH)]

# ══════════════════════════════════════
#  DECODE WORD → 8 BIT THẤP
# ══════════════════════════════════════
def word_to_8bits(w):
    """Trả về list 8 bit: [bit0, bit1, ..., bit7]"""
    return [(w >> i) & 1 for i in range(8)]

# ══════════════════════════════════════
#  SERIAL READER — thread riêng
# ══════════════════════════════════════
class SerialReader(threading.Thread):
    def __init__(self, port, word_queue):
        super().__init__(daemon=True)
        self.port       = port
        self.word_queue = word_queue  # deque of tuple(8 bits), 1 entry = 1 word
        self.running    = True
        self.status     = "connecting..."
        self.kbps       = 0.0
        self._tbytes    = 0
        self._t0        = time.time()

    def run(self):
        import serial
        try:
            s = serial.Serial(self.port, 115200, timeout=2)
            self.status = f"OK {self.port}"
            need = BUFFER_SIZE * 4
            while self.running:
                raw = b""
                while len(raw) < need:
                    c = s.read(need - len(raw))
                    if c:
                        raw += c
                    else:
                        time.sleep(0.001)
                if len(raw) < need:
                    continue
                self._tbytes += len(raw)
                now = time.time()
                if now - self._t0 >= 1.0:
                    self.kbps = self._tbytes / 1024 / (now - self._t0)
                    self._tbytes = 0
                    self._t0 = now
                words = struct.unpack(f"<{BUFFER_SIZE}I", raw)
                for w in words:
                    # Đẩy tuple 8-bit của cả word vào 1 queue duy nhất
                    self.word_queue.append(word_to_8bits(w))
        except Exception as e:
            self.status = f"ERR: {e}"

# ══════════════════════════════════════
#  TÌM CỔNG RP2040
# ══════════════════════════════════════
def find_port():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if getattr(p, 'vid', None) == 0x2E8A:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("Không tìm thấy cổng serial nào → chuyển sang DEMO")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} — {p.description}")
    return ports[int(input("Chọn: "))].device

# ══════════════════════════════════════
#  VẼ SÓNG  (1 sample = 1 pixel)
# ══════════════════════════════════════
def draw_wave(surf, samples, x0, y0, wave_h):
    """Vẽ trực tiếp lên surf; x0,y0 là góc trên-trái của vùng sóng."""
    HIGH_Y = y0 + int(wave_h * 0.18)
    LOW_Y  = y0 + int(wave_h * 0.82)
    n = len(samples)
    if n < 2:
        return
    i = 0
    while i < n:
        bit   = samples[i]
        sy    = HIGH_Y if bit else LOW_Y
        color = COLOR_HIGH if bit else COLOR_LOW
        j = i + 1
        while j < n and samples[j] == bit:
            j += 1
        pygame.draw.line(surf, color, (x0 + i, sy), (x0 + j - 1, sy), 2)
        if j < n:
            ny = HIGH_Y if samples[j] else LOW_Y
            pygame.draw.line(surf, COLOR_EDGE, (x0 + j, sy), (x0 + j, ny), 2)
        i = j

# ══════════════════════════════════════
#  STATS (đơn vị cột, chuyển về Hz)
# ══════════════════════════════════════
def calc_stats(samples, spc):
    """spc = số mẫu thật mỗi cột hiển thị."""
    if len(samples) < 2:
        return "—", "—"
    periods, last_rise, high = [], -1, 0
    for i in range(1, len(samples)):
        if samples[i]:
            high += 1
        if samples[i] == 1 and samples[i - 1] == 0:
            if last_rise >= 0:
                periods.append((i - last_rise) * spc)
            last_rise = i
    freq = (SAMPLE_RATE / (sum(periods) / len(periods))) if periods else None
    duty = high / len(samples) * 100
    if freq:
        fs = f"{freq / 1000:.2f} kHz" if freq >= 1000 else f"{freq:.1f} Hz"
    else:
        fs = "—"
    return fs, f"{duty:.1f}%"

# ══════════════════════════════════════
#  MAIN
# ══════════════════════════════════════
def main():
    demo = "--demo" in sys.argv

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption("Logic Analyzer — 8 CH / 10 s window")
    font = pygame.font.SysFont("Consolas", 11)
    clk  = pygame.time.Clock()

    # Một queue duy nhất chứa tuple 8-bit (1 entry = 1 word 32-bit từ firmware)
    word_queue = deque(maxlen=400_000)
    reader     = None

    if not demo:
        try:
            port   = find_port()
            reader = SerialReader(port, word_queue)
            reader.start()
        except Exception as e:
            print(f"\n[!] Không tìm thấy thiết bị serial: {e}")
            print("    Thoát chương trình.")
            pygame.quit()
            sys.exit(0)

    # ── Bộ tích lũy word → cột pixel ─────────────────────────────────
    wave_w_init = WIN_W - PAD_L - PAD_R
    spc         = SAMPLE_RATE * WINDOW_S / wave_w_init   # word/cột

    # Tất cả kênh dùng chung 1 bộ đếm → luôn đồng bộ theo word
    display_bufs = [deque(maxlen=wave_w_init) for _ in range(NUM_CH)]
    acc_count    = 0.0
    acc_last     = [0] * NUM_CH   # giá trị bit cuối của từng kênh trong cột hiện tại

    wave_w_prev = wave_w_init

    # Demo: sóng vuông với tần số khác nhau mỗi kênh
    DEMO_PERIODS        = [200_000, 100_000, 50_000, 25_000,
                           12_500,    6_250,  3_125,  1_562]
    demo_phases         = [0] * NUM_CH
    DEMO_BITS_PER_FRAME = int(SAMPLE_RATE / FPS)

    freq_str, duty_str = "—", "—"
    stat_timer = 0

    # ── Helper: nạp 1 word (tuple 8 bit) — cập nhật tất cả kênh cùng lúc ──
    def feed_word(bits8):
        nonlocal acc_count
        for ch in range(NUM_CH):
            acc_last[ch] = bits8[ch]
        acc_count += 1
        if acc_count >= spc:
            for ch in range(NUM_CH):
                display_bufs[ch].append(acc_last[ch])
            acc_count -= spc

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_x \
                    and (ev.mod & pygame.KMOD_CTRL):
                running = False
            if ev.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode((ev.w, ev.h), pygame.RESIZABLE)

        W, H = screen.get_size()
        wave_w = max(W - PAD_L - PAD_R, 1)
        # Chia chiều cao đều cho NUM_CH hàng
        wave_h = max((H - PAD_T - PAD_B - CH_GAP * (NUM_CH - 1)) // NUM_CH, 1)

        # ── Cập nhật spc & resize ring buffer khi cửa sổ thay đổi ───
        if wave_w != wave_w_prev:
            new_spc = SAMPLE_RATE * WINDOW_S / wave_w
            for ch in range(NUM_CH):
                old_items = list(display_bufs[ch])
                new_buf   = deque(maxlen=wave_w)
                if old_items:
                    n_old = len(old_items)
                    for i in range(min(wave_w, n_old)):
                        idx = int(i * n_old / min(wave_w, n_old))
                        new_buf.append(old_items[min(idx, n_old - 1)])
                display_bufs[ch] = new_buf
            spc         = new_spc
            acc_count   = 0.0   # reset tích lũy khi resize
            wave_w_prev = wave_w

        # ── Nạp dữ liệu (theo word — tất cả kênh cùng bước) ─────────
        if demo:
            # Sinh word giả: mỗi kênh 1 bit độc lập, cùng đẩy vào feed_word
            for _ in range(DEMO_BITS_PER_FRAME):
                bits8 = []
                for ch in range(NUM_CH):
                    period = DEMO_PERIODS[ch]
                    b = 1 if (demo_phases[ch] % period) < (period // 2) else 0
                    bits8.append(b)
                    demo_phases[ch] += 1
                feed_word(bits8)
        else:
            # Pop từng word ra, feed cả 8 kênh cùng lúc
            n = len(word_queue)
            for _ in range(n):
                feed_word(word_queue.popleft())

        # ── Stats mỗi 30 frame (CH1) ─────────────────────────────────
        stat_timer += 1
        if stat_timer >= 30 and len(display_bufs[0]) > 10:
            freq_str, duty_str = calc_stats(list(display_bufs[0]), spc)
            stat_timer = 0

        # ── Render ───────────────────────────────────────────────────
        surf = pygame.Surface((W, H))
        surf.fill(BG)

        wx = PAD_L

        for ch_idx in range(NUM_CH):
            wy = PAD_T + ch_idx * (wave_h + CH_GAP)

            # Grid dọc: mỗi 1 giây = 1/10 chiều rộng
            for sec in range(11):
                gx = wx + int(sec * wave_w / 10)
                pygame.draw.line(surf, GRID_C, (gx, wy), (gx, wy + wave_h), 1)
                # Nhãn thời gian chỉ vẽ ở hàng cuối
                if ch_idx == NUM_CH - 1:
                    lbl = font.render(f"{sec}s", True, COLOR_TEXT)
                    surf.blit(lbl, (gx - lbl.get_width() // 2, wy + wave_h + 4))

            # Border
            pygame.draw.rect(surf, COLOR_BORDER, (wx, wy, wave_w, wave_h), 1)

            # Nhãn Y (1 / 0)
            HIGH_Y = wy + int(wave_h * 0.18)
            LOW_Y  = wy + int(wave_h * 0.82)
            surf.blit(font.render("1", True, COLOR_HIGH), (wx - 14, HIGH_Y - 6))
            surf.blit(font.render("0", True, COLOR_LOW),  (wx - 14, LOW_Y  - 6))

            # Sóng
            samples_ch = list(display_bufs[ch_idx])
            if samples_ch and wave_w > 0 and wave_h > 0:
                try:
                    clip = surf.subsurface(pygame.Rect(wx, wy, wave_w, wave_h))
                    draw_wave(clip, samples_ch, 0, 0, wave_h)
                except ValueError:
                    pass

            # Tên kênh (góc trên trái của vùng sóng)
            surf.blit(font.render(CH_NAMES[ch_idx], True, COLOR_HIGH), (wx + 4, wy + 3))

        # ── Footer ───────────────────────────────────────────────────
        wy_last = PAD_T + (NUM_CH - 1) * (wave_h + CH_GAP)
        fy = wy_last + wave_h + 20
        spc_ms = spc / SAMPLE_RATE * 1000
        if reader:
            info = (f"{reader.status}  |  {reader.kbps:.1f} KB/s  |  "
                    f"CH1 Freq: {freq_str}  Duty: {duty_str}  |  "
                    f"Window: {WINDOW_S:.0f}s  |  {SAMPLE_RATE // 1000} kHz  |  "
                    f"{spc_ms:.2f} ms/col")
        else:
            info = (f"[DEMO]  CH1 Freq: {freq_str}  Duty: {duty_str}  |  "
                    f"Window: {WINDOW_S:.0f}s  |  {spc_ms:.2f} ms/col  |  ESC để thoát")

        surf.blit(font.render(info, True, COLOR_TEXT), (wx, fy))

        screen.blit(surf, (0, 0))
        pygame.display.flip()
        clk.tick(FPS)

    if reader:
        reader.running = False
    pygame.quit()


if __name__ == "__main__":
    main()