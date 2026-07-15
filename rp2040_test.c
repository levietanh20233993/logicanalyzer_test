/**
 * main.c - RP2040 PIO GPIO Reader  |  Dual DMA Ping-Pong  |  USB CDC
 * =====================================================================
 * Luồng dữ liệu:
 *   GPIO pin → PIO (in pins,2) → autopush mỗi 8 bit → RX FIFO
 *   → DMA_SIZE_8 → buffer_A / buffer_B (uint8_t)
 *   → IRQ → CPU → fwrite() → USB CDC → máy tính
 *
 * Mỗi byte nhận được:
 *   bit[0] = trạng thái GPIO_PIN
 *   bit[1] = trạng thái GPIO_PIN + 1
 *   (các bit còn lại = 0, PIO chỉ shift 2 bit mỗi lần vào ISR)
 *
 * Autopush threshold = 8 bit  →  cứ 4 lần "in pins,2" thì push 1 byte
 * DMA chuyển từng BYTE (DMA_SIZE_8) từ PIO RX FIFO vào RAM
 *
 * Build (CMakeLists.txt):
 *   target_link_libraries(... pico_stdlib hardware_pio hardware_dma
 *                             hardware_irq hardware_clocks)
 *   pico_enable_stdio_usb(... 1)
 *   pico_enable_stdio_uart(... 0)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "blink.pio.h"

/* ─────────────────────────────────────────────────────────────────────
 * CẤU HÌNH
 * ───────────────────────────────────────────────────────────────────── */

#define GPIO_PIN        2        // chân đầu tiên (đọc pin này và pin+1)
#define SAMPLE_FREQ_HZ  500000   // tần số lấy mẫu (Hz)
                                 // = tần số PIO clock / 1 chu kỳ lệnh
                                 // Mỗi "in pins,2" chiếm 1 chu kỳ PIO

/*
 * BUFFER_SIZE: số BYTE mỗi buffer ping-pong.
 * PIO autopush 8 bit = 1 byte / lần push.
 * DMA chuyển BUFFER_SIZE lần × 1 byte = BUFFER_SIZE byte rồi mới IRQ.
 * Chọn sao cho thời gian fill 1 buffer ≈ 1–10 ms để CPU kịp gửi USB.
 *
 *   Thời gian fill = BUFFER_SIZE / SAMPLE_FREQ_HZ
 *   Ví dụ: 100 bytes / 100000 Hz = 1 ms
 */
#define BUFFER_SIZE     2048      // byte mỗi buffer (tăng/giảm tuỳ throughput USB)

/* ─────────────────────────────────────────────────────────────────────
 * BUFFER PING-PONG  (uint8_t vì DMA_SIZE_8)
 * ───────────────────────────────────────────────────────────────────── */
static uint8_t buffer_A[BUFFER_SIZE];
static uint8_t buffer_B[BUFFER_SIZE];

static int  dma_chan_a;
static int  dma_chan_b;

volatile bool buffer_A_ready = false;
volatile bool buffer_B_ready = false;

/* ─────────────────────────────────────────────────────────────────────
 * Tính PIO clock divider
 * PIO chạy ở sys_clk / divider
 * Mỗi "in pins,2" = 1 chu kỳ PIO → sample rate = sys_clk / divider
 * ───────────────────────────────────────────────────────────────────── */
static float calc_clkdiv(uint32_t freq_hz) {
    float div = (float)clock_get_hz(clk_sys) / (float)freq_hz;
    if (div < 1.0f)      div = 1.0f;
    if (div > 65535.0f)  div = 65535.0f;
    return div;
}

/* ─────────────────────────────────────────────────────────────────────
 * DMA IRQ handler
 *
 * Khi DMA A hoàn thành BUFFER_SIZE byte → set cờ A_ready, reset write addr
 * Khi DMA B hoàn thành BUFFER_SIZE byte → set cờ B_ready, reset write addr
 *
 * chain_to đã tự kích channel kia, không cần trigger thủ công.
 * Chỉ cần reset write_addr để lần chain tiếp theo ghi từ đầu buffer.
 * ───────────────────────────────────────────────────────────────────── */
static void dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_chan_a)) {
        dma_channel_acknowledge_irq0(dma_chan_a);
        buffer_A_ready = true;
        // Reset địa chỉ ghi để vòng chain tiếp theo ghi đè từ đầu buffer
        dma_channel_set_write_addr(dma_chan_a, buffer_A, false);
        dma_channel_set_trans_count(dma_chan_a, BUFFER_SIZE, false);
    }

    if (dma_channel_get_irq0_status(dma_chan_b)) {
        dma_channel_acknowledge_irq0(dma_chan_b);
        buffer_B_ready = true;
        dma_channel_set_write_addr(dma_chan_b, buffer_B, false);
        dma_channel_set_trans_count(dma_chan_b, BUFFER_SIZE, false);
    }
}

/* ─────────────────────────────────────────────────────────────────────
 * Khởi tạo DMA ping-pong
 *
 *  DMA A  →  buffer_A  →  chain → DMA B
 *  DMA B  →  buffer_B  →  chain → DMA A
 *
 *  DMA_SIZE_8 : mỗi lần transfer = 1 BYTE
 *               khớp với PIO autopush 8 bit → RX FIFO chứa 1 byte/entry
 *
 *  read  addr : &pio->rxf[sm]  (byte thấp nhất của FIFO 32-bit)
 *               Khi DMA_SIZE_8, SDK tự trỏ vào byte [0] của rxf[sm]
 * ───────────────────────────────────────────────────────────────────── */
static void dma_init(PIO pio, uint sm) {
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);

    /* ── DMA A ── */
    dma_channel_config ca = dma_channel_get_default_config(dma_chan_a);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_8);            // ← 8-bit
    channel_config_set_read_increment(&ca,  false);                    // đọc mãi từ FIFO
    channel_config_set_write_increment(&ca, true);                     // ghi tiến trong buffer
    channel_config_set_dreq(&ca, pio_get_dreq(pio, sm, false));        // DREQ = RX FIFO không rỗng
    channel_config_set_chain_to(&ca, dma_chan_b);                      // xong → kích B

    dma_channel_configure(
        dma_chan_a, &ca,
        buffer_A,           // write addr
        &pio->rxf[sm],      // read addr  (PIO RX FIFO)
        BUFFER_SIZE,        // BUFFER_SIZE lần × 1 byte
        false               // chưa start
    );

    /* ── DMA B ── */
    dma_channel_config cb = dma_channel_get_default_config(dma_chan_b);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_8);            // ← 8-bit
    channel_config_set_read_increment(&cb,  false);
    channel_config_set_write_increment(&cb, true);
    channel_config_set_dreq(&cb, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&cb, dma_chan_a);                      // xong → kích A

    dma_channel_configure(
        dma_chan_b, &cb,
        buffer_B,
        &pio->rxf[sm],
        BUFFER_SIZE,
        false
    );

    /* Bật IRQ0 cho cả hai channel */
    dma_channel_set_irq0_enabled(dma_chan_a, true);
    dma_channel_set_irq0_enabled(dma_chan_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    /* Khởi động vòng ping-pong bằng cách start DMA A */
    dma_channel_start(dma_chan_a);
}

/* ─────────────────────────────────────────────────────────────────────
 * Khởi tạo PIO
 * ───────────────────────────────────────────────────────────────────── */
static void pio_setup(PIO pio, uint sm, uint pin) {
    uint offset = pio_add_program(pio, &read_gpio_program);
    read_gpio_program_init(pio, sm, offset, pin);  // hàm từ blink.pio.h
    pio_sm_set_clkdiv(pio, sm, calc_clkdiv(SAMPLE_FREQ_HZ));
    pio_sm_set_enabled(pio, sm, true);
}

/* ─────────────────────────────────────────────────────────────────────
 * Gửi buffer qua USB CDC
 *
 * buffer là mảng uint8_t, mỗi byte = 1 sample:
 *   bit0 = GPIO_PIN
 *   bit1 = GPIO_PIN+1
 * ───────────────────────────────────────────────────────────────────── */
static inline void usb_send(const uint8_t *buf, size_t len) {
    fwrite(buf, 1, len, stdout);
    fflush(stdout);
}

/* ─────────────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────────────── */
int main(void) {
    stdio_init_all();

    // Bật LED báo hiệu đang khởi động
    const uint LED = 25;
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    // Chờ USB host kết nối
    while (!stdio_usb_connected()) { sleep_ms(10); }

    gpio_put(LED, 0);   // tắt LED khi USB sẵn sàng

    PIO  pio = pio0;
    uint sm  = 0;

    pio_setup(pio, sm, GPIO_PIN);
    dma_init(pio, sm);

    while (true) {
        /*
         * CPU chờ cờ IRQ. Khi DMA A hoàn thành BUFFER_SIZE byte:
         *   1. IRQ set buffer_A_ready = true
         *   2. IRQ reset DMA A write_addr về đầu buffer_A
         *   3. DMA B (đã được chain) đang chạy song song
         *   4. CPU gửi buffer_A qua USB trong khi DMA B đang fill buffer_B
         */
        if (buffer_A_ready) {
            buffer_A_ready = false;
            usb_send(buffer_A, BUFFER_SIZE);
        }

        if (buffer_B_ready) {
            buffer_B_ready = false;
            usb_send(buffer_B, BUFFER_SIZE);
        }

        tight_loop_contents();
    }

    return 0;
}