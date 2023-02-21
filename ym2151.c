#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32/rom/ets_sys.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h> // esp-idf v5 (platform = espressif32@6.0.1)
#include "ym2151.h"

#define SAMPLE_RATE 55930 // 3579545Hz / 2 / 32 = 55930.390625 なので少し小さい

#define GPIO_TO_IC GPIO_NUM_26
#define GPIO_TO_A0 GPIO_NUM_27
#define GPIO_TO_WR GPIO_NUM_14

#define GPIO_TO_D0 GPIO_NUM_4
#define GPIO_TO_D1 GPIO_NUM_16
#define GPIO_TO_D2 GPIO_NUM_17
#define GPIO_TO_D3 GPIO_NUM_18
#define GPIO_TO_D4 GPIO_NUM_19
#define GPIO_TO_D5 GPIO_NUM_21
#define GPIO_TO_D6 GPIO_NUM_22
#define GPIO_TO_D7 GPIO_NUM_23

i2s_chan_handle_t tx_handle, rx_handle;

void init() {
    // I2S port0 (入力)
    // .slot_cfg: PHILIPSでなくMSBだと動かなかった(EXORとJK-FFで位相がずれるから？)
    i2s_chan_config_t chan_cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg0, NULL, &rx_handle));
    i2s_std_config_t std_cfg0 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .bclk = GPIO_NUM_34,
            .ws = GPIO_NUM_39,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_36,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg0));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    // I2S port1 (出力)
    // .clk_cfg: STD_CLK_DEFAULT_CONFIGの256から128に変更
    i2s_chan_config_t chan_cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg1, &tx_handle, NULL));
    i2s_std_config_t std_cfg1 = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_128 // JK-FFで2分周してYM2151のΦMへ入れる
        },
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_0,
            .bclk = GPIO_NUM_25,
            .ws = GPIO_NUM_32,
            .dout = GPIO_NUM_33,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // YM2151へのピン
    gpio_set_direction(GPIO_TO_D0, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D4, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D5, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D6, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_D7, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_TO_WR);
    gpio_set_direction(GPIO_TO_IC, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_A0, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TO_WR, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_TO_A0, 1);
    gpio_set_level(GPIO_TO_WR, 1);
    gpio_set_level(GPIO_TO_IC, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

	gpio_set_level(GPIO_TO_IC, 0);
	vTaskDelay(pdMS_TO_TICKS(2));
	gpio_set_level(GPIO_TO_IC, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/* D0～D7にアドレス/データをセットする */
void setGPIO(uint8_t v) {
    gpio_set_level(GPIO_TO_D0, (v & 0b00000001) ? 1 : 0);
    gpio_set_level(GPIO_TO_D1, (v & 0b00000010) ? 1 : 0);
    gpio_set_level(GPIO_TO_D2, (v & 0b00000100) ? 1 : 0);
    gpio_set_level(GPIO_TO_D3, (v & 0b00001000) ? 1 : 0);
    gpio_set_level(GPIO_TO_D4, (v & 0b00010000) ? 1 : 0);
    gpio_set_level(GPIO_TO_D5, (v & 0b00100000) ? 1 : 0);
    gpio_set_level(GPIO_TO_D6, (v & 0b01000000) ? 1 : 0);
    gpio_set_level(GPIO_TO_D7, (v & 0b10000000) ? 1 : 0);
}

/** YM2151のレジスタaddrにデータdataを書き込む(delayの数値は適当) */
void writeData(uint8_t addr, uint8_t data) {
    // アドレス
    gpio_set_level(GPIO_TO_A0, 0);
    setGPIO(addr);
    ets_delay_us(4);
    gpio_set_level(GPIO_TO_WR, 0);
    ets_delay_us(4);
    gpio_set_level(GPIO_TO_WR, 1);
    ets_delay_us(4);

    // データ
    gpio_set_level(GPIO_TO_A0, 1);
    setGPIO(data);
    ets_delay_us(4);
    gpio_set_level(GPIO_TO_WR, 0);
    ets_delay_us(4);
    gpio_set_level(GPIO_TO_WR, 1);

    gpio_set_level(GPIO_TO_A0, 0); // A0のLEDを消す
    ets_delay_us(24);
}

/* テスト用音色のセット chA, op4だけ音を出す */
void setTestTimbre() {
    writeData(0x20, 0xc7); // Rch, Lchをenable, FL=0, ALG=7(op1～4を並列に)
    writeData(0x28, 0x7a); // OCT=7, NOTE=A
    writeData(0x60+0x00, 127);  // chA, op1のTL(無音127)
    writeData(0x60+0x08, 127);  // chA, op2のTL(無音127)
    writeData(0x60+0x10, 127);  // chA, op3のTL(無音127)
    writeData(0x60+0x18,  0);  // chA, op4のTL(最大0より小さい32)
    writeData(0x80+0x18, 0x1f); // chA, op4のAR
    writeData(0xA0+0x18, 0x1F); // chA, op4のD1R
    writeData(0xC0+0x18, 0x00); // chA, op4のD2R
    writeData(0xE0+0x18, 0x06); // chA, op4のD1L, RR
}

#define SAMPLE_LENGTH 64
#define TICKS_TO_WAIT 100

/* YM2151からSO, SH1, SH2を受け取り、I2S形式に変換し、uda1334aに送り出すタスク */
void ym2151Task() {
    static uint16_t in_buffer[SAMPLE_LENGTH*2]; // YM2151からのデータ LRで*2
    static int16_t out_buffer[SAMPLE_LENGTH*2]; // uda1334aへのデータ
    uint16_t borrow = 0;

    while (true) {
        // 入力(無音時は0x4018 001(無効?)+0000000001(仮数部)+100(指数部)=0x200cが1ビットずれて出てくる)
        size_t read_bytes = 0;
        ESP_ERROR_CHECK(i2s_channel_read(rx_handle, in_buffer,
            SAMPLE_LENGTH * sizeof(uint16_t) * 2, &read_bytes, TICKS_TO_WAIT));

        // 変換
        for (int i=0; i<SAMPLE_LENGTH*2; i++) {
            // (0)1bitずらす(MSBデータをPHILIPSで受けたから)
            uint16_t in = in_buffer[i];
            uint16_t d = borrow | (in>>1);
            borrow = in<<15;
            // (1)bit並びの反転(MSB...LSBをLSB...MSBにする)
            d = (d & 0x00ff)<<8 | (d & 0xff00)>>8;
            d = (d & 0x0f0f)<<4 | (d & 0xf0f0)>>4;
            d = (d & 0x3333)<<2 | (d & 0xcccc)>>2;
            d = (d & 0x5555)<<1 | (d & 0xaaaa)>>1;
            // (2)仮数部
            int16_t mantissa = d << 3; // MSB側に寄せる
            mantissa ^= 0x8000;        // 最上位ビットを反転して2の補数に変換
            mantissa &= 0xffc0;        // 無効部消す(仮数部10bit)
            // (3)指数部
            int expt = d>>13;
            // (4)シフト
            out_buffer[i] = mantissa >> (7-expt);
        }

        // 出力
        size_t written_bytes = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx_handle, out_buffer,
            SAMPLE_LENGTH * sizeof(uint16_t) * 2, &written_bytes, TICKS_TO_WAIT));
    }
}