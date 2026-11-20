/**
 * @file    lcd_st7735.c
 * @brief   1.8 寸 TFT LCD (ST7735S, ZJY180S0800TG01, 128x160 竖屏) 非阻塞 SPI1 驱动实现
 * @details
 * 分块架构：
 *   1. SPI1 (10 MHz, Mode 0, 单线只发 PB3 SCK / PB5 MOSI) 与控制 GPIO (PD10..13) 底层收发
 *   2. ST7735S 竖屏初始化与像素偏移补偿 (ZJY180S0800TG01 竖屏需 x+2, y+1 显存窗偏移)
 *   3. 固定点数格式化辅助函数（避免 printf 浮点栈开销，确保 Release -Werror 零警告）
 *   4. 脏标记对比 + 轮询单槽位分片刷新引擎（每次至多发送 1 个字段，绝不阻塞 200 Hz 主控）
 */

#include "lcd_st7735.h"
#include "lcd_font.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * 1. 竖屏分辨率 (128x160)、引脚操作宏与 RGB565 工业仪表配色定义
 * ============================================================================ */

#define LCD_W       128U
#define LCD_H       160U
#define LCD_X_OFF   2U
#define LCD_Y_OFF   1U

#define COL_BG      0x0842U  /* 深靛蓝夜色背景 */
#define COL_BAR     0x1127U  /* 顶部标题栏深蓝灰 */
#define COL_WELL    0x18E5U  /* 进度条底槽灰蓝 */
#define COL_DIV     0x29AAU  /* 区域分割细线 */
#define COL_TEXT    0xFFFFU  /* 纯白主文字 */
#define COL_MUTED   0x9CD3U  /* 浅灰标签 */
#define COL_CYAN    0x07FFU  /* 亮青：红外电压 / 左偏指示 */
#define COL_GREEN   0x07E0U  /* 翠绿：居中 / 正常正转 */
#define COL_AMBER   0xFD20U  /* 琥珀橙：硬件仿真标记 / 右偏指示 */
#define COL_RED     0xF800U  /* 警示红：大偏离 / 反转 / 故障 */

#define LCD_RES_LOW()   (GPIOD->BSRR = (uint32_t)GPIO_PIN_10 << 16U)
#define LCD_RES_HIGH()  (GPIOD->BSRR = GPIO_PIN_10)
#define LCD_CS_LOW()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U)
#define LCD_CS_HIGH()   (GPIOD->BSRR = GPIO_PIN_11)
#define LCD_DC_CMD()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U)
#define LCD_DC_DATA()   (GPIOD->BSRR = GPIO_PIN_12)
#define LCD_BL_OFF()    (GPIOD->BSRR = (uint32_t)GPIO_PIN_13 << 16U)
#define LCD_BL_ON()     (GPIOD->BSRR = GPIO_PIN_13)

/* ============================================================================
 * 2. SPI1 硬件底层发送与 ST7735S 显存窗口原语
 * ============================================================================ */

static void spi1_write_u8(uint8_t byte) {
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
    *(__IO uint8_t *)&SPI1->DR = byte;
}

static void spi1_wait_idle(void) {
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {}
    while ((SPI1->SR & SPI_SR_BSY) != 0U) {}
}

static void lcd_write_cmd(uint8_t cmd) {
    LCD_DC_CMD();
    LCD_CS_LOW();
    spi1_write_u8(cmd);
    spi1_wait_idle();
    LCD_CS_HIGH();
    LCD_DC_DATA();
}

static void lcd_write_data8(uint8_t data) {
    LCD_DC_DATA();
    LCD_CS_LOW();
    spi1_write_u8(data);
    spi1_wait_idle();
    LCD_CS_HIGH();
}

static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint16_t xs = x1 + LCD_X_OFF, xe = x2 + LCD_X_OFF;
    uint16_t ys = y1 + LCD_Y_OFF, ye = y2 + LCD_Y_OFF;
    lcd_write_cmd(0x2AU);
    LCD_DC_DATA();
    LCD_CS_LOW();
    spi1_write_u8((uint8_t)(xs >> 8)); spi1_write_u8((uint8_t)(xs & 0xFFU));
    spi1_write_u8((uint8_t)(xe >> 8)); spi1_write_u8((uint8_t)(xe & 0xFFU));
    spi1_wait_idle();
    LCD_CS_HIGH();

    lcd_write_cmd(0x2BU);
    LCD_DC_DATA();
    LCD_CS_LOW();
    spi1_write_u8((uint8_t)(ys >> 8)); spi1_write_u8((uint8_t)(ys & 0xFFU));
    spi1_write_u8((uint8_t)(ye >> 8)); spi1_write_u8((uint8_t)(ye & 0xFFU));
    spi1_wait_idle();
    LCD_CS_HIGH();

    lcd_write_cmd(0x2CU);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (w == 0U || h == 0U || x >= LCD_W || y >= LCD_H) return;
    if (x + w > LCD_W) w = (uint16_t)(LCD_W - x);
    if (y + h > LCD_H) h = (uint16_t)(LCD_H - y);
    lcd_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);
    uint32_t total = (uint32_t)w * (uint32_t)h;
    LCD_DC_DATA();
    LCD_CS_LOW();
    for (uint32_t i = 0; i < total; ++i) {
        spi1_write_u8(hi);
        spi1_write_u8(lo);
    }
    spi1_wait_idle();
    LCD_CS_HIGH();
}

static void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg, bool size_16) {
    uint8_t idx = (ch >= ' ' && ch <= '~') ? (uint8_t)(ch - ' ') : 0U;
    uint16_t w = size_16 ? 8U : 6U;
    uint16_t h = size_16 ? 16U : 12U;
    if (x + w > LCD_W || y + h > LCD_H) return;
    const uint8_t *glyph = size_16 ? k_font_8x16[idx] : k_font_6x12[idx];
    uint8_t fhi = (uint8_t)(fg >> 8), flo = (uint8_t)(fg & 0xFFU);
    uint8_t bhi = (uint8_t)(bg >> 8), blo = (uint8_t)(bg & 0xFFU);
    lcd_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    LCD_DC_DATA();
    LCD_CS_LOW();
    for (uint16_t row = 0; row < h; ++row) {
        uint8_t bits = glyph[row];
        for (uint16_t col = 0; col < w; ++col) {
            if (bits & (1U << col)) {
                spi1_write_u8(fhi); spi1_write_u8(flo);
            } else {
                spi1_write_u8(bhi); spi1_write_u8(blo);
            }
        }
    }
    spi1_wait_idle();
    LCD_CS_HIGH();
}

static void lcd_draw_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, bool size_16) {
    uint16_t step = size_16 ? 8U : 6U;
    for (const char *p = s; *p != '\0'; ++p) {
        if (x + step > LCD_W) break;
        lcd_draw_char(x, y, *p, fg, bg, size_16);
        x = (uint16_t)(x + step);
    }
}

/* ============================================================================
 * 3. 定点格式化与状态缓存（消除冗余 SPI 刷屏，确保 200 Hz 主环零抖动）
 * ============================================================================ */

typedef struct {
    bool valid;
    char mode_tag[12];
    char ir_volt[4][6];
    uint8_t ir_bar[4];
    uint16_t ir_col[4];
    char pid_e_big[8];
    uint16_t pid_e_col;
    int8_t pid_bar_px;
    char pid_err_small[11];
    char spd_str[2][8];
    uint16_t spd_col[2];
} LcdSnapshot;

static LcdSnapshot prev_snap;
static uint32_t last_slot_ms;
static uint8_t next_slot;
static float lcd_adc_vref = 3.300f;

/** @brief 格式化电压为固定 5 字符 "X.XXV" */
static void fmt_volt_5(char out[6], float v) {
    if (!isfinite(v) || v < 0.0f) v = 0.0f;
    if (v > 9.99f) v = 9.99f;
    int32_t cv = (int32_t)(v * 100.0f + 0.5f);
    out[0] = (char)('0' + (cv / 100) % 10);
    out[1] = '.';
    out[2] = (char)('0' + (cv / 10) % 10);
    out[3] = (char)('0' + cv % 10);
    out[4] = 'V';
    out[5] = '\0';
}

/** @brief 格式化 PID_E 为固定 6 字符 "X.XXXV" */
static void fmt_pid_e_6(char out[8], float v) {
    if (!isfinite(v) || v < 0.0f) v = 0.0f;
    if (v > 9.999f) v = 9.999f;
    int32_t mv = (int32_t)(v * 1000.0f + 0.5f);
    out[0] = (char)('0' + (mv / 1000) % 10);
    out[1] = '.';
    out[2] = (char)('0' + (mv / 100) % 10);
    out[3] = (char)('0' + (mv / 10) % 10);
    out[4] = (char)('0' + mv % 10);
    out[5] = 'V';
    out[6] = '\0';
}

/** @brief 格式化相对中点误差为固定 9 字符 "e:+X.XXXV" */
static void fmt_err_9(char out[11], float e) {
    if (!isfinite(e)) e = 0.0f;
    if (e > 2.50f) e = 2.50f;
    if (e < -2.50f) e = -2.50f;
    int32_t mv = (int32_t)(e * 1000.0f + (e >= 0.0f ? 0.5f : -0.5f));
    char sign = mv < 0 ? '-' : '+';
    int32_t amv = mv < 0 ? -mv : mv;
    out[0] = 'e'; out[1] = ':'; out[2] = sign;
    out[3] = (char)('0' + (amv / 1000) % 10);
    out[4] = '.';
    out[5] = (char)('0' + (amv / 100) % 10);
    out[6] = (char)('0' + (amv / 10) % 10);
    out[7] = (char)('0' + amv % 10);
    out[8] = 'V';
    out[9] = '\0';
}

/** @brief 格式化轮速为固定 6 字符右对齐定点数，如 "+120.4" / "  +0.0" */
static void fmt_speed_6(char out[8], float v_mm_s) {
    if (!isfinite(v_mm_s)) v_mm_s = 0.0f;
    if (v_mm_s > 999.9f) v_mm_s = 999.9f;
    if (v_mm_s < -999.9f) v_mm_s = -999.9f;
    int32_t tenths = (int32_t)(v_mm_s * 10.0f + (v_mm_s >= 0.0f ? 0.5f : -0.5f));
    char sign = tenths < 0 ? '-' : '+';
    int32_t at = tenths < 0 ? -tenths : tenths;
    int32_t whole = at / 10;
    int32_t frac = at % 10;
    for (unsigned i = 0; i < 6U; ++i) out[i] = ' ';
    out[6] = '\0';
    out[5] = (char)('0' + frac);
    out[4] = '.';
    out[3] = (char)('0' + (whole % 10));
    int pos = 2;
    if (whole >= 10) { out[pos--] = (char)('0' + ((whole / 10) % 10)); }
    if (whole >= 100) { out[pos--] = (char)('0' + ((whole / 100) % 10)); }
    out[pos] = sign;
}

static const char *compact_state_tag(LfState s) {
    switch (s) {
    case LF_STOPPED:       return "STOP ";
    case LF_CALIBRATING:   return "IRCAL";
    case LF_ARMING:        return "ARM  ";
    case LF_TRACKING:      return "RUN  ";
    case LF_LOST_BRAKE:
    case LF_SEARCHING:     return "SRCH ";
    case LF_STOPPING:      return "BRK  ";
    case LF_FAULT:         return "FAULT";
    case LF_OBSTACLE_STOP: return "OBST ";
    case LF_PROBING:       return "PROBE";
    case LF_MOTOR_CAL:     return "MCAL ";
    default:               return "UNK  ";
    }
}

static void draw_ir_bar(uint16_t x, uint16_t y, uint8_t fill_px, uint16_t color) {
    if (fill_px > 66U) fill_px = 66U;
    if (fill_px > 0U) lcd_fill_rect(x, y, fill_px, 8U, color);
    if (fill_px < 66U) lcd_fill_rect((uint16_t)(x + fill_px), y, (uint16_t)(66U - fill_px), 8U, COL_WELL);
}

static void draw_balance_needle(int8_t offset_px, uint16_t color) {
    /* 竖屏中心线 x=64, 总宽 [-55..+55] px, 框内部区域 [8..120, y=111..117] */
    lcd_fill_rect(8U, 111U, 113U, 7U, COL_WELL);
    lcd_fill_rect(64U, 110U, 1U, 9U, COL_DIV);
    if (offset_px > 55) offset_px = 55;
    if (offset_px < -55) offset_px = -55;
    if (offset_px >= 0) {
        uint16_t w = (uint16_t)(offset_px < 2 ? 2 : offset_px);
        lcd_fill_rect(64U, 112U, w, 5U, color);
    } else {
        uint16_t w = (uint16_t)(-offset_px < 2 ? 2 : -offset_px);
        lcd_fill_rect((uint16_t)(64 - (int)w), 112U, w, 5U, color);
    }
}

/* ============================================================================
 * 4. 屏幕初始化（含 ZJY180S0800TG01 竖屏参数配置）与分片刷新服务
 * ============================================================================ */

static void lcd_draw_static_frame(void) {
    lcd_fill_rect(0U, 0U, LCD_W, LCD_H, COL_BG);
    lcd_fill_rect(0U, 0U, LCD_W, 17U, COL_BAR);
    lcd_draw_text(3U, 3U, "IR/PID MON", COL_TEXT, COL_BAR, false);
    lcd_fill_rect(0U, 17U, LCD_W, 1U, COL_DIV);

    /* 四路红外区行标签 (物理顺序 L2 -> L1 -> R1 -> R2，横向 66px 进度条 + 右侧电压) */
    static const char *const ir_lbls[4] = {"L2", "L1", "R1", "R2"};
    for (uint16_t ch = 0; ch < 4U; ++ch) {
        uint16_t ry = (uint16_t)(21U + ch * 14U);
        lcd_draw_text(4U, ry, ir_lbls[ch], COL_MUTED, COL_BG, false);
    }
    lcd_fill_rect(0U, 77U, LCD_W, 1U, COL_DIV);

    /* 硬件 PID_E 区静态框架与零中点光标槽 */
    lcd_draw_text(4U, 80U, "PID_E", COL_MUTED, COL_BG, false);
    lcd_fill_rect(7U, 110U, 115U, 1U, COL_DIV);
    lcd_fill_rect(7U, 118U, 115U, 1U, COL_DIV);
    lcd_fill_rect(7U, 110U, 1U, 9U, COL_DIV);
    lcd_fill_rect(121U, 110U, 1U, 9U, COL_DIV);
    draw_balance_needle(0, COL_GREEN);
    lcd_fill_rect(0U, 121U, LCD_W, 1U, COL_DIV);

    /* 左右轮实际转速区静态标签 */
    lcd_fill_rect(64U, 123U, 1U, 35U, COL_DIV);
    lcd_draw_text(4U, 124U, "vL mm/s", COL_MUTED, COL_BG, false);
    lcd_draw_text(69U, 124U, "vR mm/s", COL_MUTED, COL_BG, false);
}

void lcd_st7735_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* 初始化控制脚默认安全电平：CS=HIGH, DC=HIGH, RES=HIGH, BL=OFF */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_InitTypeDef pin = {
        .Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = 0U
    };
    HAL_GPIO_Init(GPIOD, &pin);

    pin.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pin.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &pin);

    /* 配置 SPI1：80 MHz / 8 = 10 MHz, 8位数据, 单线只发, 软件片选, Mode 0 (上升沿采样) */
    SPI1->CR1 = 0U;
    SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_MSTR;
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);

    /* ST7735S 硬件复位与 ZJY180S0800TG01 官方寄存器初始化序列 */
    LCD_RES_LOW();
    HAL_Delay(60U);
    LCD_RES_HIGH();
    HAL_Delay(120U);

    lcd_write_cmd(0x11U); /* Sleep Out */
    HAL_Delay(120U);

    lcd_write_cmd(0xB1U); lcd_write_data8(0x05U); lcd_write_data8(0x3CU); lcd_write_data8(0x3CU);
    lcd_write_cmd(0xB2U); lcd_write_data8(0x05U); lcd_write_data8(0x3CU); lcd_write_data8(0x3CU);
    lcd_write_cmd(0xB3U); lcd_write_data8(0x05U); lcd_write_data8(0x3CU); lcd_write_data8(0x3CU);
    lcd_write_data8(0x05U); lcd_write_data8(0x3CU); lcd_write_data8(0x3CU);
    lcd_write_cmd(0xB4U); lcd_write_data8(0x03U); /* Dot inversion */

    lcd_write_cmd(0xC0U); lcd_write_data8(0x28U); lcd_write_data8(0x08U); lcd_write_data8(0x04U);
    lcd_write_cmd(0xC1U); lcd_write_data8(0xC0U);
    lcd_write_cmd(0xC2U); lcd_write_data8(0x0DU); lcd_write_data8(0x00U);
    lcd_write_cmd(0xC3U); lcd_write_data8(0x8DU); lcd_write_data8(0x2AU);
    lcd_write_cmd(0xC4U); lcd_write_data8(0x8DU); lcd_write_data8(0xEEU);
    lcd_write_cmd(0xC5U); lcd_write_data8(0x1AU); /* VCOM */

    lcd_write_cmd(0x36U); lcd_write_data8(0xC0U); /* MADCTL: 128x160 默认正竖屏 (0xC0 含上下与左右180°同步正转，RGB 顺序) */
    lcd_write_cmd(0x3AU); lcd_write_data8(0x05U); /* COLMOD: 16-bit RGB565 */

    static const uint8_t gm_pos[16] = {
        0x04U, 0x22U, 0x07U, 0x0AU, 0x2EU, 0x30U, 0x25U, 0x2AU,
        0x28U, 0x26U, 0x2EU, 0x3AU, 0x00U, 0x01U, 0x03U, 0x13U
    };
    static const uint8_t gm_neg[16] = {
        0x04U, 0x16U, 0x06U, 0x0DU, 0x2DU, 0x26U, 0x23U, 0x27U,
        0x27U, 0x25U, 0x2DU, 0x3BU, 0x00U, 0x01U, 0x04U, 0x13U
    };
    lcd_write_cmd(0xE0U);
    for (unsigned i = 0; i < 16U; ++i) lcd_write_data8(gm_pos[i]);
    lcd_write_cmd(0xE1U);
    for (unsigned i = 0; i < 16U; ++i) lcd_write_data8(gm_neg[i]);

    lcd_draw_static_frame();
    lcd_write_cmd(0x29U); /* Display On */
    LCD_BL_ON();

    memset(&prev_snap, 0, sizeof(prev_snap));
    last_slot_ms = HAL_GetTick();
    next_slot = 0U;
}

void lcd_st7735_set_flip(bool flip_180) {
    lcd_write_cmd(0x36U);
    lcd_write_data8(flip_180 ? 0x00U : 0xC0U);
    memset(&prev_snap, 0, sizeof(prev_snap));
    lcd_draw_static_frame();
}

void lcd_st7735_set_vref(float vref_v) {
    if (isfinite(vref_v) && vref_v >= 1.80f && vref_v <= 3.60f) {
        lcd_adc_vref = vref_v;
        prev_snap.valid = false;
    }
}

void lcd_st7735_service(const LfController *c, uint32_t now_ms) {
    if (!c) return;
    if ((uint32_t)(now_ms - last_slot_ms) < 12U) return;
    last_slot_ms = now_ms;

    uint8_t slot = next_slot;
    next_slot = (uint8_t)((next_slot + 1U) % 9U);

    if (slot == 0U) {
        char tag[12];
        const char *pfx = c->cfg.hw.enable ? "HWE " : "MCU ";
        const bool uncal_stop = (c->state == LF_STOPPED && !c->cal.valid && !c->cfg.hw.enable);
        const char *st = uncal_stop ? "!CAL " : compact_state_tag(c->state);
        for (unsigned i = 0; i < 4U; ++i) tag[i] = pfx[i];
        for (unsigned i = 0; i < 5U; ++i) tag[4U + i] = st[i];
        tag[9] = '\0';
        uint16_t col = c->state == LF_FAULT ? COL_RED :
                       (uncal_stop || c->cfg.hw.enable ? COL_AMBER :
                       (c->state == LF_TRACKING ? COL_GREEN : COL_CYAN));
        if (!prev_snap.valid || strcmp(prev_snap.mode_tag, tag) != 0) {
            memcpy(prev_snap.mode_tag, tag, sizeof(tag));
            lcd_draw_text(71U, 3U, tag, col, COL_BAR, false);
        }
    } else if (slot >= 1U && slot <= 4U) {
        unsigned ch = (unsigned)(slot - 1U);
        float v = (float)c->raw[ch] * (lcd_adc_vref / 4095.0f);
        char vtxt[6];
        fmt_volt_5(vtxt, v);
        /* 进度条严格与物理电压成正比（电压越高条越长、电压越低条越短）；标定见黑时变绿提示 */
        float q = (lcd_adc_vref > 0.5f) ? (v / lcd_adc_vref) : 0.0f;
        if (q < 0.0f) q = 0.0f;
        if (q > 1.0f) q = 1.0f;
        uint8_t bar = (uint8_t)(q * 66.0f + 0.5f);
        uint16_t bcol = (c->cal.valid && c->darkness[ch] >= 0.45f) ? COL_GREEN : COL_CYAN;
        uint16_t ry = (uint16_t)(21U + ch * 14U);
        if (!prev_snap.valid || prev_snap.ir_bar[ch] != bar || prev_snap.ir_col[ch] != bcol) {
            prev_snap.ir_bar[ch] = bar;
            prev_snap.ir_col[ch] = bcol;
            draw_ir_bar(20U, (uint16_t)(ry + 2U), bar, bcol);
        }
        if (!prev_snap.valid || strcmp(prev_snap.ir_volt[ch], vtxt) != 0) {
            memcpy(prev_snap.ir_volt[ch], vtxt, sizeof(vtxt));
            lcd_draw_text(91U, ry, vtxt, COL_CYAN, COL_BG, false);
        }
    } else if (slot == 5U) {
        float e = c->hw_pid_e - c->cfg.hw.v_mid;
        char btxt[8];
        fmt_pid_e_6(btxt, c->hw_pid_e);
        float ae = fabsf(e);
        uint16_t pcol = ae < 0.08f ? COL_GREEN : (e < 0.0f ? COL_CYAN : COL_AMBER);
        if (ae > 0.70f) pcol = COL_RED;
        int8_t needle = (int8_t)(e * 55.0f + (e >= 0.0f ? 0.5f : -0.5f));
        if (!prev_snap.valid || strcmp(prev_snap.pid_e_big, btxt) != 0 || prev_snap.pid_e_col != pcol) {
            memcpy(prev_snap.pid_e_big, btxt, sizeof(btxt));
            prev_snap.pid_e_col = pcol;
            lcd_draw_text(40U, 93U, btxt, pcol, COL_BG, true);
        }
        if (!prev_snap.valid || prev_snap.pid_bar_px != needle) {
            prev_snap.pid_bar_px = needle;
            draw_balance_needle(needle, pcol);
        }
    } else if (slot == 6U) {
        float e = c->hw_pid_e - c->cfg.hw.v_mid;
        char etxt[11];
        fmt_err_9(etxt, e);
        if (!prev_snap.valid || strcmp(prev_snap.pid_err_small, etxt) != 0) {
            memcpy(prev_snap.pid_err_small, etxt, sizeof(etxt));
            lcd_draw_text(68U, 80U, etxt, COL_TEXT, COL_BG, false);
        }
    } else {
        unsigned w = (unsigned)(slot - 7U);
        float spd = c->measured_mm_s[w];
        char stxt[8];
        fmt_speed_6(stxt, spd);
        uint16_t scol = spd < -5.0f ? COL_RED : (spd > 10.0f ? COL_GREEN : COL_TEXT);
        uint16_t sx = (w == 0U) ? 8U : 72U;
        if (!prev_snap.valid || strcmp(prev_snap.spd_str[w], stxt) != 0 || prev_snap.spd_col[w] != scol) {
            memcpy(prev_snap.spd_str[w], stxt, sizeof(stxt));
            prev_snap.spd_col[w] = scol;
            lcd_draw_text(sx, 139U, stxt, scol, COL_BG, true);
        }
        prev_snap.valid = true;
    }
}
