#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

// dispaly
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "lvgl.h"

// freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* LCD Size */
#define DISP_WIDTH  240
#define DISP_HEIGHT 135

/* LCD Pins */
#define PIN_NUM_CS   5
#define PIN_NUM_DC   16
#define PIN_NUM_RST  23
#define PIN_NUM_BL   4
#define PIN_NUM_MOSI 19
#define PIN_NUM_SCK  18
#define PIN_NUM_MISO -1
#define LCD_HOST SPI2_HOST

// Bit number used to represent command and parameter
#define LCD_CMD_BITS     8
#define LCD_PARAM_BITS   8

#define LVGL_TICK_PERIOD_MS 2
#define CLOCK_SPEED 20 * 1000 * 1000

#define LVGL_BUFFER_SIZE        DISP_HEIGHT * 20 * sizeof(lv_color_t)

static void increase_lvgl_tick(void)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lv_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    

    lv_display_flush_ready(display);
}

static esp_err_t lcd_init(void)
{
    /* initialise GPIO */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BL, 1)); // turn on backlight
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_DC, 1)); // data mode
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_RST, 1)); // not reset

    /* initialise SPI bus */
    ESP_LOGI("APP_MAIN", "Initialising SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_HEIGHT * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature

    /* LCD io */
    ESP_LOGI("APP_MAIN", "Configuring LCD panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = CLOCK_SPEED,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) 1, &io_config, &io_handle));

    /* LCD panel*/
    ESP_LOGI("APP_MAIN", "Configuring LCD panel");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .flags = { 
            .reset_active_high = 0 
        }
    };

    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_LOGI("APP_MAIN", "Resetting LCD panel");
    esp_lcd_panel_reset(panel_handle);
    ESP_LOGI("APP_MAIN", "Initialising LCD panel");
    esp_lcd_panel_init(panel_handle);

    /* turn on display */
    ESP_LOGI("APP_MAIN", "Turning on display");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, 1));

    return ESP_OK;
}


static esp_err_t lvgl_init(void)
{
    /* initialise LVGL */
    lv_init();

    /* tick interface*/
    ESP_LOGI("APP_MAIN", "Initialising LVGL tick interface");

    /* create a screen */
    lv_display_t *disp = lv_display_create(DISP_WIDTH, DISP_HEIGHT);

    /* allocate draw buffer */
    lv_color_t *buf1 = heap_caps_malloc(LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* flush callback */

    return ESP_OK;

}

void app_main(void)
{
    printf("Hello, world!\n");

    /* initialise LCD */
    ESP_LOGI("APP_MAIN", "Initialising LCD");
    ESP_ERROR_CHECK(lcd_init());

    /* initialise LVGL */
    ESP_LOGI("APP_MAIN", "Initialising LVGL");
    ESP_ERROR_CHECK(lvgl_init());

}