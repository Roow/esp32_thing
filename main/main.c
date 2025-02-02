#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

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
#define LVGL_TASK_STACK_SIZE    4096
#define LVGL_TASK_PRIORITY      2


static _lock_t lvgl_api_lock;
static esp_lcd_panel_handle_t panel_handle = NULL;

////////////// FUNTIONS //////////////
extern void lv_example_get_started_1(void);

static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lv_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    /* rotate screen */
    esp_lcd_panel_swap_xy(panel_handle, 1);
    esp_lcd_panel_mirror(panel_handle, 1, 0);

    /* add a gap, apparently panel specific */
    esp_lcd_panel_set_gap(panel_handle, 40, 53);

    /* Flush display */
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) lv_display_get_user_data(display);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1));

    /* inverter colour */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, 1));

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map));

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
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .flags = { 
            .reset_active_high = 0 
        }
    };

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
    esp_timer_handle_t lvgl_tick_timer = NULL;
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    /* create a screen */
    ESP_LOGI("APP_MAIN", "Creating LVGL screen");
    lv_display_t *disp = lv_display_create(DISP_WIDTH, DISP_HEIGHT);

    /* allocate draw buffer */
    ESP_LOGI("APP_MAIN", "Allocating draw buffer");
    lv_color_t *buf1 = heap_caps_malloc(LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lv_display_set_buffers(disp, buf1, buf2, LVGL_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* associate panel handle to the display */
    ESP_LOGI("APP_MAIN", "Associating panel handle to display");
    lv_display_set_user_data(disp, panel_handle);

    /* set colour depth */
    ESP_LOGI("APP_MAIN", "Setting colour depth");
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    /* flush callback */
    ESP_LOGI("APP_MAIN", "Setting flush callback");
    lv_display_set_flush_cb(disp, lv_flush_cb);

    ESP_LOGI("APP_MAIN", "LVGL initialised");
    return ESP_OK;

}

static void lvgl_task(void *arg)
{
    ESP_LOGI("APP_MAIN", "Starting LVGL task");
    

    while (1) {
        _lock_acquire(&lvgl_api_lock);
        uint32_t time_till_next = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        uint32_t time_threshold = 1000 / CONFIG_FREERTOS_HZ;
        time_till_next = time_till_next < time_threshold ? time_till_next : time_threshold;
        usleep(time_till_next * 1000);
    }
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

    /* create LVGL task */
    ESP_LOGI("APP_MAIN", "Creating LVGL task");
    xTaskCreate(lvgl_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* create example */
    _lock_acquire(&lvgl_api_lock);
    lv_example_get_started_1();
    _lock_release(&lvgl_api_lock);

}