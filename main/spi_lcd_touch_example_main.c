/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
//CfmOS
#include "esp_elf.h"
#include "private/elf_symbol.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_task_wdt.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "cfmos.h"
#define MOUNT_POINT "/sdcard"
#define CALIBRATION_FILE MOUNT_POINT"/sdcard/calibration.dat"
#define REPEAT_CAL 0 // Set to 1 to force recalibration

#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#include "esp_lcd_ili9341.h"
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#include "esp_lcd_gc9a01.h"
#endif

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_STMPE610
#include "esp_lcd_touch_xpt2046.h"
#endif

static const char *TAG = "example";
#include <errno.h>    // For errno
#include <string.h>   // For strerror()
// Using SPI2 in the example
#define LCD_HOST  SPI3_HOST

    uint16_t calData[5];
    bool calDataOK = false;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           18
#define EXAMPLE_PIN_NUM_MOSI           23
#define EXAMPLE_PIN_NUM_MISO           19
#define EXAMPLE_PIN_NUM_LCD_DC         2
#define EXAMPLE_PIN_NUM_LCD_RST        4
#define EXAMPLE_PIN_NUM_LCD_CS         15
#define EXAMPLE_PIN_NUM_BK_LIGHT       27
#define EXAMPLE_PIN_NUM_TOUCH_CS       5
__attribute__((used)) int foogay()
{
    ESP_LOGI(TAG,"OMG");
    return 0;
}
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              320
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              240
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
esp_lcd_touch_handle_t tp = NULL;
#endif
static SemaphoreHandle_t lvgl_mux = NULL;
void touch_calibrate(lv_disp_t *disp)
{
    FILE* f = fopen(MOUNT_POINT "/calibration.dat", "rb");
    if (f) {
        if (fread((char *)calData, sizeof(uint16_t), 5, f) == 5) {
            calDataOK = true;
        }
        fclose(f);
    }

    if (calDataOK && !REPEAT_CAL) {
    } else {
        lv_obj_t *scr = lv_disp_get_scr_act(disp);

        lv_obj_t *label = lv_label_create(scr);
        lv_label_set_text(label, "Touch the corners as indicated");
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

        uint16_t points[4][2] = {{20, 20}, {EXAMPLE_LCD_H_RES - 20, 20}, {EXAMPLE_LCD_H_RES - 20, EXAMPLE_LCD_V_RES - 20}, {20, EXAMPLE_LCD_V_RES - 20}};
        for (int i = 0; i < 4; i++) {
            lv_obj_t *circle = lv_obj_create(scr);
            lv_obj_set_size(circle, 10, 10);
            lv_obj_set_style_bg_color(circle, lv_color_hex(0xFF0000), 0);
            lv_obj_align(circle, LV_ALIGN_CENTER, points[i][0] - EXAMPLE_LCD_H_RES / 2, points[i][1] - EXAMPLE_LCD_V_RES / 2);

            bool touched = false;
            while (!touched) {
                esp_lcd_touch_read_data(tp);
                uint16_t touchpad_x[1], touchpad_y[1];
                uint8_t touchpad_cnt = 0;
                if (esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1)) {
                    calData[i * 2] = touchpad_x[0];
                    calData[i * 2 + 1] = touchpad_y[0];
                    touched = true;
                    ESP_LOGI(TAG, "Collected point: (%d, %d) -> (%d, %d)", points[i][0], points[i][1], touchpad_x[0], touchpad_y[0]);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            lv_obj_del(circle);
        }

        lv_obj_clean(scr);
        lv_label_set_text(label, "Calibration complete!");

        f = fopen(MOUNT_POINT "/calibration.dat", "wb");
        if (f) {
            fwrite((const unsigned char *)calData, sizeof(uint16_t), 5, f);
            fclose(f);
        }

        calDataOK = true;
    }
}

void loadProgram(lv_event_t * e)
{
        ESP_LOGI(TAG, "Reading ELF file from SD card...");
    FILE *fp = fopen("/sdcard/program.elf", "rb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open ELF file");
    }

    fseek(fp, 0, SEEK_END);
    long elf_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint8_t *elf_buffer[elf_size];

    fread(elf_buffer, 1, elf_size, fp);
    fclose(fp);

    ESP_LOGI(TAG, "Initializing ELF...");
    esp_elf_t elf;
    int ret = esp_elf_init(&elf);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to initialize ELF file errno=%d", ret);
        free(elf_buffer);
    }

    ESP_LOGI(TAG, "Relocating ELF...");
    ret = esp_elf_relocate(&elf, elf_buffer);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to relocate ELF file errno=%d", ret);
        esp_elf_deinit(&elf);
        free(elf_buffer);
    }

    ESP_LOGI(TAG, "Executing ELF...");
    esp_elf_request(&elf, 0, 0, NULL); // Adjust argc and argv as needed

    ESP_LOGI(TAG, "Deinitializing ELF...");
    esp_elf_deinit(&elf);
    free(elf_buffer);
}
#define LV_COLOR_BUFFER_SIZE 240 * 21  // Adjust according to BMP size


// BMP file structure
#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;      // BMP Signature, should be 0x4D42 ('BM')
    uint32_t bfSize;      // File size
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;   // Offset to the start of pixel data
} BMPFileHeader;

/** @brief ESP-IDF public functions symbols look-up table */

typedef struct {
    uint32_t biSize;      // Size of this header (40 bytes)
    int32_t biWidth;      // Width of the image
    int32_t biHeight;     // Height of the image
    uint16_t biPlanes;    // Number of planes (always 1)
    uint16_t biBitCount;  // Bits per pixel (24 for RGB)
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t biXPelsPerMeter;
    int32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BMPInfoHeader;
#pragma pack(pop)
void cfmos(lv_disp_t *disp)
{
    
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    // Create an image object
 FILE *f = fopen("/sdcard/wallpaper/bliss.bmp", "r");
    if (!f) {
    ESP_LOGE(TAG, "Failed to open BMP file: %s (errno: %d)", strerror(errno), errno);
    return;
    }

    // Read BMP file header
    BMPFileHeader fileHeader;
    fread(&fileHeader, sizeof(BMPFileHeader), 1, f);

    if (fileHeader.bfType != 0x4D42) {  // 'BM' in ASCII
        ESP_LOGE(TAG, "Invalid BMP file");
        fclose(f);
        return;
    }

    // Read BMP info header
    BMPInfoHeader infoHeader;
    fread(&infoHeader, sizeof(BMPInfoHeader), 1, f);

    if (infoHeader.biBitCount != 24) {
        ESP_LOGE(TAG, "Only 24-bit BMP is supported");
        fclose(f);
        return;
    }

    int width = infoHeader.biWidth;
    int height = abs(infoHeader.biHeight);  // Height can be negative for top-down images
    ESP_LOGI(TAG, "BMP Width: %d, Height: %d", width, height);

    // Allocate a buffer for pixel data (BMP is BGR, we need RGB565 for LVGL)
    uint16_t *lv_color_buffer = malloc(width * height * sizeof(uint16_t));

    // Move file pointer to the pixel data offset
    fseek(f, fileHeader.bfOffBits, SEEK_SET);

    // Read pixel data row by row
    uint8_t pixel_row[width * 3];
    for (int y = 0; y < height; y++) {
        fread(pixel_row, width * 3, 1, f);

        // Convert BMP pixel row (BGR) to LVGL's RGB565
        for (int x = 0; x < width; x++) {
            uint8_t b = pixel_row[x * 3];
            uint8_t g = pixel_row[x * 3 + 1];
            uint8_t r = pixel_row[x * 3 + 2];

            // Convert RGB888 to RGB565
            lv_color_buffer[y * width + x] =
                ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }
    }

    fclose(f);

    // Create an LVGL image from the buffer
    lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = width;
    img_dsc.header.h = height;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data = (const uint8_t *)lv_color_buffer;
    img_dsc.data_size = width * height * sizeof(uint16_t);

    // Create an image object in LVGL and set the source
    lv_obj_t *img = lv_img_create(scr);
    lv_img_set_src(img, &img_dsc);
    lv_obj_set_pos(img, 0, 21);

    // Free buffer if needed (or keep if it's reused later)
    free(lv_color_buffer);

    ESP_LOGI(TAG, "Creating a styled button with text");
    /*Init the style for the default state*/

    // Set the background color of the screen to black
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&style, 8);
    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_pad_all(&style, 10);

    /*Init the pressed style*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&style_pr, 30);
    lv_style_set_outline_opa(&style_pr, LV_OPA_TRANSP);

    lv_style_set_translate_y(&style_pr, 5);
    lv_style_set_shadow_ofs_y(&style_pr, 3);
    lv_style_set_bg_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 4));

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0};
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);

    lv_style_set_transition(&style_pr, &trans);

    lv_obj_t * btn1 = lv_btn_create(scr);
    lv_obj_remove_style_all(btn1);                          /*Remove the style coming from the theme*/
    lv_obj_add_style(btn1, &style, 0);
    lv_obj_add_style(btn1, &style_pr, LV_STATE_PRESSED);
    lv_obj_set_size(btn1, 21, 21);
    lv_obj_set_pos(btn1, 0, 0);
    //lv_obj_center(btn1);

      lv_obj_add_event_cb(btn1, loadProgram, LV_EVENT_CLICKED, disp);
    lv_obj_t * label = lv_label_create(btn1);
    lv_label_set_text(label, "Load");
    lv_obj_center(label);


}




extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    }
}

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_read_data(drv->user_data);

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        if (calDataOK) {
            uint16_t x = touchpad_x[0];
            uint16_t y = touchpad_y[0];

            // Apply the calibration
            x = (x - calData[0]) * EXAMPLE_LCD_H_RES / (calData[2] - calData[0]);
            y = (y - calData[1]) * EXAMPLE_LCD_V_RES / (calData[4] - calData[1]);

            data->point.x = x;
            data->point.y = y;
        } else {
            data->point.x = touchpad_x[0];
            data->point.y = touchpad_y[0];
        }
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void)
{
    
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

host.max_freq_khz = 800;  // 0.8 MHz for SDSPI

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 13,
        .miso_io_num = 16,
        .sclk_io_num = 14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 33;
slot_config.host_id = SPI2_HOST;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // All done, unmount partition and disable SPI peripheral
    //deinitialize the bus after all devices are removed
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    ESP_LOGI(TAG, "Turn off LCD backlight");

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
        .max_transfer_sz=4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(EXAMPLE_PIN_NUM_TOUCH_CS);
    // Attach the TOUCH to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_STMPE610
    ESP_LOGI(TAG, "Initialize touch controller STMPE610");
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));
#endif // CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_STMPE610
#endif // CONFIG_EXAMPLE_LCD_TOUCH_ENABLED

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
foogay();
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);
#endif


    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
 FILE *f = fopen("/sdcard/wallpaper/bliss.bmp", "r");
     if (!f) {
    ESP_LOGE(TAG, "Failed to open BMP file: %s (errno: %d)", strerror(errno), errno);
    return;
    }
    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        //example_lvgl_demo_ui(disp);
        touch_calibrate(disp);
        cfmos(disp);
        // Release the mutex
        example_lvgl_unlock();
    }

    
}
