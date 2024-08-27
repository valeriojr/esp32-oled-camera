#include <driver/i2c.h>
#include <esp_camera.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_psram.h>

#define XPOWERS_CHIP_AXP2101

#include <XPowersLib.h>


#define TAG "floyd-steinberg"
#define BOARD LILYGO_T_CAMERA_S3
#define DISPLAY_WIDTH              128
#define DISPLAY_HEIGHT              64
#define DISPLAY_CMD_BITS           8
#define DISPLAY_PARAM_BITS         8

#if BOARD == LILYGO_T_CAMERA_S3
#define CAMERA_PIN_PWDN (-1)
#define CAMERA_PIN_RESET 39
#define CAMERA_PIN_XCLK 38
#define CAMERA_PIN_SCL 4
#define CAMERA_PIN_SDA 5
#define CAMERA_PIN_D7 9
#define CAMERA_PIN_D6 10
#define CAMERA_PIN_D5 11
#define CAMERA_PIN_D4 13
#define CAMERA_PIN_D3 21
#define CAMERA_PIN_D2 48
#define CAMERA_PIN_D1 47
#define CAMERA_PIN_D0 14
#define CAMERA_PIN_VSYNC 8
#define CAMERA_PIN_HREF 18
#define CAMERA_PIN_PCLK 12
#define CAMERA_XCLK_FREQ_HZ 20000000
#endif


static XPowersPMU pmu;
static uint8_t screen_buffer[DISPLAY_HEIGHT * DISPLAY_WIDTH];
static uint8_t bitmap[DISPLAY_HEIGHT * DISPLAY_WIDTH / 8];


static esp_err_t init_pmu(i2c_port_t port_num, uint8_t addr, int sda, int scl);

static esp_err_t init_camera(pixformat_t pixel_format, framesize_t frame_size, int jpeg_quality, size_t fb_count,
                             camera_fb_location_t fb_location, camera_grab_mode_t grab_mode);

static esp_err_t
init_display(uint32_t dev_addr, esp_lcd_panel_handle_t* panel_handle);

static void dither(uint8_t* src, size_t width, size_t height);

static void to_bitmap(const uint8_t* src, size_t src_width, size_t src_height, uint8_t* dest);

#ifdef __cplusplus
extern "C" {
#endif
void app_main(void) {
    camera_fb_t* frame = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    camera_fb_location_t fb_location = esp_psram_is_initialized() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    ESP_ERROR_CHECK(init_pmu(I2C_NUM_0, AXP2101_SLAVE_ADDRESS, CONFIG_PMU_I2C_SDA, CONFIG_PMU_I2C_SCL));
    ESP_ERROR_CHECK(init_camera(PIXFORMAT_GRAYSCALE, FRAMESIZE_96X96, 0, 2, fb_location, CAMERA_GRAB_LATEST));
    ESP_ERROR_CHECK(init_display(0x3C, &panel_handle));

    while (true) {
        frame = esp_camera_fb_get();

        if (!frame) {
            continue;
        }

        int i, j;
        for (i = 0; i < frame->height && i < DISPLAY_HEIGHT; i++) {
            for (j = 0; j < frame->width && j < DISPLAY_WIDTH; j++) {
                screen_buffer[i * DISPLAY_WIDTH + j] = frame->buf[i * frame->width + j];
            }
        }

        dither(screen_buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        to_bitmap(screen_buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, bitmap);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, bitmap);

        esp_camera_fb_return(frame);
    }
}
#ifdef __cplusplus
};
#endif


static esp_err_t init_pmu(i2c_port_t port_num, uint8_t addr, int sda, int scl) {
    esp_err_t err = ESP_OK;

    pmu.begin(port_num, addr, sda, scl);

    //Turn off not use power channel
    pmu.disableDC2();
    pmu.disableDC3();
    pmu.disableDC4();
    pmu.disableDC5();

    pmu.disableALDO1();
    pmu.disableALDO2();
    pmu.disableALDO3();
    pmu.disableALDO4();
    pmu.disableBLDO1();
    pmu.disableBLDO2();

    pmu.disableCPUSLDO();
    pmu.disableDLDO1();
    pmu.disableDLDO2();


    //ESP32s3 Core VDD
    pmu.setDC3Voltage(3300);
    pmu.enableDC3();

    //Extern 3.3V VDD
    pmu.setDC1Voltage(3300);
    pmu.enableDC1();

    // CAM DVDD  1500~1800
    pmu.setALDO1Voltage(1800);
    // pmu.setALDO1Voltage(1500);
    pmu.enableALDO1();

    // CAM DVDD 2500~2800
    pmu.setALDO2Voltage(2800);
    pmu.enableALDO2();

    // CAM AVDD 2800~3000
    pmu.setALDO4Voltage(3000);
    pmu.enableALDO4();

    // PIR VDD 3300
    pmu.setALDO3Voltage(3300);
    pmu.enableALDO3();

    // OLED VDD 3300
    pmu.setBLDO1Voltage(3300);
    pmu.enableBLDO1();

    // MIC VDD 33000
    pmu.setBLDO2Voltage(3300);
    pmu.enableBLDO2();

    ESP_LOGI(TAG, "DCDC=======================================================================\n");
    ESP_LOGI(TAG, "DC1  : %s   Voltage:%u mV \n", pmu.isEnableDC1() ? "+" : "-", pmu.getDC1Voltage());
    ESP_LOGI(TAG, "DC2  : %s   Voltage:%u mV \n", pmu.isEnableDC2() ? "+" : "-", pmu.getDC2Voltage());
    ESP_LOGI(TAG, "DC3  : %s   Voltage:%u mV \n", pmu.isEnableDC3() ? "+" : "-", pmu.getDC3Voltage());
    ESP_LOGI(TAG, "DC4  : %s   Voltage:%u mV \n", pmu.isEnableDC4() ? "+" : "-", pmu.getDC4Voltage());
    ESP_LOGI(TAG, "DC5  : %s   Voltage:%u mV \n", pmu.isEnableDC5() ? "+" : "-", pmu.getDC5Voltage());
    ESP_LOGI(TAG, "ALDO=======================================================================\n");
    ESP_LOGI(TAG, "ALDO1: %s   Voltage:%u mV\n", pmu.isEnableALDO1() ? "+" : "-", pmu.getALDO1Voltage());
    ESP_LOGI(TAG, "ALDO2: %s   Voltage:%u mV\n", pmu.isEnableALDO2() ? "+" : "-", pmu.getALDO2Voltage());
    ESP_LOGI(TAG, "ALDO3: %s   Voltage:%u mV\n", pmu.isEnableALDO3() ? "+" : "-", pmu.getALDO3Voltage());
    ESP_LOGI(TAG, "ALDO4: %s   Voltage:%u mV\n", pmu.isEnableALDO4() ? "+" : "-", pmu.getALDO4Voltage());
    ESP_LOGI(TAG, "BLDO=======================================================================\n");
    ESP_LOGI(TAG, "BLDO1: %s   Voltage:%u mV\n", pmu.isEnableBLDO1() ? "+" : "-", pmu.getBLDO1Voltage());
    ESP_LOGI(TAG, "BLDO2: %s   Voltage:%u mV\n", pmu.isEnableBLDO2() ? "+" : "-", pmu.getBLDO2Voltage());
    ESP_LOGI(TAG, "CPUSLDO====================================================================\n");
    ESP_LOGI(TAG, "CPUSLDO: %s Voltage:%u mV\n", pmu.isEnableCPUSLDO() ? "+" : "-", pmu.getCPUSLDOVoltage());
    ESP_LOGI(TAG, "DLDO=======================================================================\n");
    ESP_LOGI(TAG, "DLDO1: %s   Voltage:%u mV\n", pmu.isEnableDLDO1() ? "+" : "-", pmu.getDLDO1Voltage());
    ESP_LOGI(TAG, "DLDO2: %s   Voltage:%u mV\n", pmu.isEnableDLDO2() ? "+" : "-", pmu.getDLDO2Voltage());
    ESP_LOGI(TAG, "===========================================================================\n");

    pmu.clearIrqStatus();

    pmu.enableVbusVoltageMeasure();
    pmu.enableBattVoltageMeasure();
    pmu.enableSystemVoltageMeasure();
    pmu.enableTemperatureMeasure();

    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    pmu.disableTSPinMeasure();

    // Disable all interrupts
    pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    pmu.clearIrqStatus();
    // Enable the required interrupt function
    pmu.enableIRQ(
            XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |   //BATTERY
            XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |   //VBUS
            XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |   //POWER KEY
            XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
            // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );

    /*
      The default setting is CHGLED is automatically controlled by the pmu.
    - XPOWERS_CHG_LED_OFF,
    - XPOWERS_CHG_LED_BLINK_1HZ,
    - XPOWERS_CHG_LED_BLINK_4HZ,
    - XPOWERS_CHG_LED_ON,
    - XPOWERS_CHG_LED_CTRL_CHG,
    * */
    pmu.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);

    // Set the precharge charging current
    pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    // Set constant current charge current limit
    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    // Set stop charging termination current
    pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    // Set charge cut-off voltage
    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    // Set the watchdog trigger event type
    // pmu.setWatchdogConfig(XPOWERS_AXP2101_WDT_IRQ_TO_PIN);
    // Set watchdog timeout
    pmu.setWatchdogTimeout(XPOWERS_AXP2101_WDT_TIMEOUT_4S);

    // Enable watchdog to trigger interrupt event
    pmu.enableWatchdog();

    return err;
}


esp_err_t init_camera(pixformat_t pixel_format, framesize_t frame_size, int jpeg_quality, size_t fb_count,
                      camera_fb_location_t fb_location, camera_grab_mode_t grab_mode) {
    esp_err_t err = ESP_OK;
    sensor_t* sensor = NULL;
    camera_config_t camera_config = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = CAMERA_PIN_SDA,
            .pin_sccb_scl = CAMERA_PIN_SCL,
            .pin_d7 = CAMERA_PIN_D7,
            .pin_d6 = CAMERA_PIN_D6,
            .pin_d5 = CAMERA_PIN_D5,
            .pin_d4 = CAMERA_PIN_D4,
            .pin_d3 = CAMERA_PIN_D3,
            .pin_d2 = CAMERA_PIN_D2,
            .pin_d1 = CAMERA_PIN_D1,
            .pin_d0 = CAMERA_PIN_D0,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,
            .xclk_freq_hz = CAMERA_XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,
            .pixel_format = pixel_format,
            .frame_size = frame_size,
            .jpeg_quality = jpeg_quality,
            .fb_count = fb_count,
            .fb_location = fb_location,
            .grab_mode = grab_mode,
    };

    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        return err;
    }

    sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    sensor->set_vflip(sensor, true);

    return err;
}


esp_err_t
init_display(uint32_t dev_addr, esp_lcd_panel_handle_t* panel_handle) {
    esp_err_t err = ESP_OK;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = dev_addr,
            .control_phase_bytes = 1,               // According to SSD1306 datasheet
            .dc_bit_offset = 6,                     // According to SSD1306 datasheet
            .lcd_cmd_bits = DISPLAY_CMD_BITS,   // According to SSD1306 datasheet
            .lcd_param_bits = DISPLAY_PARAM_BITS, // According to SSD1306 datasheet
    };
    esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = -1,
            .bits_per_pixel = 1,
    };

    err = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t) I2C_NUM_0, &io_config, &io_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_new_panel_ssd1306(io_handle, &panel_config, panel_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_reset(*panel_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_init(*panel_handle);
    if (err != ESP_OK) {
        return err;
    }

    return esp_lcd_panel_disp_on_off(*panel_handle, true);
}


void dither(uint8_t* src, size_t width, size_t height) {
    int i, j;

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            int old_value = src[i * width + j];
            int new_value = old_value > 128 ? 255 : 0;
            int error = old_value - new_value;

            src[(i) * width + j] = new_value;

            if (j < width - 1) {
                src[i * width + j + 1] += 7 * error / 16;
            }
            if (i < height - 1) {
                if (j > 0) {
                    src[(i + 1) * width + j - 1] += 3 * error / 16;
                }
                src[(i + 1) * width + j] += 5 * error / 16;
                if (j < width - 1) {
                    src[(i + 1) * width + j + 1] += 1 * error / 16;
                }
            }
        }
    }
}


void to_bitmap(const uint8_t* src, size_t src_width, size_t src_height, uint8_t* dest) {
    int byte = 0;
    int i, j;

    for (i = 0; i < src_height; i += 8) {
        for (j = 0; j < src_width; j++) {
            dest[byte] =
                    (0b10000000 & src[(i + 7) * src_width + j]) |
                    (0b01000000 & src[(i + 6) * src_width + j]) |
                    (0b00100000 & src[(i + 5) * src_width + j]) |
                    (0b00010000 & src[(i + 4) * src_width + j]) |
                    (0b00001000 & src[(i + 3) * src_width + j]) |
                    (0b00000100 & src[(i + 2) * src_width + j]) |
                    (0b00000010 & src[(i + 1) * src_width + j]) |
                    (0b00000001 & src[(i + 0) * src_width + j]);

            byte++;
        }
    }
}
