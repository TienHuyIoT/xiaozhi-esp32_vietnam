#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "features/music/music_visualizer.h"
#include "assets/lang_config.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <esp_log.h>
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include <esp_timer.h>
#include "esp_io_expander_tca9554.h"
#ifdef CONFIG_TOUCH_PANEL_ENABLE
#include <esp_lcd_touch.h>
#include <esp_lcd_touch_cst816s.h>
#include "display/lcd_touch.h"
#endif
#ifdef CONFIG_SD_CARD_MMC_INTERFACE
#include "sdmmc.h"
#elif defined(CONFIG_SD_CARD_SPI_INTERFACE)
#include "sdspi.h"
#endif

#define TAG "waveshare_lcd_1_85c"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

static const st77916_lcd_init_cmd_t vendor_specific_init_new[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){0x00}, 1, 0},
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0x29, (uint8_t []){0x00}, 1, 0},  
};

class CustomBoard : public WifiBoard {
private:
    Button boot_button_;
    i2c_master_bus_handle_t i2c_bus_;
    esp_io_expander_handle_t io_expander = NULL;
    LcdDisplay* display_;
#ifdef CONFIG_TOUCH_PANEL_ENABLE
    LcdTouch *touch_;
    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_ = nullptr;
#endif

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)I2C_BUS_PORT,
            .sda_io_num = I2C_BUS_SDA_PIN,
            .scl_io_num = I2C_BUS_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void CheckI2CDevice(uint8_t addr, const char *name)
    {
        i2c_master_dev_handle_t dev_handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };

        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle) == ESP_OK)
        {
            uint8_t data;
            if (i2c_master_receive(dev_handle, &data, 1, 100) == ESP_OK)
            {
                ESP_LOGI(TAG, "✓ Found %s at I2C address 0x%02X", name, addr);
            }
            else
            {
                ESP_LOGW(TAG, "✗ Device at 0x%02X (%s) not responding", addr, name);
            }
            i2c_master_bus_rm_device(dev_handle);
        }
        else
        {
            ESP_LOGE(TAG, "✗ Failed to add device at 0x%02X (%s)", addr, name);
        }
    }

    void InitializeTca9554(void)
    {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, IO_EXPANDER_I2C_TCA9554_ADDR, &io_expander);
        if(ret != ESP_OK)
            ESP_LOGE(TAG, "TCA9554 create returned error");                                                                          // 打印引脚状态

        ret = esp_io_expander_set_dir(io_expander, IO_EXPANDER_01_TP_RST | IO_EXPANDER_02_LCD_RST | IO_EXPANDER_03_SDMMC, IO_EXPANDER_OUTPUT);                 // 设置引脚 EXIO0 和 EXIO1 模式为输出
        ESP_ERROR_CHECK(ret);
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_01_TP_RST | IO_EXPANDER_02_LCD_RST | IO_EXPANDER_03_SDMMC, 1);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_01_TP_RST | IO_EXPANDER_02_LCD_RST | IO_EXPANDER_03_SDMMC, 0);                                // 复位 LCD 与 TouchPad
        ESP_ERROR_CHECK(ret);
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_01_TP_RST | IO_EXPANDER_02_LCD_RST | IO_EXPANDER_03_SDMMC, 1);                                // 复位 LCD 与 TouchPad
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(ret);
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");

        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(QSPI_PIN_NUM_LCD_PCLK,
                                                                        QSPI_PIN_NUM_LCD_DATA0,
                                                                        QSPI_PIN_NUM_LCD_DATA1,
                                                                        QSPI_PIN_NUM_LCD_DATA2,
                                                                        QSPI_PIN_NUM_LCD_DATA3,
                                                                        QSPI_LCD_H_RES * 80 * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void Initializest77916Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");

        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = QSPI_PIN_NUM_LCD_CS,               
            .dc_gpio_num = -1,                  
            .spi_mode = 0,                     
            .pclk_hz = 3 * 1000 * 1000,      
            .trans_queue_depth = 10,            
            .on_color_trans_done = NULL,                            
            .user_ctx = NULL,                   
            .lcd_cmd_bits = 32,                 
            .lcd_param_bits = 8,                
            .flags = {                          
            .dc_low_on_data = 0,            
            .octal_mode = 0,                
            .quad_mode = 1,                 
            .sio_mode = 0,                  
            .lsb_first = 0,                 
            .cs_high_active = 0,            
            },                                  
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install ST77916 panel driver");
        
        st77916_vendor_config_t vendor_config = {
            .flags = {
                .use_qspi_interface = 1,
            },
        };
        
        printf("-------------------------------------- Version selection -------------------------------------- \r\n");
        esp_err_t ret;
        int lcd_cmd = 0x04;
        uint8_t register_data[4]; 
        size_t param_size = sizeof(register_data);
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_READ_CMD << 24;  // Use the read opcode instead of write
        ret = esp_lcd_panel_io_rx_param(panel_io, lcd_cmd, register_data, param_size); 
        if (ret == ESP_OK) {
            printf("Register 0x04 data: %02x %02x %02x %02x\n", register_data[0], register_data[1], register_data[2], register_data[3]);
        } else {
            printf("Failed to read register 0x04, error code: %d\n", ret);
        } 
        // panel_io_spi_del(io_handle);
        io_config.pclk_hz = 80 * 1000 * 1000;
        if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io) != ESP_OK) {
            printf("Failed to set LCD communication parameters -- SPI\r\n");
            return ;
        }
        printf("LCD communication parameters are set successfully -- SPI\r\n");
        
        // Check register values and configure accordingly
        if (register_data[0] == 0x00 && register_data[1] == 0x7F && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            // Handle the case where the register data matches this pattern
            printf("Vendor-specific initialization for case 1.\n");
        }
        else if (register_data[0] == 0x00 && register_data[1] == 0x02 && register_data[2] == 0x7F && register_data[3] == 0x7F) {
            // Provide vendor-specific initialization commands if register data matches this pattern
            vendor_config.init_cmds = vendor_specific_init_new;
            vendor_config.init_cmds_size = sizeof(vendor_specific_init_new) / sizeof(st77916_lcd_init_cmd_t);
            printf("Vendor-specific initialization for case 2.\n");
        }
        printf("------------------------------------- End of version selection------------------------------------- \r\n");
 
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18)
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_disp_on_off(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }


#ifdef CONFIG_TOUCH_PANEL_ENABLE
    static void IRAM_ATTR touch_isr_callback(void* arg) {
        CustomBoard *board = static_cast<CustomBoard *>(arg);
        board->NotifyTouchEvent();
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY) {
        if (touch_isr_mux_ != NULL) {
            BaseType_t result = xSemaphoreTake(touch_isr_mux_, timeout);
            return result == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent() {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    void InitializeTouch() {
        ESP_LOGI(TAG, "Initialize touch controller cst816s");
        ESP_LOGI(TAG, "Touch I2C: SDA=%d, SCL=%d, ADDR=0x%02X", TOUCH_I2C_SDA_PIN, TOUCH_I2C_SCL_PIN, TOUCH_I2C_ADDR);
        ESP_LOGI(TAG, "Touch pins: RST=%d, INT=%d", TOUCH_RST_PIN, TOUCH_INT_PIN);
        
        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE("EchoEar", "Failed to create touch semaphore");
        }
        
        // Manual reset of touch controller
        if (TOUCH_RST_PIN != GPIO_NUM_NC) {
            /* Prepare pin for touch controller reset */
            ESP_LOGI(TAG, "Resetting touch controller...");
            const gpio_config_t rst_gpio_config = {
                .pin_bit_mask = BIT64(TOUCH_RST_PIN),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&rst_gpio_config);
            
            gpio_set_level(TOUCH_RST_PIN, 0);  // Reset low
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(TOUCH_RST_PIN, 1);  // Reset high
            vTaskDelay(pdMS_TO_TICKS(200));    // 200ms is a minimum wait for touch controller to boot
            ESP_LOGI(TAG, "Touch controller reset complete");
        }

        if (TOUCH_INT_PIN != GPIO_NUM_NC) {
            const gpio_config_t int_gpio_config = {
                .pin_bit_mask = (1ULL << TOUCH_INT_PIN),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_NEGEDGE
            };
            gpio_config(&int_gpio_config);
            gpio_install_isr_service(0);
            gpio_intr_enable(TOUCH_INT_PIN);
            gpio_isr_handler_add(TOUCH_INT_PIN, touch_isr_callback, this);
        }
        
        // Check I2C devices
        CheckI2CDevice(TOUCH_I2C_ADDR, "cst816s Touch");
        
        // Touch configuration - enable interrupt, use polling mode
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            // TOUCH_RST_PIN already handled above, should not handle reset here by driver 
            // due to timing delay 10ms is very short and causes issues inside driver
            .rst_gpio_num = GPIO_NUM_NC, // TOUCH_RST_PIN
            .int_gpio_num = GPIO_NUM_NC, // TOUCH_INT_PIN
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = DISPLAY_SWAP_XY ? 1U : 0U,
                .mirror_x = DISPLAY_MIRROR_X ? 1U : 0U,
                .mirror_y = DISPLAY_MIRROR_Y ? 1U : 0U,
            },
            // .interrupt_callback = [](esp_lcd_touch_handle_t tp) {
            //     if (tp->config.user_data == NULL) {
            //         ESP_LOGW(TAG, "Touch ISR callback user_data is NULL");
            //         return;
            //     }
            //     CustomBoard *board = static_cast<CustomBoard *>(tp->config.user_data);
            //     board->NotifyTouchEvent();
            // },
            // .user_data = this,
        };
        
        ESP_LOGI(TAG, "Using polling mode (interrupt disabled) for touch detection");
        
        ESP_LOGI(TAG, "Touch config: x_max=%d, y_max=%d, swap_xy=%d, mirror_x=%d, mirror_y=%d",
                tp_cfg.x_max, tp_cfg.y_max, tp_cfg.flags.swap_xy, 
                tp_cfg.flags.mirror_x, tp_cfg.flags.mirror_y);

        // Use FT6x36 driver with custom debug panel IO (creates its own device handle)
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
        tp_io_config.dev_addr = TOUCH_I2C_ADDR;
        tp_io_config.scl_speed_hz = 400 * 1000;  // 400kHz
        
        ESP_LOGI(TAG, "Creating touch I2C panel IO...");
        esp_err_t ret;
        ret = esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch I2C panel IO: %s", esp_err_to_name(ret));
        return;
        }
        
        ESP_LOGI(TAG, "Creating cst816s touch controller...");
        esp_lcd_touch_handle_t tp_ = NULL;
        ret = esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp_);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create cst816s touch controller: %s", esp_err_to_name(ret));
        return;
        }
        ESP_LOGI(TAG, "✅ Touch panel cst816s initialized successfully with custom driver!");

        touch_ = new I2cLcdTouch(tp_, tp_io_handle, 
                            DISPLAY_WIDTH, DISPLAY_HEIGHT, 
                            DISPLAY_SWAP_XY, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        touch_->SetInterruptCallback([this]()->bool {
            return this->WaitForTouchEvent();
        });

        touch_->SetGestureCallback([this](TouchGesture gesture, int16_t x, int16_t y) {
            ESP_LOGI(TAG, "Touch gesture detected: %d at (%d, %d)", static_cast<int>(gesture), x, y);
            switch (gesture) {
                case TOUCH_GESTURE_SWIPE_RIGHT:
                    {
                    ESP_LOGI(TAG, "👉 Swipe RIGHT");
                    music::SourceType source = Application::GetInstance().BuildMusicInfo().source;
                    ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                    if (source == music::SourceType::SD_CARD) {
                        ESP_LOGI(TAG, "Play Next track");
                        auto& app = Application::GetInstance();
                        auto sd_music = app.GetSdMusic();
                        if (sd_music) {
                            sd_music->Stop();
                            sd_music->Next();
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }
                    } else {
                        auto& board = Board::GetInstance();
                        auto backlight = board.GetBacklight();
                        int new_brightness = backlight->brightness();
                        
                        // Swipe right - increase brightness
                        new_brightness += 5;
                        if (new_brightness > 100) new_brightness = 100;
                        ESP_LOGI(TAG, "Brightness: %d → %d", backlight->brightness(), new_brightness);
                        
                        backlight->SetBrightness(new_brightness);
                        auto display = board.GetDisplay();
                        display->ShowNotification("Brightness: " + std::to_string(new_brightness));
                    }
                    }
                    break;
                case TOUCH_GESTURE_SWIPE_LEFT:
                    {
                    ESP_LOGI(TAG, "👈 Swipe LEFT");
                    music::SourceType source = Application::GetInstance().BuildMusicInfo().source;
                    ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                    if (source == music::SourceType::SD_CARD) {
                        ESP_LOGI(TAG, "Play Previous track");
                        auto& app = Application::GetInstance();
                        auto sd_music = app.GetSdMusic();
                        if (sd_music) {
                            sd_music->Stop();
                            sd_music->Prev();
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }
                        break;
                    }
                    auto& board = Board::GetInstance();
                    auto backlight = board.GetBacklight();
                    int new_brightness = backlight->brightness();
                    
                    // Swipe left - decrease brightness
                    new_brightness -= 5;
                    if (new_brightness <= 0) new_brightness = 0;  // Min 5% to keep visible
                    ESP_LOGI(TAG, "Brightness: %d → %d", backlight->brightness(), new_brightness);
                    
                    backlight->SetBrightness(new_brightness);
                    auto display = board.GetDisplay();
                    display->ShowNotification("Brightness: " + std::to_string(new_brightness));
                    }
                    break;
                case TOUCH_GESTURE_SWIPE_DOWN:
                    ESP_LOGI(TAG, "👇 Swipe DOWN");
                    {
                    auto codec = GetAudioCodec();
                    auto volume = codec->output_volume() - 5;
                    if (volume < 0) {
                        volume = 0;
                    }
                    codec->SetOutputVolume(volume);
                    GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
                    }
                    break;
                case TOUCH_GESTURE_SWIPE_UP:
                    ESP_LOGI(TAG, "👆 Swipe UP");
                    {
                    auto codec = GetAudioCodec();
                    auto volume = codec->output_volume() + 5;
                    if (volume > 100) {
                        volume = 100;
                    }
                    codec->SetOutputVolume(volume);
                    GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
                    }
                    break;
                case TOUCH_GESTURE_TAP:
                    ESP_LOGI(TAG, "🖐️ Tap at (%d, %d)", x, y);
                    break;
                case TOUCH_GESTURE_DOUBLE_TAP:
                    ESP_LOGI(TAG, "👆👆 Double Tap at (%d, %d)", x, y);
                    {
                    auto& app = Application::GetInstance();
                    if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                        ResetWifiConfiguration();
                    }
                    app.ToggleChatState();
                    }
                    break;
                case TOUCH_GESTURE_LONG_PRESS:
                    ESP_LOGW(TAG, "Long Press at (%d, %d)", x, y);
                    {
                    music::SourceType source = Application::GetInstance().BuildMusicInfo().source;
                    ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                    if (source == music::SourceType::NONE) {
                        auto& app = Application::GetInstance();
                        auto sd_music = app.GetSdMusic();
                        if (sd_music) {
                        ESP_LOGI(TAG, "Toggle Play/Pause");
                        sd_music->Play();
                        }
                    } else {
                        GetAudioCodec()->SetOutputVolume(0);
                        GetDisplay()->ShowNotification(Lang::Strings::MUTED);
                    }
                    }
                    break;
                default:
                    break;
            }
        });
        ESP_LOGI(TAG, "Touch screen is ready - try touching now...");
    }
#endif

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CustomBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeTca9554();
        InitializeSpi();
        Initializest77916Display();
#ifdef CONFIG_TOUCH_PANEL_ENABLE
        InitializeTouch();
#endif
        InitializeButtons();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_LEFT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_RIGHT); // I2S_STD_SLOT_LEFT / I2S_STD_SLOT_RIGHT / I2S_STD_SLOT_BOTH

        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

#ifdef CONFIG_SD_CARD_MMC_INTERFACE
    virtual SdCard* GetSdCard() override {
#ifdef CARD_SDMMC_BUS_WIDTH_4BIT
        static SdMMC sdmmc(CARD_SDMMC_CLK_GPIO,
                           CARD_SDMMC_CMD_GPIO,
                           CARD_SDMMC_D0_GPIO,
                           CARD_SDMMC_D1_GPIO,
                           CARD_SDMMC_D2_GPIO,
                           CARD_SDMMC_D3_GPIO);
#else
#ifdef CARD_SDMMC_D3_GPIO
        if (CARD_SDMMC_D3_GPIO != GPIO_NUM_NC) {
            gpio_set_direction(CARD_SDMMC_D3_GPIO, GPIO_MODE_INPUT);
            gpio_pullup_en(CARD_SDMMC_D3_GPIO);
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for the pin to stabilize
        }
#endif
        static SdMMC sdmmc(CARD_SDMMC_CLK_GPIO,
                           CARD_SDMMC_CMD_GPIO,
                           CARD_SDMMC_D0_GPIO);
#endif
        return &sdmmc;
    }
#endif
#ifdef CONFIG_SD_CARD_SPI_INTERFACE
    virtual SdCard* GetSdCard() override {
        static SdSPI sdspi(CARD_SPI_MISO_GPIO,
                           CARD_SPI_MOSI_GPIO,
                           CARD_SPI_SCLK_GPIO,
                           CARD_SPI_CS_GPIO);
        return &sdspi;
    }
#endif
};

DECLARE_BOARD(CustomBoard);
