/*
I (3466825) main: ----- Task Stack Monitor -----
I (3466825) main: TaskMonitor      | Free: 2220 bytes | Prio:  1
I (3466825) main: IDLE             | Free:  468 bytes | Prio:  0
I (3466825) main: tiT              | Free: 2212 bytes | Prio: 18
I (3466835) main: taskLVGL         | Free: 1416 bytes | Prio:  1
I (3466835) main: mqtt_task        | Free: 3744 bytes | Prio:  5    3K Found in: Component config > ESP-MQTT Configurations > CONFIG_MQTT_USE_CUSTOM_CONFIG
I (3466845) main: Tmr Svc          | Free: 1804 bytes | Prio:  1    
I (3466845) main: opus_codec       | Free: 1524 bytes | Prio:  2    1K
I (3466855) main: audio_output     | Free: 1332 bytes | Prio:  4    1K
I (3466865) main: main_event_loop  | Free: 2676 bytes | Prio:  3    2K
I (3466865) main: wifi             | Free: 1368 bytes | Prio: 23
I (3466875) main: esp_timer        | Free: 2692 bytes | Prio: 22
I (3466875) main: sys_evt          | Free:  588 bytes | Prio: 20
I (3466885) main: audio_input      | Free: 2572 bytes | Prio:  8    2K


I (69449) main: --------------------------------
I (74459) main: ----- Task Stack Monitor -----
I (74459) main: TaskMonitor      | Free:  256 bytes | Prio:  1
I (74459) main: taskLVGL         | Free: 1416 bytes | Prio:  1
I (74469) main: IDLE             | Free:  468 bytes | Prio:  0
I (74469) main: mqtt_task        | Free:  552 bytes | Prio:  5
I (74479) main: audio_input      | Free:  520 bytes | Prio:  8
I (74479) main: tiT              | Free: 2220 bytes | Prio: 18
I (74489) main: Tmr Svc          | Free: 1804 bytes | Prio:  1
I (74499) main: audio_output     | Free:  300 bytes | Prio:  4
I (74519) main: esp_timer        | Free: 2692 bytes | Prio: 22
I (74519) main: wifi             | Free: 1360 bytes | Prio: 23
I (74529) main: main_event_loop  | Free: 1648 bytes | Prio:  3
I (74529) main: opus_codec       | Free:  548 bytes | Prio:  2
I (74539) main: sys_evt          | Free:  588 bytes | Prio: 20
I (74539) main: udp_receive      | Free: 3532 bytes | Prio:  1
*/
#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "system_info.h"

#define TAG "main"

void task_monitor(void *pvParameters)
{
    const int max_tasks = 30;
    TaskStatus_t task_list[max_tasks];
    UBaseType_t task_count;
    uint32_t total_runtime;

    while (1)
    {
        task_count = uxTaskGetSystemState(task_list, max_tasks, &total_runtime);

        ESP_LOGI(TAG, "----- Task Stack Monitor -----");

        for (int i = 0; i < task_count; i++)
        {
            UBaseType_t hw = task_list[i].usStackHighWaterMark;  // đơn vị: words (4 bytes)
#if ( configTASKLIST_INCLUDE_COREID == 1 )
            ESP_LOGI(TAG,
                     "%-16s | Free: %4u bytes | Prio: %2u | Core: %u",
                     task_list[i].pcTaskName,
                     hw,
                     task_list[i].uxCurrentPriority,
                     task_list[i].xCoreID
            );
#else
            ESP_LOGI(TAG,
                     "%-16s | Free: %4u bytes | Prio: %2u",
                     task_list[i].pcTaskName,
                     hw,
                     task_list[i].uxCurrentPriority
            );
#endif
        }

        ESP_LOGI(TAG, "--------------------------------");

        vTaskDelay(pdMS_TO_TICKS(5000));  // 5s
    }
}

extern "C" void app_main(void)
{
    // Initialize the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS flash for WiFi configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Launch the application
    auto& app = Application::GetInstance();
    app.Start();

    xTaskCreatePinnedToCore(task_monitor, "TaskMonitor", 2048, NULL, 1, NULL, tskNO_AFFINITY);
}
