
#define ESP32
#define ETL_CALLBACK_TIMER_USE_ATOMIC_LOCK

// Stdlibs
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"

// Arduino Core
#include "arduino-esp32/cores/esp32/Arduino.h"



/**********************
 *   APPLICATION MAIN
 **********************/
extern "C"
{
    void app_main();
}

void app_main() 
{
    initArduino();
    Serial.begin(115200);

    // Setup input / outputs
    pinMode(TOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MIDDLE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_BUTTON_PIN, INPUT_PULLUP);
    pinMode(EXT_POWER_OK, INPUT);
    pinMode(EN_ACC_PWR_PIN, OUTPUT);
    adcAttachPin(BATTERY_ADC);

    VibrationService::Begin(VIB_MOTOR_PIN, 5000);

    // Enable a wake-up button from deep sleep
    esp_sleep_enable_ext0_wakeup(BOTTOM_BUTTON_PIN, LOW);

    // Check external power
    Serial.printf("External power: %s\n", digitalRead(EXT_POWER_OK) == HIGH ? "Good" : "Bad");

    // Enable the accessory power (Screen, RFID, Vibration Motor, Light Sensor)
    digitalWrite(EN_ACC_PWR_PIN, HIGH);

    attachInterrupt(TOP_BUTTON_PIN, TopButtonInterrupt, CHANGE);
    attachInterrupt(BOTTOM_BUTTON_PIN, BottomButtonInterrupt, CHANGE);
    attachInterrupt(MIDDLE_BUTTON_PIN, MiddleButtonInterrupt, CHANGE);

    // Start any core services here
    MainThread.Begin();
    settingsService.Begin();

    // Create the button event handling task
    xTaskCreatePinnedToCore(buttonEventTask, "buttons", 2048, NULL, 2, NULL, 1);

    // Create the GUI task for LVGL
    // TODO: Check if changing the gui/application task priority has any effect on ui/app performance
    xTaskCreate(guiTask, "gui", 4096*2, NULL, 1, NULL);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Create the application task
    // This task handles the view and model lifetimes and is where most app functionality occurs
    // TODO: Check memory performance and add in the high-water-mark functionality for task stacks
    xTaskCreate(applicationTask, "application", 2048*8, NULL, 1, NULL);
}

static void guiTask(void *pvParameter) 
{

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateRecursiveMutex();
    // Take the semaphore initially whilst we're setting up
    if (pdTRUE == xSemaphoreTakeRecursive(xGuiSemaphore, portMAX_DELAY)) 
    {
        lv_init();

        /* Initialize SPI or I2C bus used by the drivers */
        lvgl_driver_init();

        lv_color_t* buf1 = (lv_color_t*)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
        assert(buf1 != NULL);

        lv_color_t* buf2 = (lv_color_t*)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
        assert(buf2 != NULL);

        static lv_disp_buf_t disp_buf;

        uint32_t size_in_px = DISP_BUF_SIZE;

        // Initialize the working buffer depending on the selected display.
        lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

        lv_disp_drv_t disp_drv;
        lv_disp_drv_init(&disp_drv);
        disp_drv.flush_cb = disp_driver_flush;
        disp_drv.buffer = &disp_buf;
        lv_disp_drv_register(&disp_drv);

        /* Create and start a periodic timer interrupt to call lv_tick_inc */
        const esp_timer_create_args_t periodic_timer_args = 
        {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
        };
        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

        // Release the semaphore now that we're done setting up
        xSemaphoreGiveRecursive(xGuiSemaphore);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 10ms

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTakeRecursive(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGiveRecursive(xGuiSemaphore);
       }
    }

    vTaskDelete(NULL);
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}