#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../components/CompUI/interfaces/ui_task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "ui_task.h"


/////////////////////////////////////////////////////////////////////////////////////////////////
static void clockTask_workerFunction(void *p)
{
    vTaskDelay( pdMS_TO_TICKS( 3UL * 1000UL ) );

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // wait a second...
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1UL * 1000UL ) );
        UItask_sendMessage('!');
    }
}


void clockTaskStart( int prio )
{
    xTaskCreate(clockTask_workerFunction, "clocktask", 8192, NULL, prio, NULL);
}
