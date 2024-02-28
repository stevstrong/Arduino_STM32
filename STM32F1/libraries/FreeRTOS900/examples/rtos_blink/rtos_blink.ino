//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include <MapleFreeRTOS900.h>

static void vLEDFlashTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(750);
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(250);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void setup() {
    // initialize the digital pin as an output:
    pinMode(LED_BUILTIN, OUTPUT);

    xTaskCreate(vLEDFlashTask,
                "Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


