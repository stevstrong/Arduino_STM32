#include <MapleFreeRTOS10.h>


uint32_t tick;
//-----------------------------------------------------------------------------
static void vLEDFlashTask(void *pvParameters)
{
    (void) pvParameters; // unused
    for (;;) {
        Serial.print("tick "); Serial.println(++tick);
        vTaskDelay(750 / TICK_DIVISOR);
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(250 / TICK_DIVISOR);
        digitalWrite(LED_BUILTIN, LOW);
    }
}
//-----------------------------------------------------------------------------
void setup()
{
    // initialize the digital pin as an output:
    // while(!Serial); delay(20);
    Serial.begin();
    delay(100);
    Serial.println ("Running...");
    // initialize the digital pin as an output:
    pinMode(LED_BUILTIN, OUTPUT);

    xTaskCreate(vLEDFlashTask,
                "Blinky",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
    // should not get here
}

//-----------------------------------------------------------------------------
void loop() {
    // Do not write any code here, it would not execute.
}

