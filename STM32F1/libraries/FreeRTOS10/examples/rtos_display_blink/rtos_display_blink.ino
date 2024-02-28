#include <MapleFreeRTOS10.h>
#include <Adafruit_ILI9341_STM.h>

#define __CS  PA4
#define __RST PA2
#define __DC  PA3

Adafruit_ILI9341_STM tft(__CS, __DC, __RST);

SemaphoreHandle_t xDisplayFree;


const float sin_d[] = {
  0, 0.17, 0.34, 0.5, 0.64, 0.77, 0.87, 0.94, 0.98, 1, 0.98, 0.94,
  0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34, -0.5, -0.64,
  -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87, -0.77,
  -0.64, -0.5, -0.34, -0.17
};
const float cos_d[] = {
  1, 0.98, 0.94, 0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34,
  -0.5, -0.64, -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87,
  -0.77, -0.64, -0.5, -0.34, -0.17, 0, 0.17, 0.34, 0.5, 0.64, 0.77,
  0.87, 0.94, 0.98
};
const float d = 5;
float cube1_px[] = {
  -d,  d,  d, -d, -d,  d,  d, -d
};
float cube1_py[] = {
  -d, -d,  d,  d, -d, -d,  d,  d
};
float cube1_pz[] = {
  -d, -d, -d, -d,  d,  d,  d,  d
};

float cube1_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube1_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube1_r[] = {
  0, 0, 0
};
const float d2 = 10;
float cube2_px[] = {
  -d2,  d2,  d2, -d2, -d2,  d2,  d2, -d2
};
float cube2_py[] = {
  -d2, -d2,  d2,  d2, -d2, -d2,  d2,  d2
};
float cube2_pz[] = {
  -d2, -d2, -d2, -d2,  d2,  d2,  d2,  d2
};

float cube2_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube2_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube2_r[] = {
  0, 0, 0
};

uint16 cube1_x, cube1_y, cube2_x, cube2_y, cube1_color, cube2_color;

static void vLEDFlashTask(void *pvParameters) {
  (void) pvParameters; // ununsed
  for (;;) {
    digitalWrite(LED_BUILTIN, LOW); // LED ON
    vTaskDelay(250 / TICK_DIVISOR);
    digitalWrite(LED_BUILTIN, HIGH); // LED OFF
    vTaskDelay((750) / TICK_DIVISOR);
  }
}

static void vCube1LoopTask(void *pvParameters) {
  (void) pvParameters; // ununsed
  while (1) {
    if ( xSemaphoreTake( xDisplayFree, ( TickType_t ) 10 ) == pdTRUE )
    {
      cube(cube1_px, cube1_py, cube1_pz, cube1_p2x, cube1_p2y, cube1_r, &cube1_x, &cube1_y, &cube1_color);
      xSemaphoreGive( xDisplayFree );
        vTaskDelay(15 / TICK_DIVISOR);
    }
  }
}

static void vCube2LoopTask(void *pvParameters) {
  (void) pvParameters; // ununsed
  while (1) {
    if ( xSemaphoreTake( xDisplayFree, ( TickType_t ) 10 ) == pdTRUE )
    {
      cube(cube2_px, cube2_py, cube2_pz, cube2_p2x, cube2_p2y, cube2_r, &cube2_x, &cube2_y, &cube2_color);
      xSemaphoreGive( xDisplayFree );
        vTaskDelay(40 / TICK_DIVISOR);
    }
  }
}


void cube(float *px, float *py, float *pz, float *p2x, float *p2y, int *r, uint16 *x, uint16 *y, uint16 *color)
{
  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], ILI9341_WHITE);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], ILI9341_WHITE);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], ILI9341_WHITE);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], ILI9341_WHITE);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], ILI9341_WHITE);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], ILI9341_WHITE);

  r[0] = r[0] + 1;
  r[1] = r[1] + 1;
  if (r[0] == 36) r[0] = 0;
  if (r[1] == 36) r[1] = 0;
  if (r[2] == 36) r[2] = 0;
  for (int i = 0; i < 8; i++)
  {
    float px2 = px[i];
    float py2 = cos_d[r[0]] * py[i] - sin_d[r[0]] * pz[i];
    float pz2 = sin_d[r[0]] * py[i] + cos_d[r[0]] * pz[i];

    float px3 = cos_d[r[1]] * px2 + sin_d[r[1]] * pz2;
    float py3 = py2;
    float pz3 = -sin_d[r[1]] * px2 + cos_d[r[1]] * pz2;

    float ax = cos_d[r[2]] * px3 - sin_d[r[2]] * py3;
    float ay = sin_d[r[2]] * px3 + cos_d[r[2]] * py3;
    float az = pz3 - 190;

    p2x[i] = *x + ax * 500 / az;
    p2y[i] = *y + ay * 500 / az;
  }

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], *color);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], *color);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], *color);
  }
  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], *color);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], *color);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], *color);
}

static void vSqrtTask(void *pvParameters) {
  (void) pvParameters; // ununsed
  while (1){
  Serial.println ("Starting Sqrt calculations...");
  // Library Sqrt
  uint32_t t0 = millis();
  for (uint32_t n = 247583650 ; n > 247400000 ; n--) {
    sqrt (n);
  }
  uint32_t t1 = millis() - t0;
  Serial.print ("Sqrt calculations took (ms): ");
  Serial.println (t1);
  delay (5000);
  }
}

void setup() {
  // initialize the digital pin as an output:
  Serial.begin(9600);
  delay (5000);
  Serial.println ("Running...");
  pinMode(LED_BUILTIN, OUTPUT);
  tft.begin();
  tft.fillScreen(ILI9341_WHITE);
  cube1_x = ((tft.width()) / 4);
  cube1_y = ((tft.height()) / 4);
  cube2_x = ((tft.width()) / 2);
  cube2_y = ((tft.height()) / 2);
  cube1_color = ILI9341_BLACK;
  cube2_color = ILI9341_RED;
  vSemaphoreCreateBinary(xDisplayFree);
  xTaskCreate(vLEDFlashTask,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(vCube1LoopTask,
              "Cube1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(vCube2LoopTask,
              "Cube2",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY+1,
              NULL);
  xTaskCreate(vSqrtTask,
              "Sqrt",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  vTaskStartScheduler();
}

void loop() {
  // Do not write any code here, it would not execute.
}


