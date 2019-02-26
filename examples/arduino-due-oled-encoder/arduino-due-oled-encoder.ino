#include <Wire.h>
#include <U8g2lib.h>
#include <Encoder.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <DueFlashStorage.h>

#define OLED_SCREEN_WIDTH 128 // OLED display width, in pixels
#define OLED_SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_I2C_ADDRESS 0x3C
#define OLED_RESET       -1 // Reset pin # (or -1 if sharing Arduino reset pin)

enum display_event_type {
  display_counter,
  display_button,
  display_temperature
};

struct display_event {
  display_event_type type;
};

struct led_event {
  int button_value;
};

struct app_data {
  int led_delay;
  int btn_state;
  float temperature;
};

struct nvm_config {
  uint8_t first_time;
  uint8_t sync;
  int encoder_value;
};

app_data data = { 0, 0, 0.0 };
nvm_config config = { 0xFF, 0xFF, 0};
QueueHandle_t display_queue;
QueueHandle_t led_queue;

void display_task(void *pvParameters)
{
  Serial.begin(9600);
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  display_event event;

  u8g2.begin();
  u8g2.clear();
  u8g2.setFont(u8g2_font_profont12_mf);
  u8g2.drawStr(10, 25, "Rotary encoder");
  u8g2.sendBuffer();
  vTaskDelay(2000);
  u8g2.clear();
 
  for( ;; ) {
    if (xQueueReceive(display_queue, &event, 10)) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_profont12_mf);

      u8g2.drawStr(0, 15, data.btn_state == LOW ? "Button pressed" : "Button released");
      if (event.type == display_button) {
        Serial.println(data.btn_state == LOW ? "Button pressed" : "Button released");
      } 
      
      String led_delay = String(fabs(data.led_delay) / 1000.0) + " s";
      u8g2.drawStr(10, 35, led_delay.c_str());
      if (event.type == display_counter) {
        Serial.println(data.led_delay);
      } 
      
      String temperature = String(data.temperature) + " degC";
      u8g2.drawStr(10, 55, temperature.c_str());
      if (event.type == display_temperature) {
        Serial.println(data.temperature);
      }

      u8g2.sendBuffer();
    }
  }
}

void button_task(void *pvParameters)
{
  display_event event_1;
  led_event event_2;

  pinMode(10, INPUT_PULLUP);
  data.btn_state = digitalRead(10);
  for( ;; ) {
    int c_btn_state = digitalRead(10);
    if (data.btn_state != c_btn_state) {
      data.btn_state = c_btn_state;

      // display the button state
      event_1.type = display_button;
      xQueueSend(display_queue, &event_1, 15);

      // notify the led task that the button has been pressed
      event_2.button_value = c_btn_state;
      xQueueSend(led_queue, &event_2, 50);
    }
    vTaskDelay(1);
  }
}

void encoder_task(void *pvParameters)
{
  Encoder myEnc(2, 3);
  display_event event;
  int p_counter, c_counter;

  // wait for nvm_config
  while (config.sync) {
    vTaskDelay(1000);
  }
  myEnc.write(config.encoder_value);
  p_counter = c_counter = myEnc.read();
  data.led_delay = c_counter / 4 * 100;
  
  // first event to display something
  event.type = display_counter;
  xQueueSend(display_queue, &event, 50);

  for( ;; ) {
    c_counter = myEnc.read();
    if (c_counter != p_counter) {
      p_counter = c_counter;
      if (c_counter % 4 == 0) {
        // limit to 0
        if (c_counter < 0) {
          myEnc.write(0);
          c_counter = 0;
          p_counter = 0;
        }
        data.led_delay = c_counter / 4 * 100;
        
        // save to nvm config
        config.encoder_value = myEnc.read();
        config.sync = 1;

        // display delay time on OLED
        event.type = display_counter;
        xQueueSend(display_queue, &event, 50);
      }
    }
    vTaskDelay(1);
  }
}

void led_task(void *pvParameters)
{
  display_event event_1;
  led_event event_2;

  pinMode(LED_BUILTIN, OUTPUT);
  for( ;; ) {
    if (xQueueReceive(led_queue, &event_2, 10)) {
      if (event_2.button_value == 1) {
        if (data.led_delay != 0) {
          digitalWrite(LED_BUILTIN, HIGH);
          vTaskDelay(data.led_delay);
          digitalWrite(LED_BUILTIN, LOW);
          xQueueReset(led_queue);
        }

        // display the delay
        event_1.type = display_counter;
        xQueueSend(display_queue, &event_1, 50);
      }
    }
  }
}

void temp_task(void *pvParameters)
{
  display_event event;
  float prev_temperature = 0;

  pinMode(A0, INPUT);
  analogReadResolution(12);
  for( ;; ) {
    int analog_in = analogRead(A0);
    float r = 10000.0 * analog_in / (4096 - analog_in);
    float steinhart;                             // https://learn.adafruit.com/thermistor/using-a-thermistor
    steinhart = r / 10000.0;                     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= 3950.0;                         // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15);            // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    data.temperature = steinhart;

    if (fabs(prev_temperature - data.temperature) > 0.1) {
      prev_temperature = data.temperature;
      event.type = display_temperature;
      xQueueSend(display_queue, &event, 50);
    }
    vTaskDelay(500);
  }
}

void nvm_task(void *pvParameters)
{
  DueFlashStorage storage;
  uint8_t *storageData;

  storageData = storage.readAddress(0);
  memcpy(&config, storageData, sizeof(nvm_config));

  // first time init
  if (config.first_time) {
    config.first_time = 0;
    config.sync = 1;
    config.encoder_value = 40; // 1 second
  }

  for( ;; ) {
    if (config.sync == 1) {
      config.sync = 0;
      storageData = (uint8_t*)(&config);
      storage.write(0, storageData, sizeof(nvm_config));
    }
    vTaskDelay(1000);
  }
}

void setup() {
  // create the queues 
  display_queue = xQueueCreate(10, sizeof(display_event));
  led_queue     = xQueueCreate(10, sizeof(led_event));

  // create the tasks
  //          function       name                             stack  parameter         priority   task
  xTaskCreate(display_task, (const portCHAR *)"display_task", 512,   NULL,             1,         NULL);
  xTaskCreate(button_task,  (const portCHAR *)"button_task",  128,   NULL,             2,         NULL);
  xTaskCreate(encoder_task, (const portCHAR *)"encoder_task", 128,   NULL,             2,         NULL);
  xTaskCreate(led_task,     (const portCHAR *)"led_task",     128,   NULL,             1,         NULL);
  xTaskCreate(temp_task,    (const portCHAR *)"temp_task",    128,   NULL,             1,         NULL);
  xTaskCreate(nvm_task,     (const portCHAR *)"nvm_task",     128,   NULL,             1,         NULL);

  // start scheduler
  vTaskStartScheduler();

  // out of memory
  Serial.println("Insufficient RAM");
  while(1);
}

void loop() {
}
