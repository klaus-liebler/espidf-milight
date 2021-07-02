#include <stdio.h>
#include <string.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_system.h>
#include <esp_log.h>
#include "milight.h"

#define TAG "MAIN"
/*
constexpr gpio_num_t CE_PIN = GPIO_NUM_22;
constexpr gpio_num_t CSN_PIN = GPIO_NUM_21;
constexpr gpio_num_t SCLK_PIN = GPIO_NUM_18; // --> 5
constexpr gpio_num_t MOSI_PIN = GPIO_NUM_23; // --> 6
constexpr gpio_num_t MISO_PIN = GPIO_NUM_19; // --> 7
*/

constexpr gpio_num_t CE_PIN = GPIO_NUM_2;//-->3
constexpr gpio_num_t CSN_PIN = GPIO_NUM_15;//-->4
constexpr gpio_num_t SCLK_PIN = GPIO_NUM_14; // --> 5
constexpr gpio_num_t MOSI_PIN = GPIO_NUM_27; // --> 6
constexpr gpio_num_t MISO_PIN = GPIO_NUM_39; // --> 7

constexpr uint8_t CHANNEL = 10;
constexpr uint8_t PAYLOAD_SIZE = 14;
const uint8_t address[6]{0x90, 0x4e, 0x6c, 0x55, 0x55}; //for 9-Byte payload
void receiver(void *pvParameters)
{
 

  Nrf24Receiver recv{};
  recv.setup(HSPI_HOST, 2, CE_PIN, CSN_PIN, MISO_PIN, MOSI_PIN, SCLK_PIN);
  recv.config(CHANNEL, PAYLOAD_SIZE, address, 5, 0,Rf24Datarate::RF24_1MBPS, Rf24PowerAmp::RF24_PA_HIGH);
  recv.printDetails();
  MilightDecoder dec{&recv};
  
  while (true)
  {
    uint8_t cmd{0};
    uint8_t arg{0};
    
    if(dec.TryReceiveNewPacket(&cmd, &arg)){
      ESP_LOGI(TAG, "CMD %0d ARG/SEQ %3d", cmd, arg);
    }
    vTaskDelay(50/portTICK_RATE_MS);
  }
}

extern "C" void app_main()
{
  xTaskCreate(receiver, "RECV", 1024 * 2, NULL, 2, NULL);
  while (true)
  {
    vTaskDelay(1);
  }
}