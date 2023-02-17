#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c-master";

#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 5
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define SLAVE_ADDRESS 0x0A
#define I2C_SLAVE_TX_BUF_LEN 255
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/

int i2c_master_port = 0;
static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
  if (err != ESP_OK) {
    return err;
  }
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_master_send(uint8_t message[], int len) {
  ESP_LOGI(TAG, "Sending Message = %s", message);
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /*
  为了启动 I2C
  总线上的通信，主机必须首先发送一个启动条件。开始条件是时钟和数据线上的特定信号模式，
  它向总线上的所有设备指示新通信即将开始。
  */
  i2c_master_start(cmd);
  // 发送开始条件后，主机发送它要与之通信的从机的地址,发送的第一个字节称为控制字节。控制字节的前七位构成从机地址。
  // 如果从机收到自己的地址，则以应答信号响应。确认信号是数据线上的一个脉冲，被从设备拉低一个时钟周期。该信号向主机表明从机已收到其地址并准备好进行通信。
  i2c_master_write_byte(cmd, SLAVE_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  // 从机侦听数据线上的数据并以确认信号响应以表明它已接收到数据。这个过程可以重复多次来发送和接收多个字节的数据
  i2c_master_write(cmd, message, len, ACK_CHECK_EN);
  //主机完成与从机的通信后，发送停止条件以结束传输。停止条件是时钟和数据线上的特定信号模式，它向总线上的所有设备指示通信已完成。
  i2c_master_stop(cmd);
  // 在主模式下发送 I2C
  // 总线上所有排队的命令。该任务将被阻塞，直到所有的命令都被发送出去。
  ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
  // 释放 I2C 命令列表。
  i2c_cmd_link_delete(cmd);
  return ret;
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C Initialized Successfully");
  uint8_t read_buffer[I2C_SLAVE_TX_BUF_LEN] = {0};
  uint8_t write_buffer[] = "LED_ON";
  esp_err_t ret;
  while (1) {
    // i2c_master_write_read_device 函数是
    // i2c_master_start()、i2c_master_write()、i2c_master_read()
    // 等函数的封装
    ret = i2c_master_write_read_device(i2c_master_port, (uint8_t)SLAVE_ADDRESS,
                                       write_buffer, sizeof(write_buffer),
                                       read_buffer, I2C_SLAVE_TX_BUF_LEN,
                                       1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Data Recived = %s", read_buffer);
      memset(read_buffer, 0, I2C_SLAVE_TX_BUF_LEN);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
