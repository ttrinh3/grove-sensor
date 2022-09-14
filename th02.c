#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"

#define SDA_GPIO 18
#define SCL_GPIO 19
#define TH_02_ADDRESS 0x40
float readTemp(void);
float readHum(void);
void app_main(void);
{
i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);


    while(1){
    printf("Temperature = %f\n", readTemp());
    printf("Humidity = %f\n", readHum());


    
    vTaskDelay(1000/portTICK_RATE_MS);
    
    }




}

float readTemp(void){

    uint8_t raw1;
    int error[9];
    uint8_t raw2;

    i2c_cmd_handle_t cmd_handle;

    //Temperature Sequence
    cmd_handle = i2c_cmd_link_create();
    if(i2c_master_start(cmd_handle))
    printf("error\n");
    //slave address followed by 0, ack
    if(i2c_master_write_byte(cmd_handle,(0x40 << 1) | I2C_MASTER_WRITE, true))
    printf("error\n");
    //0x03 for humidity and get acknowledgement
   if( i2c_master_write_byte(cmd_handle,0x03, true))
   printf("error\n");
    //0x01 for register 1 and get ack
   if( i2c_master_write_byte(cmd_handle,0x11, true))
   printf("error\n");
    //P
   if( i2c_master_stop(cmd_handle))
   printf("error\n");
   if(i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 1000/ portTICK_RATE_MS))
   printf("error\n");
   i2c_cmd_link_delete(cmd_handle);

    
    //create link for another sequence
    cmd_handle = i2c_cmd_link_create();
    //READ SEQUENCE
    error[0] = i2c_master_start(cmd_handle); //start signal, which is S
    error[1] = i2c_master_write_byte(cmd_handle,(0x40 << 1) | I2C_MASTER_WRITE, true); //slaveaddr+0 + ack
    //the next byte is the register address pointer + acknowledge
    error[2] = i2c_master_write_byte(cmd_handle,0x01, true); //choose register 1 + ack
    error[3] = i2c_master_start(cmd_handle); //Sr (repeated start)
    //write slave address but follow with 1 +ack
    //0x41 is 0x40 with 1 at the right
    error[4] = i2c_master_write_byte(cmd_handle,(0x40<<1) | I2C_MASTER_READ, true);
    //read from DataH which is register 1 and ack as master
    error[5] = i2c_master_read_byte(cmd_handle,(uint8_t *)&raw1,I2C_MASTER_ACK);
    //read from DataL, and Nack as master
    error[6] = i2c_master_read_byte(cmd_handle,(uint8_t *)&raw2, I2C_MASTER_NACK);
    //stop command
    error[7] = i2c_master_stop(cmd_handle);
    
    //error returns 0x107 (or 263) which means OPERATION TIMED OUT or bruh wtfff
    error[8] = i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 1000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);




   

    uint16_t raw3 = (raw1<<8) | (raw2);

    
    float temp = ((raw3>>2)/32.0)-50.0;
    
    return temp;

}

float readHum(void){

    uint8_t raw1;
    int error[9];
    uint8_t raw2;

    i2c_cmd_handle_t cmd_handle;


    cmd_handle = i2c_cmd_link_create();
    if(i2c_master_start(cmd_handle))
    printf("error\n");
    //slave address followed by 0, ack
    if(i2c_master_write_byte(cmd_handle,(0x40 << 1) | I2C_MASTER_WRITE, true))
    printf("error\n");
    //0x03 for humidity and get acknowledgement
   if( i2c_master_write_byte(cmd_handle,0x03, true))
   printf("error\n");
    //0x01 for register 1 and get ack
   if( i2c_master_write_byte(cmd_handle,0x01, true))
   printf("error\n");
    //P
   if( i2c_master_stop(cmd_handle))
   printf("error\n");
   if(i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 1000/ portTICK_RATE_MS))
   printf("error\n");
   i2c_cmd_link_delete(cmd_handle);

    cmd_handle = i2c_cmd_link_create();
    //READ SEQUENCE
    error[0] = i2c_master_start(cmd_handle); //start signal, which is S
    error[1] = i2c_master_write_byte(cmd_handle,(0x40 << 1) | I2C_MASTER_WRITE, true); //slaveaddr+0 + ack
    //the next byte is the register address pointer + acknowledge
    error[2] = i2c_master_write_byte(cmd_handle,0x01, true); //choose register 1 + ack
    error[3] = i2c_master_start(cmd_handle); //Sr (repeated start)
    //write slave address but follow with 1 +ack
    //0x41 is 0x40 with 1 at the right
    error[4] = i2c_master_write_byte(cmd_handle,(0x40<<1) | I2C_MASTER_READ, true);
    //read from DataH which is register 1 and ack as master
    error[5] = i2c_master_read_byte(cmd_handle,(uint8_t *)&raw1,I2C_MASTER_ACK);
    //read from DataL, and Nack as master
    error[6] = i2c_master_read_byte(cmd_handle,(uint8_t *)&raw2, I2C_MASTER_NACK);
    //stop command
    error[7] = i2c_master_stop(cmd_handle);
    
    //error returns 0x107 (or 263) which means OPERATION TIMED OUT or bruh wtfff
    error[8] = i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 1000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);




   

    uint16_t raw3 = (raw1<<8) | (raw2);

    
    float temp = ((raw3>>4)/16.0)-24.0;
    
    return temp;

}
