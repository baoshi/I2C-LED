#ifndef ADAFRUIT_I2C_LED_EMU
#define ADAFRUIT_I2C_LED_EMU


void emu_on_i2c_start_write(void);
void emu_on_i2c_stop(void);
uint8_t emu_on_i2c_data_received(uint8_t data);

void emu_run(void);


#endif