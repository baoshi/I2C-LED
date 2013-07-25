#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

typedef void (*i2c_start_write_callback)(void);
typedef void (*i2c_start_read_callback)(void);
typedef void (*i2c_stop_callback)(void);
typedef uint8_t (*i2c_data_received_callback)(uint8_t data);
typedef uint8_t (*i2c_data_needed_callback)(void);


void i2c_listen
(
   uint8_t address,
   i2c_start_write_callback on_start_write,
   i2c_start_read_callback on_start_read,
   i2c_stop_callback on_stop,
   i2c_data_received_callback on_data_received,
   i2c_data_needed_callback on_data_needed
);



#endif // I2C_SLAVE_H
