#include "stm8s.h"
#include "led.h"
#include "adafruit_i2c_led_emu.h"

/******************************************************************************
 * As the software emulated I2C slave is interrupt driven, any callbacks must *
 * hog as little CPU as possible so not to mess up I2C timing.                *
 * A command buffer is hence implemented. I2C data callback will just         *
 * validate received data and fill the command buffer.                        *
 * Real excution of the command is in "emu_run" function call                 *
 ******************************************************************************/
 

/******************************************************************************
 * Cyclic FIFO command buffer                                                 *
 ******************************************************************************/   

typedef enum
{
  CMD_UNKNOWN = (uint8_t)0,
  CMD_DISPLAY_ADDRESS_POINTER,
  CMD_SYSTEM_SETUP,
  CMD_DISPLAY_SETUP,
  CMD_DIMMING_SET,
  DATA_DISPLAY_DATA
} COMMANDTYPE;


typedef struct
{
  uint8_t value;
  COMMANDTYPE type;
} COMMANDENTRY;


#define COMMAND_BUFFER_SIZE 32
COMMANDENTRY _command_buffer[COMMAND_BUFFER_SIZE];   // cyclic command buffer
uint8_t _command_buffer_head = 0;
uint8_t _command_buffer_tail = 0;
uint8_t _command_buffer_used = 0;


// Store command into the buffer. Return 0 if buffer full
static uint8_t _store_command(uint8_t value, COMMANDTYPE type)
{
  if (_command_buffer_used < COMMAND_BUFFER_SIZE)
  {
    _command_buffer[_command_buffer_tail].value = value;
    _command_buffer[_command_buffer_tail].type = type;
    _command_buffer_tail = (_command_buffer_tail + 1) & COMMAND_BUFFER_SIZE;
    ++_command_buffer_used;
    return 1;
  }
  return 0;
}


// Get command out of the buffer.  Return 0 if buffer empty
static uint8_t _retrieve_command(uint8_t *value, COMMANDTYPE *type)
{
  if (_command_buffer_used == 0)
    return 0;
  *value = _command_buffer[_command_buffer_head].value;
  *type = _command_buffer[_command_buffer_head].type;
  _command_buffer_head = (_command_buffer_head + 1) & COMMAND_BUFFER_SIZE;
  --_command_buffer_used;
  return 1;
}


// Peek command from the buffer without remove it. Return 0 if buffer empty
static uint8_t _peek_command(uint8_t *value, COMMANDTYPE *type)
{
  if (_command_buffer_used == 0)
    return 0;
  *value = _command_buffer[_command_buffer_head].value;
  *type = _command_buffer[_command_buffer_head].type;
  return 1;
}


/******************************************************************************
 * Pre-process received data                                                  *
 ******************************************************************************/  

#define HT16K33_SYSTEM_SETUP                  0x20
#define HT16K33_DISPLAY_SETUP                 0x80
#define HT16K33_DIMMING_SET                   0xe0

// reciving state
typedef enum
{
  WAIT_VALID_COMMAND = (uint8_t)0,
  WAIT_ANY_DATA,
  COMMAND_BUFFER_FULL
} PREPROCESSSTATE;
static PREPROCESSSTATE _preprocess_state;


void emu_on_i2c_start_write(void)
{
  _preprocess_state = WAIT_VALID_COMMAND;
}


void emu_on_i2c_stop(void)
{
  _preprocess_state = WAIT_VALID_COMMAND;
}


uint8_t emu_on_i2c_data_received(uint8_t data)
{
  uint8_t validation;
  uint8_t result = 1;
  
  // Verify data
  switch (_preprocess_state)
  {
  case WAIT_VALID_COMMAND:
    validation = data & 0xf0;
    if (validation == 0)
    {
      if (_store_command(data, CMD_DISPLAY_ADDRESS_POINTER))
      {
        _preprocess_state = WAIT_ANY_DATA;
      }
      else
      {
        // Buffer full. Skip data until next I2C start or stop
        _preprocess_state = COMMAND_BUFFER_FULL;
        result = 0;
      }
    }
    else if (validation == HT16K33_SYSTEM_SETUP)
    {
      if (!_store_command(data, CMD_SYSTEM_SETUP))
      {
        _preprocess_state = COMMAND_BUFFER_FULL;
        result = 0;
      }
    }
    else if (validation == HT16K33_DISPLAY_SETUP)
    {
      if (!_store_command(data, CMD_DISPLAY_SETUP))
      {
        _preprocess_state = COMMAND_BUFFER_FULL;
        result = 0;
      }
    }
    else if (validation == HT16K33_DIMMING_SET)
    {
      if (!_store_command(data, CMD_DIMMING_SET))
      {
        _preprocess_state = COMMAND_BUFFER_FULL;
        result = 0;
      }
    }
    else
    {
      // Not valild command
      result = 0;
    }
    break;
  case WAIT_ANY_DATA:
    if (!_store_command(data, DATA_DISPLAY_DATA))
    {
        _preprocess_state = COMMAND_BUFFER_FULL;
    }
    break;
  case COMMAND_BUFFER_FULL:
    result = 0;
    break;
  default:
    result = 0;
  }
  return result;
}


static const uint8_t _adafruit_number_table[] = 
{ 
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
	0x77, /* a */
	0x7C, /* b */
	0x39, /* C */
	0x5E, /* d */
	0x79, /* E */
	0x71  /* F */
}; 
static uint8_t _translate_adafruit_display_data(uint8_t data)
{
  uint8_t i, x;
  x = data & 0x7f; // bit 7 is DP
  for (i = 0; i < 16; ++i)
  {
    if (x == _adafruit_number_table[i])
    {
      return (i | (data & 0x80));
    }
  }
  return 0x7f;
}


static uint8_t _display_ram[16] = {0}; // HT16K33 has 16 byte display ram
static uint8_t _display_address = 0;

static void _handle_display_address_pointer(uint8_t cmd)
{
  // Command syntax: is (0 0 0 0 A3 A2 A1 A0)
  _display_address = cmd & 0x0f;
}


static void _handle_display_data(uint8_t data)
{
  _display_ram[_display_address] = data;
  switch (_display_address)
  {
  case 8:
    led_set_digit(4, _translate_adafruit_display_data(_display_ram[8]));
    break;
  case 6:
    led_set_digit(3, _translate_adafruit_display_data(_display_ram[6]));
    break;
  case 2:
    led_set_digit(2, _translate_adafruit_display_data(_display_ram[2]));
    break;
  case 0:
    led_set_digit(1, _translate_adafruit_display_data(_display_ram[0]));
    break;
  }
  _display_address = (_display_address + 1) & 0x0f;
}


static void _handle_system_setup(uint8_t cmd)
{
  // Command syntax: 0 0 1 0 X X X S
  // S=1: Oscillator on; S=0: Oscillator off;
  if (cmd & 0x01)
    led_on();
  else
    led_off();
}


static void _handle_display_setup(uint8_t cmd)
{
  // Command syntax: 1 0 0 0 X B1 B0 D
  // D=1: Display on; D=0: Display 0ff;
  // {B1,B0} = {0,0}: Blink off;
  // {B1,B0} = {0,1}: Blink 2Hz
  // {B1,B0} = {1,0}: Blink 1Hz;
  // {B1,B0} = {1,1}: Blink 0.5Hz;
  // D=0 is ignored. Use system setup to turn led off
  switch (cmd & 0x06)
  {
  case 0x00:
    led_set_blink(LED_BLINK_OFF);
    break;
  case 0x02:
    led_set_blink(LED_BLINK_2HZ);
    break;
  case 0x04:
    led_set_blink(LED_BLINK_1HZ);
    break;
  case 0x06:
    led_set_blink(LED_BLINK_HALFHZ);
    break;
  }
}


static void _handle_dimming_set(uint8_t cmd)
{
  // Command synatx: 1 1 1 0 P3 P2 P1 P0
  // {P3,P2,P1,P0} is duty
  led_set_duty(cmd & 0x0f);
}


void emu_run(void)
{
  uint8_t value;
  COMMANDTYPE type;
  while (_retrieve_command(&value, &type))
  {
    switch (type)
    {
    case CMD_DISPLAY_ADDRESS_POINTER:
      _handle_display_address_pointer(value);
      break;
    case DATA_DISPLAY_DATA:
      _handle_display_data(value);
      break;
    case CMD_SYSTEM_SETUP:
      _handle_system_setup(value);
      break;
    case CMD_DISPLAY_SETUP:
      _handle_display_setup(value);
      break;
    case CMD_DIMMING_SET:
      _handle_dimming_set(value);
      break;
    default:
      // Unknown command, skip
      break;
    }
  }
}


