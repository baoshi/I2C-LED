#include "stm8s.h"
#include "soft_i2c_slave.h"

#define I2C_PORT GPIOD
#define I2C_SCL_PIN GPIO_PIN_5
#define I2C_SDA_PIN GPIO_PIN_6
#define I2C_EXTI_PORT EXTI_PORT_GPIOD
#define I2C_EXTI_SENSITIVITY_MASK EXTI_CR1_PDIS


/*
  STM8 does not allow per-pin configuration of external interrupt.
  So we setup a flag and track the interrupt state we desire.
*/

// Quick interrupt mode consts
typedef enum {
   INTR_SDA_ERF_SCL_EF = (uint8_t)0,
   INTR_SDA_EF_SCL_DR,
   INTR_SDA_DR_SCL_ER,
   INTR_SDA_DR_SCL_EF
} INTERRUPTMODE;

static INTERRUPTMODE _interrupt_mode;


/* SCL states */
typedef enum 
{
  SCL_W1LH = (uint8_t)0,
  SCL_W1HL,
  SCL_W2to6LH,
  SCL_W7LH,
  SCL_W8LH,
  SCL_W8HL,
  SCL_W9HL
} SCLSTATE;

static const SCLSTATE _SCL_WRT[] = 
{
  SCL_W1LH,
  SCL_W1HL,
  SCL_W2to6LH,
  SCL_W2to6LH,
  SCL_W2to6LH,
  SCL_W2to6LH,
  SCL_W2to6LH,
  SCL_W7LH,
  SCL_W8LH,
  SCL_W8HL,
  SCL_W9HL
};
static const SCLSTATE* _scl_state;


// branching flag during 8th bit falling clock
static enum
{
  WRITE_IN = (uint8_t)0,  // Receiving data
  RW_BIT                  // Receiving address
} _br_clk8lh; // Bit 8 is read/write or data


// branching flag during 8th bit rising clock
static enum
{
    N_ACK = (uint8_t)0,
    W_ACK,
    W_CALLBACK,
    R_ACK
} _br_clk8hl; // Whether to send ACK


// I2C Address
static uint8_t _my_address;

              
// Data received
static uint8_t _i2c_data;


// callbacks
static i2c_start_write_callback _on_start_write;
static i2c_start_read_callback _on_start_read;
static i2c_stop_callback _on_stop;
static i2c_data_received_callback _on_data_received;
static i2c_data_needed_callback _on_data_needed;


static void _init_port(void)
{
  // Disable interrupt
  I2C_PORT->CR2 &= (uint8_t)(~(I2C_SCL_PIN | I2C_SDA_PIN));
  // Init state
  // SCL Input, Interrupt on rising edge, Disabled
  // SDA Input, Intrerrupt on falling edge, Enabled
  _interrupt_mode = INTR_SDA_EF_SCL_DR;
  //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_IN_FL_IT);
  I2C_PORT->DDR &= (uint8_t)(~(I2C_SDA_PIN));  // Input
  I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN));  // Float
  //GPIO_Init(I2C_PORT, I2C_SCL_PIN, GPIO_MODE_IN_FL_NO_IT);
  I2C_PORT->DDR &= (uint8_t)(~(I2C_SCL_PIN));  // Input
  I2C_PORT->CR1 &= (uint8_t)(~(I2C_SCL_PIN));  // Float
  //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_FALL_ONLY);
  EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK); 
  EXTI->CR1 |= 0x80;
  I2C_PORT->CR2 &= (uint8_t)(~(I2C_SCL_PIN));  // SCL No Interrupt
  I2C_PORT->CR2 |= (uint8_t)I2C_SDA_PIN;       // SDA Interrupt
}


void i2c_listen
(
   uint8_t address,
   i2c_start_write_callback on_start_write,
   i2c_start_read_callback on_start_read,
   i2c_stop_callback on_stop,
   i2c_data_received_callback on_data_received,
   i2c_data_needed_callback on_data_needed
)
{
  _my_address = address;
  _on_start_write = on_start_write;
  _on_start_read = on_start_read;
  _on_stop = on_stop;
  _on_data_received = on_data_received;
  _on_data_needed = on_data_needed;
  
  // Initialize TIM2 for incoming timeout.
  // An AVR running at 8MHz has minimal TWI speed of 16KHz.
  // Transfer complete buffer takes 11.49ms (Adafruit's Arduino library), we
  // set timeout 16ms
  // Clock is 16MHz, use Prescaler 2048, each counter is 2048  / 16MHz = 128us,
  // Period = (16ms / 128us - 1) = (125 - 1) = 124
  // TIM2_TimeBaseInit(TIM2_PRESCALER_2048, 132);
  TIM2->PSCR = (uint8_t)(TIM2_PRESCALER_2048);
  TIM2->ARRH = (uint8_t)(0);
  TIM2->ARRL = (uint8_t)(124);
  //TIM2_UpdateRequestConfig(TIM2_UPDATESOURCE_GLOBAL);
  TIM2->CR1 &= (uint8_t)(~TIM2_CR1_URS);     // Only update event will set UIF
  TIM2->EGR = TIM2_PSCRELOADMODE_IMMEDIATE;  // Cause immediate update
  // TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  TIM2->IER |= (uint8_t)(TIM2_IT_UPDATE);
  // TIM2_Cmd(ENABLE);
  TIM2->CR1 |= (uint8_t)(TIM2_CR1_CEN);
  // Once timer enables, an update event will be immediately generated.
	// I2C listener will be initialized inside update event handler.
}


static void _SDA_ISR(uint8_t SDA, uint8_t SCL)
{
  // Was INTR_SDA_EF_SCL_DR (initial waiting start/stop)
  // or Was INTR_SDA_ERF_SCL_EF (looking for start/stop after SCL_W1LH
  if (SCL)
  {
    // Start/Stop condition
    if (SDA)
    {
      // SDA rise, Stop condition
      // Disable timeout
      //TIM2_Cmd(DISABLE);
      TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN);
      //TIM2_ITConfig(TIM2_IT_UPDATE, DISABLE);
      TIM2->IER &= (uint8_t)(~TIM2_IT_UPDATE);
      // OnStop Callback
      if (_on_stop) 
        _on_stop();
      // Restart listening
      _scl_state = &(_SCL_WRT[0]);
      _init_port();
    }
    else
    {
      // SDA fall, Start condition
      _i2c_data = 0;
      _scl_state = &(_SCL_WRT[0]);
      _br_clk8hl = N_ACK;  // Default not acknowledged
      // Enable timeout
      //TIM2_SetCounter(0);
      TIM2->CNTRH = (uint8_t)(0);
      TIM2->CNTRL = (uint8_t)(0);
      //TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
      TIM2->IER |= (uint8_t)TIM2_IT_UPDATE;
      //TIM2_Cmd(ENABLE);
      TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
      // Was INTR_SDA_ERF_SCL_DR
      //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_IN_FL_NO_IT);      
      I2C_PORT->DDR &= (uint8_t)(~(I2C_SDA_PIN));  // Input
      I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN));  // Float
      // GPIO_Init(I2C_PORT, I2C_SCL_PIN, GPIO_MODE_IN_FL_IT);
      I2C_PORT->DDR &= (uint8_t)(~(I2C_SCL_PIN));  // Input
      I2C_PORT->CR1 &= (uint8_t)(~(I2C_SCL_PIN));  // Float
      //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_RISE_ONLY);
      EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK);
      EXTI->CR1 |= 0x40;
      _interrupt_mode = INTR_SDA_DR_SCL_ER;
      I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
      // Now INTR_SDA_DR_SCL_ER
    }
  }
  else 
  {
    // Not Start/Stop condition, SCL/SDA lines are used by other transfers
    // Or after SCL_W7LH and determined not my address
    // Must re-arm the interrupts since we disable them inside PORT ISR
    I2C_PORT->CR2 |= (uint8_t)I2C_SDA_PIN;  // Enable SDA Interrupt
  } 
}


static void _SCL_ISR(uint8_t SDA, uint8_t SCL)
{
  switch (*_scl_state)
  {
  case SCL_W1LH: // Bit 1 (first SCL rising edge)
    _i2c_data = SDA ? (_i2c_data << 1 | 0x01) : (_i2c_data << 1); // Shift bit in
    ++_scl_state; // Next state
    // Was INTR_SDA_DR_SCL_ER
    //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_IN_FL_IT);
    I2C_PORT->DDR &= (uint8_t)(~(I2C_SDA_PIN));  // Input
    I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN));  // Float
    //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_RISE_FALL);
    EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK);
    EXTI->CR1 |= 0xc0;
    _interrupt_mode = INTR_SDA_ERF_SCL_EF;
    I2C_PORT->CR2 |= (uint8_t)(I2C_SCL_PIN | I2C_SDA_PIN);  // Enable SCL/SDA Interrupt
    // Now INTR_SDA_ERF_SCL_EF
    break;
  case SCL_W1HL:
    ++_scl_state; // Next state
    // Was INTR_SDA_ERF_SCL_EF
    //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_IN_FL_NO_IT);
    I2C_PORT->DDR &= (uint8_t)(~(I2C_SDA_PIN));  // Input
    I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN));  // Float
    //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_RISE_ONLY);
    EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK);
    EXTI->CR1 |= 0x40;
    _interrupt_mode = INTR_SDA_DR_SCL_ER;
    I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
    // Now INTR_SDA_DR_SCL_ER
    break;
  case SCL_W2to6LH:
    // Was INTR_SDA_DR_SCL_ER
    _i2c_data = SDA ? (_i2c_data << 1 | 0x01) : (_i2c_data << 1); // Shift bit in
    ++_scl_state; // Next state
    I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
    // Now INTR_SDA_DR_SCL_ER (no change)
    break;
  case SCL_W7LH:
    // Was INTR_SDA_DR_SCL_ER
    _i2c_data = SDA ? (_i2c_data << 1 | 0x01) : (_i2c_data << 1); // Shift bit in
    if (_br_clk8hl == W_ACK || _br_clk8hl == W_CALLBACK)
    {
      // Write operation and already acknowledged
      _br_clk8lh = WRITE_IN;
      ++_scl_state; // Next state
      I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
      // Now INTR_SDA_DR_SCL_ER (no change)
    }
    else
    {
      if (_i2c_data == _my_address)
      {
        // This is for me
        _br_clk8lh = RW_BIT;
        ++_scl_state; // Next state
        I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
        _interrupt_mode = INTR_SDA_DR_SCL_ER;
        // Now INTR_SDA_DR_SCL_ER (no change)
      }
      else
      {
        // Not my address
        // Disable timtout
        //TIM2_Cmd(DISABLE);
        TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN);
        //TIM2_ITConfig(TIM2_IT_UPDATE, DISABLE);
        TIM2->IER &= (uint8_t)(~TIM2_IT_UPDATE);
        // Restart listening
        _scl_state = &(_SCL_WRT[0]);
        _init_port();
      }
    }
    break;
  case SCL_W8LH:
    // Was INTR_SDA_DR_SCL_ER
    switch (_br_clk8lh)
    {
    case RW_BIT:
      if (SDA)
      {
        _br_clk8hl = R_ACK; // Read
        if (_on_start_read) // Callback
          _on_start_read();
      }
      else
      {
        _br_clk8hl = W_ACK;  // Write
        if (_on_start_write) // Callback
          _on_start_write();
      }
      break;
    case WRITE_IN:
      _i2c_data = SDA ? (_i2c_data << 1 | 0x01) : (_i2c_data << 1); // Shift bit in
      _br_clk8hl = W_CALLBACK;
      break;
    }
    ++_scl_state; // Next state
    //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_FALL_ONLY);
    EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK);
    EXTI->CR1 |= 0x80;
    _interrupt_mode = INTR_SDA_DR_SCL_EF;
    I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
    // Now INTR_SDA_DR_SCL_EF
    break;
  case SCL_W8HL:
    // Was INTR_SDA_DR_SCL_EF
    switch (_br_clk8hl)
    {
    case W_ACK:
      // Acknowledge slave address
      //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_OUT_OD_LOW_FAST);
      I2C_PORT->ODR &= (uint8_t)(~(I2C_SDA_PIN)); // Low
      I2C_PORT->DDR |= (uint8_t)I2C_SDA_PIN;      // Output
      I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN)); // Open-Drain
      I2C_PORT->CR2 |= (uint8_t)I2C_SDA_PIN;      // Fast
      ++_scl_state; // Next state
      I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
      // Now INTR_SDA_DR_SCL_EF, no change but SDA is output if data acknowldged
      break;
    case W_CALLBACK:
      // Clock stretching, pull SCL low until we finish data_received callback
      //GPIO_Init(I2C_PORT, I2C_SCL_PIN, GPIO_MODE_OUT_OD_LOW_FAST);
      I2C_PORT->ODR &= (uint8_t)(~(I2C_SCL_PIN)); // Low
      I2C_PORT->DDR |= (uint8_t)I2C_SCL_PIN;      // Output
      I2C_PORT->CR1 &= (uint8_t)(~(I2C_SCL_PIN)); // Open-Drain
      I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;      // Fast
      // Callback _on_data_received
      if (_on_data_received && _on_data_received(_i2c_data))
      {
        // Send ACK if data acknowledged
        //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_OUT_OD_LOW_FAST);
        I2C_PORT->ODR &= (uint8_t)(~(I2C_SDA_PIN)); // Low
        I2C_PORT->DDR |= (uint8_t)I2C_SDA_PIN;      // Output
        I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN)); // Open-Drain
        I2C_PORT->CR2 |= (uint8_t)I2C_SDA_PIN;      // Fast
      }
      // Restore SCL pin state EF
      GPIO_Init(I2C_PORT, I2C_SCL_PIN, GPIO_MODE_IN_FL_IT);
      I2C_PORT->DDR &= (uint8_t)(~(I2C_SCL_PIN));  // Input
      I2C_PORT->CR1 &= (uint8_t)(~(I2C_SCL_PIN));  // Float
      ++_scl_state; // Next state
      I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
      // Now INTR_SDA_DR_SCL_EF, no change but SDA is output if data acknowldged
      break;
    case R_ACK:
      // I'm too busy to implement I2C read at this time :D
      // The actual implementation should look like
      // _i2c_data = _on_data_needed();
      // _scl_state = &(_SCL_RD[0]);
      // ...
      // break;
      // Now I simply ignore write
    case N_ACK:
      // Disable timeout
      //TIM2_Cmd(DISABLE);
      TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN);
      //TIM2_ITConfig(TIM2_IT_UPDATE, DISABLE);
      TIM2->IER &= (uint8_t)(~TIM2_IT_UPDATE);
      // Restart listening
      _scl_state = &(_SCL_WRT[0]);
      _init_port();
      break;
    }
    break;
  case SCL_W9HL:
    // Was INTR_SDA_DR_SCL_EF and SDA possible in output state
    //GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_MODE_IN_FL_NO_IT);  // Set SDA input
    I2C_PORT->DDR &= (uint8_t)(~(I2C_SDA_PIN));  // Input
    I2C_PORT->CR1 &= (uint8_t)(~(I2C_SDA_PIN));  // Float
    _i2c_data = 0;  // Reset data for next byte
    _scl_state = &(_SCL_WRT[0]);
    //EXTI_SetExtIntSensitivity(I2C_EXTI_PORT, EXTI_SENSITIVITY_RISE_ONLY);
    EXTI->CR1 &= (uint8_t)(~I2C_EXTI_SENSITIVITY_MASK);
    EXTI->CR1 |= 0x40;
    _interrupt_mode = INTR_SDA_DR_SCL_ER;
    I2C_PORT->CR2 |= (uint8_t)I2C_SCL_PIN;  // Enable SCL Interrupt
    // Now INTR_SDA_DR_SCL_ER
    break;
  }
}


// ISR for SDA and SCL pins
void I2C_GPIO_ISR(void)
{
  uint8_t Port, SCL, SDA;
  
  Port = I2C_PORT->IDR;
  SDA = Port & I2C_SDA_PIN;
  SCL = Port & I2C_SCL_PIN;
  
  // Disable interrupt
  I2C_PORT->CR2 &= (uint8_t)(~(I2C_SCL_PIN | I2C_SDA_PIN));
    
  switch (_interrupt_mode)
  {
  case INTR_SDA_DR_SCL_EF:
    // call SCL ISR
    _SCL_ISR(SDA, SCL);
    break;
  case INTR_SDA_DR_SCL_ER:
    // call SCL ISR
    _SCL_ISR(SDA, SCL);
    break;
  case INTR_SDA_EF_SCL_DR:
    // SDA ISR
    _SDA_ISR(SDA, SCL);
    break;
  case INTR_SDA_ERF_SCL_EF:
    // Could be SDA rise/fall or SCL fall
    if (!SCL) 
      _SCL_ISR(SDA, SCL); // SCL fall
    else
      _SDA_ISR(SDA, SCL); // SDA rise/fall
    break;
  }
}


// ISR for timeout
void I2C_TIM2_UPDATE_ISR(void)
{
  // Disable timeout
  //TIM2_Cmd(DISABLE);
  TIM2->CR1 &= (uint8_t)(~TIM2_CR1_CEN);
  //TIM2_ITConfig(TIM2_IT_UPDATE, DISABLE);
  TIM2->IER &= (uint8_t)(~TIM2_IT_UPDATE);
  // Restart listening
  _scl_state = &(_SCL_WRT[0]);
  _init_port();
}


