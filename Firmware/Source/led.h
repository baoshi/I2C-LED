#ifndef LED_H
#define LED_H


typedef enum
{
  LED_BLINK_OFF = (uint8_t)0,
  LED_BLINK_2HZ,
  LED_BLINK_1HZ,
  LED_BLINK_HALFHZ
} led_blink_type;


void led_init(void);
void led_on(void);
void led_off(void);
void led_set_duty(uint8_t duty);
void led_set_blink(led_blink_type blink);
void led_set_digit(uint8_t digit, uint8_t valuie);
#endif
