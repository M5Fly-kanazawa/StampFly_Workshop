#ifndef LED_HPP
#define LED_HPP
#include <FastLED.h>

#define BOARD_LED_PIN 39
#define STAMP_LED_PIN 21

#define WHITE 0xffffff
#define RED 0xff0000
#define GREEN 0x00ff00
#define BLUE 0x0000ff
#define PERPLE 0xff00ff
#define YELLOW 0xffff00
#define POWEROFFCOLOR 0x18EBF9

#define TAIL_LED 0
#define BOTTOM_LED 1

extern CRGB Stamp_led[1];
extern CRGB Board_led[2];

void init_led(uint8_t state);
void board_tail_led(CRGB color, uint8_t state);
void board_bottom_led(CRGB color, uint8_t state);
void stamp_led(CRGB color, uint8_t state);
void blink_led(void);


#endif