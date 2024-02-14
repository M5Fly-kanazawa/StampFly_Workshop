#include "led.hpp"

CRGB Stamp_led[1];
CRGB Board_led[2];

uint32_t LED_counter=0;

void init_led(uint8_t state)
{
  //Initialaze LED function

  //LED 初期化
  FastLED.addLeds<WS2812, BOARD_LED_PIN, GRB>(Board_led, 2);
  FastLED.addLeds<WS2812, STAMP_LED_PIN, GRB>(Stamp_led, 1);

  //色設定
  Board_led[0] = RED;
  Stamp_led[TAIL_LED] = WHITE;
  Stamp_led[BOTTOM_LED] = BLUE;
  
  //発色
  if (state!=0) FastLED.show();
}

void board_tail_led(CRGB color, uint8_t state)
{
    if(state!=0) Board_led[TAIL_LED] = color;
    else Board_led[TAIL_LED] = 0x000000;
}

void board_bottom_led(CRGB color, uint8_t state)
{
    if(state!=0) Board_led[BOTTOM_LED] = color;
    else Board_led[BOTTOM_LED] = 0x000000;
}

void stamp_led(CRGB color, uint8_t state)
{
    if(state!=0) Stamp_led[0] = color;
    else Stamp_led[0] = 0x000000;
}


void blink_led(void)
{
    //ここに自分のコードを追加する（引数が必要なら変更指定良い）
    LED_counter++;
    if (LED_counter<400)
        board_bottom_led(RED, 1);
    else if(LED_counter <800)
        board_bottom_led(RED, 0);
    else LED_counter = 0;
       
}
