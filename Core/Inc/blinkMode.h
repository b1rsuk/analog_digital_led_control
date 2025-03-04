/*
 * BlinkMode.h
 *
 *  Created on: Feb 22, 2025
 *      Author: borsuk
 */

#ifndef INC_BLINKMODE_H_
#define INC_BLINKMODE_H_

typedef enum {
    LED_BLINK_OFF = 0,
    LED_BLINK_SLOW_SYNC = 1,
    LED_BLINK_FAST_SYNC = 2,
    LED_BLINK_FAST_SEQ = 3,
    LED_BLINK_SLOW_SEQ = 4,
    LED_BLINK_RANDOM = 5
} LedBlinkMode;
extern LedBlinkMode ledBlinkMode;
#endif /* INC_BLINKMODE_H_ */
