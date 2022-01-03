//
// Code from https://forum.digikey.com/t/using-the-segment-lcd-controller-on-the-kinetis-kl46/13277
//

#ifndef _LCD_H_
#define _LCD_H_       /**< Symbol preventing repeated inclusion */

void lcd_ini(void);
void lcd_set(uint8_t value, uint8_t digit);
void lcd_display_dec(uint16_t value);
void lcd_display_hex(uint16_t value);
void lcd_display_time(uint8_t value1, uint8_t value2);
void lcd_display_error(uint8_t errorNum);
void lcd_clear();
void lcd_blink(uint8_t bool);

// Define Number of Front and Back plane pins
#define LCD_NUM_FRONTPLANE_PINS 8
#define LCD_NUM_BACKPLANE_PINS 4

// Create macros for segments
#define LCD_SEG_D 0x11
#define LCD_SEG_E 0x22
#define LCD_SEG_G 0x44
#define LCD_SEG_F 0x88
#define LCD_SEG_DECIMAL 0x11
#define LCD_SEG_C 0x22
#define LCD_SEG_B 0x44
#define LCD_SEG_A 0x88
#define LCD_CLEAR 0x00

// Create Macros for each pin
#define LCD_FRONTPLANE0 37u  // (pin5 in LCD-S401: digit 1)
#define LCD_FRONTPLANE1 17u  // (pin6 in LCD-S401: digit 1)
#define LCD_FRONTPLANE2 7u   // (pin7 in LCD-S401: digit 2)
#define LCD_FRONTPLANE3 8u   // (pin8 in LCD-S401: digit 2)
#define LCD_FRONTPLANE4 53u  // (pin9 in LCD-S401: digit 3)
#define LCD_FRONTPLANE5 38u  // (pin10 in LCD-S401: digit 3)
#define LCD_FRONTPLANE6 10u  // (pin11 in LCD-S401: digit 4)
#define LCD_FRONTPLANE7 11u  // (pin12 in LCD-S401: digit 4)
#define LCD_BACKPLANE0 40u   // (pin1 in LCD-S401: COM 0)
#define LCD_BACKPLANE1 52u   // (pin2 in LCD-S401: COM 1)
#define LCD_BACKPLANE2 19u   // (pin3 in LCD-S401: COM 2)
#define LCD_BACKPLANE3 18u   // (pin4 in LCD-S401: COM 3)

// Macros for turning decimal points and colon on and off
#define SegLCD_DP1_On() LCD->WF8B[LCD_FRONTPLANE1]  |= LCD_SEG_DECIMAL;
#define SegLCD_DP1_Off() LCD->WF8B[LCD_FRONTPLANE1] &= ~LCD_SEG_DECIMAL;
#define SegLCD_DP2_On() LCD->WF8B[LCD_FRONTPLANE3]  |= LCD_SEG_DECIMAL;
#define SegLCD_DP2_Off() LCD->WF8B[LCD_FRONTPLANE3] &= ~LCD_SEG_DECIMAL;
#define SegLCD_DP3_On() LCD->WF8B[LCD_FRONTPLANE5]  |= LCD_SEG_DECIMAL;
#define SegLCD_DP3_Off() LCD->WF8B[LCD_FRONTPLANE5] &= ~LCD_SEG_DECIMAL;
#define SegLCD_Col_On() LCD->WF8B[LCD_FRONTPLANE7]  |= LCD_SEG_DECIMAL;
#define SegLCD_Col_Off() LCD->WF8B[LCD_FRONTPLANE7] &= ~LCD_SEG_DECIMAL;

#endif  /* _LCD_H_ */
