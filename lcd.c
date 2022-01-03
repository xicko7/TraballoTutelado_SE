//
// Code from https://forum.digikey.com/t/using-the-segment-lcd-controller-on-the-kinetis-kl46/13277
//

#include "MKL46Z4.h"
#include "lcd.h"

const static uint8_t LCD_Frontplane_Pin[LCD_NUM_FRONTPLANE_PINS] = {
    LCD_FRONTPLANE0, LCD_FRONTPLANE1, LCD_FRONTPLANE2, LCD_FRONTPLANE3,
    LCD_FRONTPLANE4, LCD_FRONTPLANE5, LCD_FRONTPLANE6, LCD_FRONTPLANE7};

/* NOT USED
const static uint8_t LCD_Backplane_Pin[LCD_NUM_BACKPLANE_PINS] = {
  LCD_BACKPLANE0, LCD_BACKPLANE1, LCD_BACKPLANE2, LCD_BACKPLANE3
};
*/

void lcd_ini(void)
{
  SIM->COPC = 0;

  // Enable Clock to ports B, C, D and E, and SLCD Peripheral
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_SLCD_MASK;

  // Set PADSAFE to disable LCD while configuring
  LCD->GCR |= LCD_GCR_PADSAFE_MASK;

  // Clear LCDEN (LCD Enable) while configuring
  LCD->GCR &= ~LCD_GCR_LCDEN_MASK;

  // Configure pins
  // -> From Reference manual, set pins to MUX 0 for normal LCD display
  //    operation, only use MUX 7 if using LCD fault detection
  //
  PORTB->PCR[7] = PORT_PCR_MUX(0u);  // Set PTB7 to LCD_P7
  PORTB->PCR[8] = PORT_PCR_MUX(0u);  // Set PTB8 to LCD_P8
  PORTB->PCR[10] = PORT_PCR_MUX(0u); // Set PTB10 to LCD_P10
  PORTB->PCR[11] = PORT_PCR_MUX(0u); // Set PTB11 to LCD_P11
  PORTB->PCR[21] = PORT_PCR_MUX(0u); // Set PTB21 to LCD_P17
  PORTB->PCR[22] = PORT_PCR_MUX(0u); // Set PTB22 to LCD_P18
  PORTB->PCR[23] = PORT_PCR_MUX(0u); // Set PTB23 to LCD_P19
  PORTC->PCR[17] = PORT_PCR_MUX(0u); // Set PTC17 to LCD_P37
  PORTC->PCR[18] = PORT_PCR_MUX(0u); // Set PTC18 to LCD_P38
  PORTD->PCR[0] = PORT_PCR_MUX(0u);  // Set PTD0 to LCD_P40
  PORTE->PCR[4] = PORT_PCR_MUX(0u);  // Set PTE4 to LCD_P52
  PORTE->PCR[5] = PORT_PCR_MUX(0u);  // Set PTE5 to LCD_P53

  // Configure LCD Registers
  // Configure LCD_GCR - General Control Register, controls most options
  //                     in LCD Peripheral
  //
  LCD->GCR =

      // Set LCD_GCR_SOURCE, LCD_GCR_ALTSOURCE / LCD_GCR_ALTDIV
      // Part of setting clock source to OSCERCLK, or external oscillator
      // See pgs 135, 111, 124 and 869-873 in KL46 doc for references
      LCD_GCR_SOURCE(1) |    // Select AltClk instead of default (ERCLK32K)
      LCD_GCR_ALTSOURCE(0) | // Select MCGIRCLK
      LCD_GCR_ALTDIV(0) |    // No divide alt clk

      // LCD_GCR_RVEN_MASK |  //Clear LCD_GCR_RVEN, disable voltage regulator
      LCD_GCR_RVTRIM(0x00) | // Set RVTRIM to 0, irrelevant as voltage regulator is disabled, but setting it to a known state
      LCD_GCR_CPSEL_MASK |   // Set LCD_GCR_CPSEL to use capacitor charge pump
      LCD_GCR_LADJ(0x03) |   // Set LCD_GCR_LADJ to 11, slow clock rate = lower power, but higher load capacitance on the LCD requires higher clock speed
      // LCD_GCR_VSUPPLY |    //Clear LCD_GCR_VSUPPLY, drive VLL3 externally
      LCD_GCR_PADSAFE_MASK | // Set LCD_GCR_PADSAFE, leave enabled during configuration process
      // LCD_GCR_FDCIEN_MASK |    //Clear LCD_GCR_FDCIEN, No interrupt from fault dection
      LCD_GCR_FFR_MASK | // Set LCD_GCR_FFR, allow an LCD Frame Frequency of 46.6Hz to 146.2Hz.  Disable to change range to 23.3Hz to 73.1Hz
      // LCD_GCR_LCDDOZE_MASK |   //Clear LCD_GCR_LCDDOZE, allows LCD peripheral to run even in doze mode.  Set to disable LCD in doze mode
      // LCD_GCR_LCDSTP_MASK |    //Clear LCD_GCR_LCDSTP, allows LCD peripheral to run even in stop mode.  Set to disable LCD in stop mode
      // LCD_GCR_LCDEN_MASK |     //Clear LCD_GCR_LCDEN, Disables all front and backplane pins.  Leave disabled during configuration process
      LCD_GCR_LCLK(0x04) | // Set LCD_GCR_LCLK to 111, LCD Clock prescaler where LCD controller frame frequency = LCD clock/((DUTY  |  1) x 8 x (4 | LCLK[2:0]) x Y), where Y = 2, 2, 3, 3, 4, 5, 8, 16 chosen by module duty cycle config
      LCD_GCR_DUTY(0x03);  // Set LCD_GCR_DUTY to 011, Have 4 backplane pins, so need a 1/4 duty cycle

  // Configure LCD_SEG_AR  - Auxiliary Register, controls blinking of LCD
  //
  LCD->AR =             // PARA QUE PESTANEXE
      LCD_AR_BLINK(0) | // Clear LCD_SEG_AR_BLINK, Disable SLCD blinking. Enable to make LCD Blink
      // LCD_AR_ALT_MASK |   //Clear LCD_SEG_AR_ALT, if enabled LCD back plane sequencer changes to an alternate display.  Only functional if DUTY[2:0] is less than 100(binary). This allows a blink screen that is not blank
      // LCD_AR_BLANK_MASK | //Clear LCD_SEG_AR_BLANK, asserting bit clears all segments in LCD.
      // LCD_AR_BMODE_MASK | //Clear LCD_SEG_AR_BMODE, if enabled displays alternate display during blink period instead of blank.
      LCD_AR_BRATE(0x00); // Set LCD_SEG_AR_BRATE to 000.  Frequency of blink is determined by LCD clock/(2^(12 + BRATE))

  // Configure LCD_SEG_FDCR - Fault Detect Control Register, controls
  //                          use of Fault Detect features of SLCD.
  //
  LCD->FDCR = 0x00000000; // Clear all bits in FDCR.  As this will not be covering use of fault detect, this register is cleared.

  // Configure LCD_PENn - Pin Enable Register, controls which of the
  //                      possible LCD pins are used
  //
  // LCD->PEN[0] contains bits 0-31, while LCD->PEN[1] contains bits 32-63
  // For pins >= 32, set in LCD->PEN[1] as LCD_PEN_PEN(Pin#-32)
  //
  LCD->PEN[0] = LCD_PEN_PEN(1u << 7u) |  // LCD_P7
                LCD_PEN_PEN(1u << 8u) |  // LCD_P8
                LCD_PEN_PEN(1u << 10u) | // LCD_P10
                LCD_PEN_PEN(1u << 11u) | // LCD_P11
                LCD_PEN_PEN(1u << 17u) | // LCD_P17
                LCD_PEN_PEN(1u << 18u) | // LCD_P18
                LCD_PEN_PEN(1u << 19u);  // LCD_P19

  LCD->PEN[1] = LCD_PEN_PEN(1u << 5u) |  // LCD_P37
                LCD_PEN_PEN(1u << 6u) |  // LCD_P38
                LCD_PEN_PEN(1u << 8u) |  // LCD_P40
                LCD_PEN_PEN(1u << 20u) | // LCD_P52
                LCD_PEN_PEN(1u << 21u);  // LCD_P53

  // Configure LCD_SEG_BPENn - Back Plane Enable Register, controls which
  //                           pins in LCD->PEN are Back Plane (commons)
  //
  // LCD->BPEN[0] contains bits 0-31, while LCD->BPEN[1] contains bits 32-63
  // For pins >= 32, set in LCD->BPEN[1] as LCD_BPEN_BPEN(Pin#-32)
  //
  LCD->BPEN[0] = LCD_BPEN_BPEN(1u << 18u) | // LCD_P18  COM3
                 LCD_BPEN_BPEN(1u << 19u);  // LCD_P19, COM2

  LCD->BPEN[1] = LCD_BPEN_BPEN(1u << 8u) | // LCD_P40, COM0
                 LCD_BPEN_BPEN(1u << 20u); // LCD_P52, COM1

  // Configure LCD_WFyTOx - Configures 4 Waveform signals, LCD_WF[z] is
  //                        defined such that x = (z*4) and y = 3 | (z*4)
  //
  // Where x is the n index value of WFn on the least significant byte
  // (bits 7-0) and y is the n index value of WFn on the most significant
  // byte (bits 31-24)
  //
  // Note that "Disabled" is used for pins that are not set as LCD pins,
  // where "Off" is used for pins that are set as LCD, but are just inactive.
  //
  LCD->WF[0] = LCD_WF_WF0(0x00) | // WF Pin 0 Disabled
               LCD_WF_WF1(0x00) | // WF Pin 1 Disabled
               LCD_WF_WF2(0x00) | // WF Pin 2 Disabled
               LCD_WF_WF3(0x00);  // WF Pin 3 Disabled

  LCD->WF[1] = LCD_WF_WF4(0x00) | // WF Pin 4 Disabled
               LCD_WF_WF5(0x00) | // WF Pin 5 Disabled
               LCD_WF_WF6(0x00) | // WF Pin 6 Disabled
               LCD_WF_WF7(0x00);  // WF Pin 7 Off

  LCD->WF[2] = LCD_WF_WF8(0x00) |  // WF Pin 8 Off
               LCD_WF_WF9(0x00) |  // WF Pin 9 Disabled
               LCD_WF_WF10(0x00) | // WF Pin 10 Off
               LCD_WF_WF11(0x00);  // WF Pin 11 Off

  LCD->WF[3] = LCD_WF_WF12(0x00) | // WF Pin 12 Disabled
               LCD_WF_WF13(0x00) | // WF Pin 13 Disabled
               LCD_WF_WF14(0x00) | // WF Pin 14 Disabled
               LCD_WF_WF15(0x00);  // WF Pin 15 Disabled

  LCD->WF[4] = LCD_WF_WF16(0x00) | // WF Pin 16 Disabled
               LCD_WF_WF17(0x00) | // WF Pin 17 Off
               LCD_WF_WF18(0x88) | // WF Pin 18 (COM3) is enabled on Phases D and H
               LCD_WF_WF19(0x44);  // WF Pin 19 (COM2) is enabled on Phases C and G

  LCD->WF[5] = LCD_WF_WF20(0x00) | // WF Pin 20 Disabled
               LCD_WF_WF21(0x00) | // WF Pin 21 Disabled
               LCD_WF_WF22(0x00) | // WF Pin 22 Disabled
               LCD_WF_WF23(0x00);  // WF Pin 23 Disabled

  LCD->WF[6] = LCD_WF_WF24(0x00) | // WF Pin 24 Disabled
               LCD_WF_WF25(0x00) | // WF Pin 25 Disabled
               LCD_WF_WF26(0x00) | // WF Pin 26 Disabled
               LCD_WF_WF27(0x00);  // WF Pin 27 Disabled

  LCD->WF[7] = LCD_WF_WF28(0x00) | // WF Pin 28 Disabled
               LCD_WF_WF29(0x00) | // WF Pin 29 Disabled
               LCD_WF_WF30(0x00) | // WF Pin 30 Disabled
               LCD_WF_WF31(0x00);  // WF Pin 31 Disabled

  LCD->WF[8] = LCD_WF_WF32(0x00) | // WF Pin 32 Disabled
               LCD_WF_WF33(0x00) | // WF Pin 33 Disabled
               LCD_WF_WF34(0x00) | // WF Pin 34 Disabled
               LCD_WF_WF35(0x00);  // WF Pin 35 Disabled

  LCD->WF[9] = LCD_WF_WF36(0x00) | // WF Pin 36 Disabled
               LCD_WF_WF37(0x00) | // WF Pin 37 Off
               LCD_WF_WF38(0x00) | // WF Pin 38 Off
               LCD_WF_WF39(0x00);  // WF Pin 39 Disabled

  LCD->WF[10] = LCD_WF_WF40(0x11) | // WF Pin 40 (COM0) is enabled on Phases A and E
                LCD_WF_WF41(0x00) | // WF Pin 41 Disabled
                LCD_WF_WF42(0x00) | // WF Pin 42 Disabled
                LCD_WF_WF43(0x00);  // WF Pin 43 Disabled

  LCD->WF[11] = LCD_WF_WF44(0x00) | // WF Pin 44 Disabled
                LCD_WF_WF45(0x00) | // WF Pin 45 Disabled
                LCD_WF_WF46(0x00) | // WF Pin 46 Disabled
                LCD_WF_WF47(0x00);  // WF Pin 47 Disabled

  LCD->WF[12] = LCD_WF_WF48(0x00) | // WF Pin 48 Disabled
                LCD_WF_WF49(0x00) | // WF Pin 49 Disabled
                LCD_WF_WF50(0x00) | // WF Pin 50 Disabled
                LCD_WF_WF51(0x00);  // WF Pin 51 Disabled

  LCD->WF[13] = LCD_WF_WF52(0x22) | // WF Pin 52 (COM1) is enabled on Phases B and F
                LCD_WF_WF53(0x00) | // WF Pin 53 Off
                LCD_WF_WF54(0x00) | // WF Pin 54 Disabled
                LCD_WF_WF55(0x00);  // WF Pin 55 Disabled

  LCD->WF[14] = LCD_WF_WF56(0x00) | // WF Pin 56 Disabled
                LCD_WF_WF57(0x00) | // WF Pin 57 Disabled
                LCD_WF_WF58(0x00) | // WF Pin 58 Disabled
                LCD_WF_WF59(0x00);  // WF Pin 59 Disabled

  LCD->WF[15] = LCD_WF_WF60(0x00) | // WF Pin 60 Disabled
                LCD_WF_WF61(0x00) | // WF Pin 61 Disabled
                LCD_WF_WF62(0x00) | // WF Pin 62 Disabled
                LCD_WF_WF63(0x00);  // WF Pin 63 Disabled

  // Disable GCR_PADSAFE and Enable GCR_LCDEN
  LCD->GCR &= ~LCD_GCR_PADSAFE_MASK; // Clear PADSAFE to unlock LCD pins
  LCD->GCR |= LCD_GCR_LCDEN_MASK;    // Set LCDEN to enable operation of LCD
}

//
// Sets a value from 0-F to a specified Digit, with 1 being the leftmost,
// 4 being the rightmost. Will not display error is Value is outside of
// 0-F, but display will not update
//
void lcd_set(uint8_t value, uint8_t digit)
{
  if (digit > 4)
  {
    // Display "Err" if trying to access a digit that does not exist
    lcd_display_error(0x01);

    return;
  }

  switch (value)
  {

  case 0x00:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x01:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_CLEAR;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = (LCD_SEG_B | LCD_SEG_C);
    break;

  case 0x02:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = (LCD_SEG_D | LCD_SEG_E | LCD_SEG_G);
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = (LCD_SEG_A | LCD_SEG_B);
    break;

  case 0x03:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x04:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x05:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_C;
    break;

  case 0x06:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_C;
    break;

  case 0x07:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_CLEAR;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x08:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x09:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x0A:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A | LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x0B:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_C;
    break;

  case 0x0C:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A;
    break;

  case 0x0D:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_B | LCD_SEG_C;
    break;

  case 0x0E:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A;
    break;

  case 0x0F:
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 2]] = LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
    LCD->WF8B[LCD_Frontplane_Pin[(2 * digit) - 1]] = LCD_SEG_A;
    break;

  default:
    // Display "Err"
    lcd_display_error(0x01);
  }
}

void lcd_clear()
{
  for (int digit = 1; digit <= 4; digit++)
  {
    LCD->WF8B[LCD_Frontplane_Pin[(2*digit)-2]] = LCD_CLEAR;
    LCD->WF8B[LCD_Frontplane_Pin[(2*digit)-1]] = LCD_CLEAR;
  }
}

void lcd_blink(uint8_t bool)
{
  LCD->AR = (LCD_AR_BLINK(bool)) | (LCD_AR_BRATE(0x02));
}

//
// Displays a 4 Digit number in decimal
//
void lcd_display_dec(uint16_t value)
{
  if (value > 9999)
  {
    // Display "Err" if value is greater than 4 digits
    lcd_display_error(0x10);
  }
  else
  {
    lcd_set(value / 1000, 1);
    lcd_set((value - (value / 1000) * 1000) / 100, 2);
    lcd_set((value - (value / 100) * 100) / 10, 3);
    lcd_set(value - (value / 10) * 10, 4);
  }
}

//
// Displays a 16 bit number in Hex Format
//
void lcd_display_hex(uint16_t value)
{
  lcd_set((value & 0xF000) >> 12, 1);
  lcd_set((value & 0x0F00) >> 8, 2);
  lcd_set((value & 0x00F0) >> 4, 3);
  lcd_set((value & 0x000F) >> 0, 4);
}

//
// Displays 2 values separated by a colon
//
void lcd_display_time(uint8_t value1, uint8_t value2)
{
  if ((value1 > 99) | (value2 > 99))
  {
    // Display "Err" if either value is greater than 2 digits
    lcd_display_error(0x10);
  }
  else
  {
    lcd_set(value1 / 10, 1);
    lcd_set(value1 % 10, 2);
    lcd_set(value2 / 10, 3);
    lcd_set(value2 % 10, 4);
    SegLCD_Col_On();
  }
}

//
// Displays Err# on screen, where # is a value 0-F.
// If ErrorNum is outside of that range, just displays Err
//
void lcd_display_error(uint8_t errorNum)
{
  LCD->WF8B[LCD_FRONTPLANE0] = LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G;
  LCD->WF8B[LCD_FRONTPLANE1] = LCD_SEG_A;
  LCD->WF8B[LCD_FRONTPLANE2] = LCD_SEG_E | LCD_SEG_G;
  LCD->WF8B[LCD_FRONTPLANE3] = LCD_CLEAR;
  LCD->WF8B[LCD_FRONTPLANE4] = LCD_SEG_E | LCD_SEG_G;
  LCD->WF8B[LCD_FRONTPLANE5] = LCD_CLEAR;
  if (errorNum < 0x10)
  {
    // Display ErrorNum in digit 4 if within valid range. If not, leave blank.
    lcd_set(errorNum, 4);
  }
  else
  {
    LCD->WF8B[LCD_FRONTPLANE6] = LCD_CLEAR;
    LCD->WF8B[LCD_FRONTPLANE7] = LCD_CLEAR;
  }
}
