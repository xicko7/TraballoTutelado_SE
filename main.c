#include "MKL46Z4.h"
#include "lcd.h"
#include "MMA8451Q.h"
#include "I2C.h"

#define ACC_CONTROL_VALUE 100
#define MAX_ANGLE 819

volatile uint32_t button = 0;
volatile uint8_t Xoffset, Yoffset, Zoffset;
volatile uint16_t Xout_14_bit, Yout_14_bit, Zout_14_bit;
uint8_t acc_output[6];

/******************************************************************************
 * CONVERSIÓN A GRADOS ANGULARES
 ******************************************************************************/
uint16_t to360(volatile uint16_t value)
{

  // uint16_t res, pot5 = 0, pot6 = 0;

  // pot5 = value >> 5; // value/2^5
  // pot6 = value >> 6; // value/2^6

  // res = pot5 + pot6;
  // res = res >> 1; // Media

  // if (res > 360)
  //   res = 360;

  // return res;

  return value >> 3;
}

/******************************************************************************
 * INICIALIZACIÓN CLK LCD
 ******************************************************************************/
void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); // 0 32KHZ internal reference clock; 1= 4MHz irc
}

/******************************************************************************
 * DELAY
 ******************************************************************************/
void millis(int ms)
{
  volatile int i;

  for (i = 0; i < ms * 1000; i++)
    ;
}

/******************************************************************************
 * BOTÓNS
 ******************************************************************************/
void buttons_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);  // IRQ on falling edge
  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge

  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}

/******************************************************************************
 * LEDS
 ******************************************************************************/
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR |= (1 << 5);
  GPIOE->PSOR |= (1 << 29);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}

void led_green(uint8_t x)
{
  if (x > 0)
    GPIOD->PSOR |= (1 << 5);
  else
    GPIOD->PSOR &= ~(1 << 5);
}

void led_red(uint8_t x)
{
  if (x > 0)
    GPIOE->PSOR |= (1 << 29);
  else
    GPIOE->PSOR &= ~(1 << 29);
}

/******************************************************************************
 * ACELERÓMETRO
 ******************************************************************************/
void MCU_Init(void)
{
  // I2C0 module initialization
  SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;  // Turn on clock to I2C0 module
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; // Turn on clock to Port E module
  PORTE->PCR[24] = PORT_PCR_MUX(5);   // PTE24 pin is I2C0 SCL line
  PORTE->PCR[25] = PORT_PCR_MUX(5);   // PTE25 pin is I2C0 SDA line
  I2C0->F = 0x14;                     // SDA hold time = 2.125us, SCL start hold time = 4.25us, SCL stop hold time = 5.125us *
  I2C0->C1 = I2C_C1_IICEN_MASK;       // Enable I2C0 module

  // Configure the PTA14 pin (connected to the INT1 of the MMA8451Q) for falling edge interrupts
  /*
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Turn on clock to Port A module
  PORTA->PCR14 |= (0|PORT->PCR_ISF_MASK|	// Clear the interrupt flag
            PORT->PCR_MUX(0x1)|	// PTA14 is configured as GPIO
            PORT->PCR_IRQC(0xA));	// PTA14 is configured for falling edge interrupts
     */

  // Configure the PTC5 pin (connected to the INT1 of the MMA8451Q) for falling edge interrupts
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;       // Turn on clock to Port C module
  PORTC->PCR[5] |= (0 | PORT_PCR_ISF_MASK | // Clear the interrupt flag
                    PORT_PCR_MUX(0x1) |     // PTC5 is configured as GPIO
                    PORT_PCR_IRQC(0xA));    // PTC5 is configured for falling edge interrupts

  NVIC_EnableIRQ(31);

  NVIC->ICPR[0U] |= 1 << ((PORTC_PORTD_IRQn - 16) % 32);
  NVIC->ISER[0U] |= 1 << ((PORTC_PORTD_IRQn - 16) % 32);
}

void Accelerometer_Init(void)
{

  I2C_ReadRegister(MMA845x_I2C_ADDRESS, WHO_AM_I_REG);

  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x40); // Reset all registers to POR values

  while (I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG2) & 0x40)
    ; // Wait for the RST bit to clear

  I2C_WriteRegister(MMA845x_I2C_ADDRESS, XYZ_DATA_CFG_REG, 0x00); // +/-2g range -> 1g = 16384/4 = 4096 counts
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x02);        // High Resolution mode
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG3, 0x00);        // Push-pull, active low interrupt
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x01);        // Enable DRDY interrupt
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x01);        // DRDY interrupt routed to INT1 - PTC5
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x3D);        // ODR = 1.56Hz, Reduced noise, Active mode
}

uint16_t acc_control(uint16_t tmp, uint16_t ant)
{

  if (tmp > ant)
  {
    if (tmp - ant > ACC_CONTROL_VALUE)
      return tmp;
    else
      return ant;
  }
  else
  {
    if (ant - tmp > ACC_CONTROL_VALUE)
      return tmp;
    else
      return ant;
  }
}

void acc_read(void)
{

  /**
   * Control de sensibilidade para que o valor do ángulo non se actualice se non varía demasiado
   * coa finalidade de evitar refrescos no LCD cando o acelerómetro está en repouso
   *
   */

  // while (!(I2C_ReadRegister(MMA845x_I2C_ADDRESS, STATUS_REG) & 0x08))
  //   ; // Wait for a first set of data

  volatile uint16_t tmpx, tmpy, tmpz;

  I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, acc_output); // Read data output registers 0x01-0x06

  tmpx = ((short)(acc_output[0] << 8 | acc_output[1])) >> 2; // Compute 14-bit X-axis output value
  tmpy = ((short)(acc_output[2] << 8 | acc_output[3])) >> 2; // Compute 14-bit Y-axis output value
  tmpz = ((short)(acc_output[4] << 8 | acc_output[5])) >> 2; // Compute 14-bit Z-axis output value

  Xout_14_bit = acc_control(tmpx, Xout_14_bit);
  Yout_14_bit = acc_control(tmpy, Yout_14_bit);
  Zout_14_bit = acc_control(tmpz, Zout_14_bit);

  Xoffset = Xout_14_bit / 8 * (-1);                    // Compute X-axis offset correction value
  Yoffset = Yout_14_bit / 8 * (-1);                    // Compute Y-axis offset correction value
  Zoffset = (Zout_14_bit - SENSITIVITY_2G) / 8 * (-1); // Compute Z-axis offset correction value

  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x00); // Standby mode to allow writing to the offset registers
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_X_REG, Xoffset);
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Y_REG, Yoffset);
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Z_REG, Zoffset);
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG3, 0x00); // Push-pull, active low interrupt
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x01); // Enable DRDY interrupt
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x01); // DRDY interrupt routed to INT1 - PTA14
  I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x3D); // ODR = 1.56Hz, Reduced noise, Active mode
}

/******************************************************************************
 * RUTINAS INTERRUPCIÓN
 ******************************************************************************/
void PORTC_PORTD_IRQHandler(void)
{
  /* Interrupt on SW1 detected */
  if (PORTC->PCR[3] & PORT_PCR_ISF_MASK)
  {
    // Para evitar que se execute constantemente a interrupción.
    PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    button = 1;
  }
  else if (PORTC->PCR[12] & PORT_PCR_ISF_MASK) /* Interrupt on SW3 detected */
  {
    // Para evitar que se execute constantemente a interrupción.
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
    button = 2;
  }
  else /* Accelerometer interrupt */
  {
    PORTC->PCR[5] |= PORT_PCR_ISF_MASK; // Clear the interrupt flag
    acc_read();
  }
}

int main(void)
{
  irclk_ini();
  lcd_ini();
  leds_ini();
  buttons_ini();
  MCU_Init();
  Accelerometer_Init();
  // Calibrate();

  uint8_t display_control = 1;
  while (1)
  {
    if (button == 1)
    {

      display_control += 1;

      if (display_control > 3)
        display_control = 1;

      switch (display_control)
      {
      case 1: // X LCD (ningún led)
        led_red_toggle();
        break;

      case 2: // Y LCD (led verde)
        led_green_toggle();
        break;

      case 3: // Z LCD (led vermello)
        led_red_toggle();
        led_green_toggle();
        break;

      default:
        break;
      }

      button = 0;
    }

    if (display_control == 1)
      lcd_display_dec(to360(Xout_14_bit));
    if (display_control == 2)
      lcd_display_dec(to360(Yout_14_bit));
    if (display_control == 3)
      lcd_display_dec(to360(Zout_14_bit));

    millis(500);
  }

  return 0;
}
