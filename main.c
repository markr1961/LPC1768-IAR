/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2006 - 2009
 *
 *    File name   : main.c
 *    Description : Define main module
 *
 *    History :
 *    1. Date        : 30, November 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *    2. Date        : 9, February 2009
 *       Author      : Stoyan Choynev
 *       Description : Port for IAR-LPC-1768-SK
 *
 *  This example project shows how to use the IAR Embedded Workbench for ARM
 * to develop code for the IAR LPC-1768-SK board. It shows basic use of the I/O,
 * the timer and the interrupt controllers.
 *  It starts by blinking LED1.
 *
 * Note:
 *  After power-up the controller get clock from internal RC oscillator that
 * is unstable and may fail with J-Link auto detect. In this case adaptive clocking
 * should be used. The adaptive clock can be select from menu:
 *  Project->Options..., section Debugger->J-Link/J-Trace  JTAG Speed - Adaptive.
 *
 *    $Revision: 28 $
 **************************************************************************/
#include <intrinsics.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <nxp/iolpc1768.h>
#include <assert.h>

#include "board.h"

#include "LPC17xx.h"

#define TIMER0_TICK_PER_SEC   10
#define TIMER0_IN_FREQ        25000000

/*variable for critical section entry control*/
Int32U CriticalSecCntr;
/*************************************************************************
 * Function Name: NVIC_IntEnable
 * Parameters: IntNumber - Interrupt number
 * Return: void
 *
 * Description: Enable interrupt at NVIC
 *
 *
 *************************************************************************/
void NVIC_IntEnable(Int32U IntNumber)
{
volatile unsigned long * pNVIC_SetEn = &SETENA0;

  assert((NVIC_WDT <= IntNumber) && (NVIC_PLL1 >= IntNumber));
  IntNumber -= NVIC_WDT;

  pNVIC_SetEn += IntNumber/32;
  *pNVIC_SetEn = (1UL<<(IntNumber%32));

}

/*************************************************************************
 * Function Name: NVIC_IntDisable
 * Parameters: IntNumber - Interrupt number
 * Return: void
 *
 * Description: Disables interrupt at NVIC
 *
 *
 *************************************************************************/
void NVIC_IntDisable(Int32U IntNumber)
{
volatile unsigned long * pNVIC_ClrEn = &CLRENA0;

  assert((NVIC_WDT <= IntNumber) && (NVIC_PLL1 >= IntNumber));
  IntNumber -= NVIC_WDT;

  pNVIC_ClrEn += IntNumber/32;
  *pNVIC_ClrEn = (1UL<<(IntNumber%32));

}

/*************************************************************************
 * Function Name: NVIC_ClrPend
 * Parameters: IntNumber - Interrupt number
 * Return: void
 *
 * Description:Clear pending interrupt at NVIC
 *
 *
 *************************************************************************/
void NVIC_ClrPend(Int32U IntNumber)
{
volatile unsigned long * pNVIC_ClrPend = &CLRPEND0;

  assert((NVIC_WDT <= IntNumber) && (NVIC_PLL1 >= IntNumber));
  IntNumber -= NVIC_WDT;

  pNVIC_ClrPend += IntNumber/32;
  *pNVIC_ClrPend = (1UL<<(IntNumber%32));

}

/*************************************************************************
 * Function Name: NVIC_ClrPend
 * Parameters: IntNumber - Interrupt number, Interrupt Priority
 * Return: void
 *
 * Description:Sets Interrupt priority
 *
 *
 *************************************************************************/
void NVIC_IntPri(Int32U IntNumber, Int8U Priority)
{
volatile Int8U * pNVIC_IntPri = (Int8U *)&IP0;

  assert((NVIC_WDT <= IntNumber) && (NVIC_PLL1 >= IntNumber));
  IntNumber -= NVIC_WDT;
  pNVIC_IntPri += IntNumber;
  *pNVIC_IntPri = Priority;
}

/*************************************************************************
 * Function Name: TMR0_IRQHandler
 * Parameters: none
 *
 * Return: none
 *
 * Description: Timer 0 interrupt handler
 *
 *************************************************************************/
void TMR0_IRQHandler (void)
{
  /* Toggle LED1 */
  LED1_FIO ^= LED1_MASK;
  /*clear interrupt */
  T0IR_bit.MR0INT = 1;
  /**/
  NVIC_ClrPend(NVIC_TIMER0);
}

#define FCCLK_FREQ 100000000
/*************************************************************************
 * Function Name: InitClock
 * Parameters: void
 * Return: void
 *
  * Description: Initialize PLL and clocks' dividers. 
                Hclk - 300MHz
 *              Fckl = 100MHz
 *
 *************************************************************************/
void InitClock(void)
{
  // 1. Init OSC
  SCS_bit.OSCRANGE = 0;
  SCS_bit.OSCEN = 1;
  // 2.  Wait for OSC ready
  while(!SCS_bit.OSCSTAT);
  // 3. Disconnect PLL
  PLL0CON_bit.PLLC = 0;
  PLL0FEED = 0xAA;
  PLL0FEED = 0x55;
  // 4. Disable PLL
  PLL0CON_bit.PLLE = 0;
  PLL0FEED = 0xAA;
  PLL0FEED = 0x55;
  // 5. Select source clock for PLL
  CLKSRCSEL_bit.CLKSRC = 1;   // Selects the main oscillator as a PLL clock source.
  PCLKSEL0 = PCLKSEL1  = 0;   // other peripherals 100/4 = 25MHz
  // 6. Set PLL settings 300 MHz
  PLL0CFG_bit.MSEL = 25-1;
  PLL0CFG_bit.NSEL = 2-1;
  PLL0FEED = 0xAA;
  PLL0FEED = 0x55;
  // 7. Enable PLL
  PLL0CON_bit.PLLE = 1;
  PLL0FEED = 0xAA;
  PLL0FEED = 0x55;
  // 8. Wait for the PLL to achieve lock
  while(!PLL0STAT_bit.PLOCK);
  // 9. Set clk divider settings
  CCLKCFG   = 3-1;            // 1/3 Fpll
  // 10. Connect the PLL
  PLL0CON_bit.PLLC = 1;
  PLL0FEED = 0xAA;
  PLL0FEED = 0x55;
}

/*************************************************************************
 * Function Name: GpioInit
 * Parameters: void
 * Return: void
 *
 * Description: Reset all GPIO pins to default: primary function
 *
 *************************************************************************/
void GpioInit(void)
{
  // Set to inputs
  FIO0DIR = 0;
  FIO1DIR = 0;
  FIO2DIR = 0;
  FIO3DIR = 0;
  FIO4DIR = 0;

  // clear mask registers
  FIO0MASK = 0;
  FIO1MASK = 0;
  FIO2MASK = 0;
  FIO3MASK = 0;
  FIO4MASK = 0;

  // Reset all GPIO pins to default primary function
  PINSEL0 = 0;
  PINSEL1 = 0;
  PINSEL2 = 0;
  PINSEL3 = 0;
  PINSEL4 = 0;
  PINSEL7 = 0;
  PINSEL8 = 0;
  PINSEL9 = 0;
}

void systick_setup(uint32_t sys_freq) {
    SysTick->LOAD = (sys_freq / 1000U) - 1U; //1ms tick
    SysTick->VAL = 0x00; //explicitly set start value (undefined on reset)
    SysTick->CTRL |= (1U << SysTick_CTRL_TICKINT_Pos) | (1U << SysTick_CTRL_CLKSOURCE_Pos); //enable systick interrupt, source processor clock
    SCB->SHP[8] = 0U;   //set SysTick interrupt priority (default: 0, the highest)
    SysTick->CTRL |= (1U << SysTick_CTRL_ENABLE_Pos); //enable SysTick
}
    
/*************************************************************************
 * Function Name: main
 * Parameters: none
 *
 * Return: none
 *
 * Description: main
 *
 *************************************************************************/
int main(void)
{
  // Flash accelerator init
  FLASHCFG = (0x5UL<<12) | 0x3AUL;
  // Init clock
  InitClock();
#if FCCLK_FREQ < 20000000
  FLASHCFG = (0x0UL<<12) | 0x3AUL;
#elif FCCLK_FREQ < 40000000
  FLASHCFG = (0x1UL<<12) | 0x3AUL;
#elif FCCLK_FREQ < 60000000
  FLASHCFG = (0x2UL<<12) | 0x3AUL;
#elif FCCLK_FREQ < 80000000
  FLASHCFG = (0x3UL<<12) | 0x3AUL;
#elif FCCLK_FREQ < 100000000
  FLASHCFG = (0x4UL<<12) | 0x3AUL;
#endif
  // Init GPIO
  GpioInit();
  // Init LED1 Pin
  LED1_FDIR = LED1_MASK;
  LED1_FSET = LED1_MASK;

  // Enable TIM0 clocks
  PCONP_bit.PCTIM0 = 1; // enable clock

  // Init Time0
  T0TCR_bit.CE = 0;     // counting  disable
  T0TCR_bit.CR = 1;     // set reset
  T0TCR_bit.CR = 0;     // release reset
  T0CTCR_bit.CTM = 0;   // Timer Mode: every rising PCLK edge
  T0MCR_bit.MR0I = 1;   // Enable Interrupt on MR0
  T0MCR_bit.MR0R = 1;   // Enable reset on MR0
  T0MCR_bit.MR0S = 0;   // Disable stop on MR0
  // set timer 0 period
  T0PR = 18-1;
  T0MR0 = (TIMER0_IN_FREQ)/(18 * TIMER0_TICK_PER_SEC);
  // init timer 0 interrupt
  T0IR_bit.MR0INT = 1;  // clear pending interrupt
  T0TCR_bit.CE = 1;     // counting Enable
  /*Enable NVIC TMR0 Interrupt*/
  NVIC_IntEnable(NVIC_TIMER0);
  __enable_interrupt();
  
  systick_setup(100000000);
  /*Enable NVIC systick Interrupt*/
  NVIC_IntEnable(STI);
  
  while(1)
  {
    
  }
}

uint32_t u32sysTick;
void SysTick_Handler(void)
{
  u32sysTick++;
}
