/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show hard fault information when hard fault happened.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#define USE_MY_HARDFAULT    1   /* Select to use default process hard fault handler or not. 0: Default  1: User define */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

int main(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk;

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 115200);
}


#if USE_MY_HARDFAULT
/**
  * @brief      User defined Process HardFault
  * @param      stack   A pointer to current stack
  * @return     None
  * @details    This function is an example to show how to implement user's process hard fault handler
  *
  */
uint32_t ProcessHardFault(uint32_t u32_lr, uint32_t u32msp, uint32_t u32psp)
{
    uint32_t u32exception_num;
    uint32_t u32r0, u32r1, u32r2, u32r3, u32r12, u32lr, u32pc, u32psr, *pu32sp;

    if (u32_lr & 4)
        pu32sp = (uint32_t *)u32psp;
    else
        pu32sp = (uint32_t *)u32msp;

    /* Get information from stack */
    u32r0  = pu32sp[0];
    u32r1  = pu32sp[1];
    u32r2  = pu32sp[2];
    u32r3  = pu32sp[3];
    u32r12 = pu32sp[4];
    u32lr  = pu32sp[5];
    u32pc  = pu32sp[6];
    u32psr = pu32sp[7];


    /* Check T bit of psr */
    if ((u32psr & (1 << 24)) == 0)
    {
        printf("PSR T bit is 0.\nHard fault caused by changing to ARM mode!\n");

        while (1);
    }

    /* Check hard fault caused by ISR */
    u32exception_num = u32psr & xPSR_ISR_Msk;

    if (u32exception_num > 0)
    {
        /*
        Exception number
            0 = Thread mode
            1 = Reserved
            2 = NMI
            3 = HardFault
            4-10 = Reserved11 = SVCall
            12, 13 = Reserved
            14 = PendSV
            15 = SysTick, if implemented[a]
            16 = IRQ0.
                .
                .
            n+15 = IRQ(n-1)[b]
            (n+16) to 63 = Reserved.
        The number of interrupts, n, is 32
        */

        printf("Hard fault is caused in IRQ #%u\n", u32exception_num - 16);

        while (1);
    }

    printf("Hard fault location is at 0x%08x\n", u32pc);
    /*
        If the hard fault location is a memory access instruction, You may debug the load/store issues.

        Memory access faults can be caused by:
            Invalid address - read/write wrong address
            Data alignment issue - Violate alignment rule of Cortex-M processor
            Memory access permission - MPU violations or unprivileged access (Cortex-M0+)
            Bus components or peripheral returned an error response.
    */

    printf("r0  = 0x%x\n", u32r0);
    printf("r1  = 0x%x\n", u32r1);
    printf("r2  = 0x%x\n", u32r2);
    printf("r3  = 0x%x\n", u32r3);
    printf("r12 = 0x%x\n", u32r12);
    printf("lr  = 0x%x\n", u32lr);
    printf("pc  = 0x%x\n", u32pc);
    printf("psr = 0x%x\n", u32psr);

    while (1);
}
#endif

extern int __initial_sp;
extern int __stack_base;
extern int __heap_base;
extern int __heap_limit;
const int s_magic = 0x43218765;
const int h_magic = 0x56781234;
int stack_set_guard(void)
{
__disable_irq();

int* msp = (int *)__get_MSP();
int* base = &__stack_base;
 
 if(msp < base) {
 __enable_irq();
 printf("the sp is to small\n\r");
 return -1;
 }
 
 
 for( ; base != msp; base++)
 *base = s_magic;
 
 __enable_irq();
 
 return (uint32_t)msp - (uint32_t)&__stack_base;
}

int stack_detect_guard(void)
{
__disable_irq();

int* msp = (int *)__get_MSP();
int* base = &__stack_base;

if(msp < base || *base != s_magic) {
__enable_irq();
printf("write over sp\n\r");
return -1;
}

for( ; base != msp; base++) {
if(*base != s_magic)
{
break;
}
}

__enable_irq();

return (uint32_t)base - (uint32_t)&__stack_base;
}

void test_stack(void)
{
unsigned int i=0;
unsigned char temp[512]={0};
for (i=0;i<512;i++)
temp[i]=i;
}
/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();
int ret = stack_set_guard();


ret = stack_detect_guard();
printf("stack_detect_guard %d\r\n", ret);	
	test_stack();
	ret = stack_detect_guard()-ret;
	printf("stack_detect_guard %d\r\n", ret);	
	while(1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
