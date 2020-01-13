/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "spi_transfer.h"
#include "isp_user.h"

#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HIRC
#define PLL_CLOCK       50000000

void TIMER0_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_XTAL;
    // Set timer frequency to 3HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 3);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    // Start Timer 3
    TIMER_Start(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    while (SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Enable HIRC */
    CLK->PWRCTL = (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_XTLEN_Msk);

    /* Waiting for clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | CLK_CLKDIV_HCLK(2);
    //CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    //CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_SPICKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPI0_SS | SYS_MFP_P05_SPI0_MOSI | SYS_MFP_P06_SPI0_MISO | SYS_MFP_P07_SPI0_CLK;

}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SPI_Init();
    GPIO_Init();
    TIMER0_Init();

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            goto _ISP;
        }

        if (TIMER0->INTSTS & TIMER_INTSTS_TIF_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    outpw(&SYS->RSTSTS, 3);//clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
