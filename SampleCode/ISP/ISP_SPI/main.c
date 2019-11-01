/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ISP_USER.h"
#include "targetdev.h"

#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HIRC
#define PLL_CLOCK       50000000

#define TEST_COUNT 16

uint32_t u32DataCount;
uint32_t *_response_buff;
uint32_t spi_rcvbuf[TEST_COUNT];

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

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 22MHz */
    SPI_Open(SPI, SPI_SLAVE, SPI_MODE_0, 32, 22000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
}

int main(void)
{
    SYS_Init();
    SPI_Init();
//    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
//    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk);
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

_ISP:
    u32DataCount = 0;

    SysTick->CTRL = 0UL;

    /* Check data count */
    while(u32DataCount < TEST_COUNT)
    {
        /* Write to TX register */
        SPI_WRITE_TX(SPI, _response_buff[u32DataCount]);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(SPI);
        /* Check SPI1 busy status */
        while(SPI_IS_BUSY(SPI))
        {
            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                goto _ISP;
            }
        }

        /* Read RX register */
        spi_rcvbuf[u32DataCount] = SPI_READ_RX(SPI);
        u32DataCount++;

        SysTick->LOAD = 1000 * CyclesPerUs;
        SysTick->VAL  = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    if((u32DataCount == TEST_COUNT) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
    {
        spi_rcvbuf[0] &= 0x000000FF;
        ParseCmd((unsigned char *)spi_rcvbuf, 64);
    }

    goto _ISP;

}
