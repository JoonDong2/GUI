/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// HAL_MSP-EXP430FR4133_Sharp96x96.c
//
//*****************************************************************************
//
//! \addtogroup display_api
//! @{
//
//*****************************************************************************

#include "grlib.h"
#include "stm32f0xx_hal.h"
#include "HAL_MSP_GRLIB_STM32F030x8_Sharp96x96.h"
#include "gpio.h"
#include "spi.h"

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the Sharp96x96 display. This function
//! configures the GPIO pins used to control the LCD display when the basic
//! GPIO interface is in use. On exit, the LCD has been reset and is ready to
//! receive command and data writes.
//!
//! \return None.
//
//*****************************************************************************

#define LCD_PWR_PIN_SET(pinState) HAL_GPIO_WritePin(GPIOC, LCD_POWER_Pin, pinState)
#define LCD_DISP_PIN_SET(pinState) HAL_GPIO_WritePin(GPIOC, LCD_DISP_Pin, pinState)

//*****************************************************************************
//
// Writes command or data to the LCD Driver
//
// \param ucCmdData is the 8 or 16 bit command to send to the LCD driver
// Uses the SET_LCD_DATA macro
//
// \return None
//
//*****************************************************************************
void HAL_LCD_writeCommandOrData(uint8_t command)
{
	HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY);
}

/******************************************************************************
*
* Clears CS line
*
* This macro allows to clear the Chip Select (CS) line
*
* \return None
*
******************************************************************************/
void HAL_LCD_clearCS(void){
	HAL_GPIO_WritePin(GPIOB, LCD_SCS_Pin, GPIO_PIN_RESET);
}

/*****************************************************************************
*
* Set CS line
*
* This macro allows to set the Chip Select (CS) line
*
* \return None
*
******************************************************************************/
void HAL_LCD_setCS(void){
	HAL_GPIO_WritePin(GPIOB, LCD_SCS_Pin, GPIO_PIN_SET);
}

/******************************************************************************
*
* Disables Shapr96x96 LCD
*
* \param None
*
* \return None
******************************************************************************/
void HAL_LCD_disableDisplay(void)
{
	LCD_PWR_PIN_SET(GPIO_PIN_RESET);
	LCD_DISP_PIN_SET(GPIO_PIN_RESET);
}

//*****************************************************************************
//
// Enables Shapr96x96 LCD
//
// \param None
//
// \return None
//*****************************************************************************
void HAL_LCD_enableDisplay(void)
{
	LCD_PWR_PIN_SET(GPIO_PIN_SET);
	LCD_DISP_PIN_SET(GPIO_PIN_SET);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
