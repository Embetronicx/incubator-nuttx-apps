/****************************************************************************
 * examples/etx_spi/etx_spi_ssd1306.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP

#define SPIIOC_TRANSFER         _SPIIOC(0x0001) // IOCTL Command

#define GPIOC_REGISTER          _GPIOC(1)       // IOCTL command to register
#define GPIOC_UNREGISTER        _GPIOC(2)       // IOCTL command to unregister

#define ETX_SPI_DRIVER_PATH     "/dev/spi2"     // SPI Driver path (SPI 2)
#define ETX_GPIO_DRIVER_PATH    "/dev/etx_gpio" // GPIO Driver path

#define SSD1306_RST_PIN  CONFIG_EXAMPLES_ETX_SPI_SSD1306_RST_PIN // Reset Pin
#define SSD1306_DC_PIN   CONFIG_EXAMPLES_ETX_SPI_SSD1306_DC_PIN  // data/cmd Pin 

#define SSD1306_MAX_SEG         ( 128 )         // Maximum segment
#define SSD1306_MAX_LINE        (   7 )         // Maximum line
#define SSD1306_DEF_FONT_SIZE   (   5 )         // Default font size

typedef enum
{
  ETX_GPIO_IN,        // GPIO as input
  ETX_GPIO_OUT,       // GPIO as ouput
  ETX_GPIO_IN_INT     // GPIO as input and enable the interrupt
} GPIO_TYPE;

typedef struct
{
  GPIO_TYPE gpio_type;    // GPIO type
  uint8_t   gpio_num;     // GPIO number
  uint8_t  *gpio_value;   // GPIO value
  void     *data;         // Data
}etx_gpio;

#endif //#ifdef CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP
