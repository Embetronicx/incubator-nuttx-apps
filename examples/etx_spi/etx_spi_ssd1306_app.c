/****************************************************************************
 * examples/etx_spi/etx_spi_ssd1306_app.c
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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/spi/spi_transfer.h>
#include "etx_spi_ssd1306.h"

#ifdef CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP


/****************************************************************************
 * Private Data
 ****************************************************************************/

static etx_gpio reset_pin;   // GPIO Pin Structure for Reset pin
static etx_gpio dc_pin;      // GPIO Pin Structure for DC pin

static int fd_gpio = -1;     // File descriptor for GPIO driver
static int fd_spi  = -1;     // File descriptor for SPI driver

/*
** Variable to store Line Number and Cursor Position.
*/ 
static uint8_t SSD1306_LineNum   = 0;
static uint8_t SSD1306_CursorPos = 0;
static uint8_t SSD1306_FontSize  = SSD1306_DEF_FONT_SIZE;

/*
**  EmbeTronicX Logo
*/

static const uint8_t etx_logo[1024] = {
  0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xE1, 0xF9, 0xFF, 0xF9, 0xE1, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF8, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0x3F, 0xFF,
  0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x0F, 0xF8, 0xF7, 0x00, 0xBF, 0xC0, 0x7F,
  0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x3F, 0xE0, 0x0F, 0x7F, 0x00, 0xFF, 0x7F, 0x80,
  0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x04, 0xFC, 0xFC, 0x0C, 0x0C, 0x0C, 0x0C, 0x7C, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0xFC, 0xF8,
  0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x08,
  0x04, 0x04, 0xFC, 0xFC, 0x04, 0x04, 0x04, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x98, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80,
  0x00, 0x00, 0x04, 0x0C, 0x38, 0xE0, 0x80, 0xE0, 0x38, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x3F, 0xFF, 0xFF, 0x00, 0xFD, 0xFE, 0xFF,
  0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0x06, 0x06, 0x06, 0xE0, 0x00, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
  0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x7C, 0xFF, 0x11, 0x10, 0x1F, 0x1F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7C, 0xFF, 0x01, 0x00, 0x01, 0xFF, 0x7C, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x87,
  0x00, 0x00, 0x00, 0x00, 0xE0, 0x3F, 0x1F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0xFC, 0xF7, 0x00, 0xDF, 0xE3, 0x7D,
  0x3E, 0x07, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
  0x02, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x03, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x02, 0x02, 0x03,
  0x00, 0x00, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0xFF, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x88, 0x88, 0x08, 0x00, 0xE0, 0x20, 0x20, 0xE0, 0x20, 0xE0,
  0x00, 0xFC, 0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0xFC, 0x00,
  0xE0, 0x20, 0x20, 0xFC, 0x00, 0xE0, 0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0xFC, 0x00, 0x00,
  0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xFC, 0x20, 0x20, 0x00, 0xE0,
  0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0x00, 0xEC, 0x00, 0x20, 0x20, 0x20, 0xE0, 0x00, 0xFC,
  0x00, 0xE0, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0xC8, 0x38, 0x00, 0x00, 0xE0,
  0x20, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0xE0, 0x00, 0xE0, 0x20, 0x20, 0xE0, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x9C, 0x9C, 0x88, 0x8E, 0x83, 0x98, 0x83, 0x8E, 0x88,
  0x88, 0x9C, 0x9C, 0x80, 0x80, 0x87, 0x84, 0x84, 0x84, 0x80, 0x87, 0x80, 0x80, 0x87, 0x80, 0x87,
  0x80, 0x87, 0x84, 0x84, 0x87, 0x80, 0x87, 0x85, 0x85, 0x85, 0x80, 0x87, 0x84, 0x84, 0x87, 0x80,
  0x87, 0x84, 0x84, 0x87, 0x80, 0x87, 0x85, 0x85, 0x85, 0x80, 0x87, 0x84, 0x84, 0x87, 0x80, 0x80,
  0x80, 0x80, 0x80, 0x87, 0x80, 0x80, 0x80, 0x87, 0x84, 0x87, 0x80, 0x87, 0x84, 0x84, 0x80, 0x87,
  0x84, 0x84, 0x87, 0x80, 0x87, 0x80, 0x80, 0x80, 0x87, 0x80, 0x87, 0x85, 0x85, 0x87, 0x80, 0x87,
  0x80, 0x85, 0x85, 0x85, 0x87, 0x80, 0x80, 0x80, 0x80, 0x86, 0x85, 0x84, 0x84, 0x84, 0x80, 0x87,
  0x84, 0x84, 0x87, 0x80, 0x87, 0x80, 0x87, 0x80, 0x87, 0x85, 0x85, 0x85, 0x80, 0x80, 0x80, 0xFF
};

/*
** Array Variable to store the letters.
*/ 
static const unsigned char SSD1306_font[][SSD1306_DEF_FONT_SIZE]= 
{
    {0x00, 0x00, 0x00, 0x00, 0x00},   // space
    {0x00, 0x00, 0x2f, 0x00, 0x00},   // !
    {0x00, 0x07, 0x00, 0x07, 0x00},   // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14},   // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12},   // $
    {0x23, 0x13, 0x08, 0x64, 0x62},   // %
    {0x36, 0x49, 0x55, 0x22, 0x50},   // &
    {0x00, 0x05, 0x03, 0x00, 0x00},   // '
    {0x00, 0x1c, 0x22, 0x41, 0x00},   // (
    {0x00, 0x41, 0x22, 0x1c, 0x00},   // )
    {0x14, 0x08, 0x3E, 0x08, 0x14},   // *
    {0x08, 0x08, 0x3E, 0x08, 0x08},   // +
    {0x00, 0x00, 0xA0, 0x60, 0x00},   // ,
    {0x08, 0x08, 0x08, 0x08, 0x08},   // -
    {0x00, 0x60, 0x60, 0x00, 0x00},   // .
    {0x20, 0x10, 0x08, 0x04, 0x02},   // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E},   // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},   // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},   // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},   // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},   // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},   // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},   // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},   // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},   // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},   // 9
    {0x00, 0x36, 0x36, 0x00, 0x00},   // :
    {0x00, 0x56, 0x36, 0x00, 0x00},   // ;
    {0x08, 0x14, 0x22, 0x41, 0x00},   // <
    {0x14, 0x14, 0x14, 0x14, 0x14},   // =
    {0x00, 0x41, 0x22, 0x14, 0x08},   // >
    {0x02, 0x01, 0x51, 0x09, 0x06},   // ?
    {0x32, 0x49, 0x59, 0x51, 0x3E},   // @
    {0x7C, 0x12, 0x11, 0x12, 0x7C},   // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},   // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},   // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},   // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},   // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},   // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A},   // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},   // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},   // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},   // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},   // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},   // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},   // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},   // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},   // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},   // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},   // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},   // R
    {0x46, 0x49, 0x49, 0x49, 0x31},   // S
    {0x01, 0x01, 0x7F, 0x01, 0x01},   // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},   // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},   // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F},   // W
    {0x63, 0x14, 0x08, 0x14, 0x63},   // X
    {0x07, 0x08, 0x70, 0x08, 0x07},   // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},   // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00},   // [
    {0x55, 0xAA, 0x55, 0xAA, 0x55},   // Backslash (Checker pattern)
    {0x00, 0x41, 0x41, 0x7F, 0x00},   // ]
    {0x04, 0x02, 0x01, 0x02, 0x04},   // ^
    {0x40, 0x40, 0x40, 0x40, 0x40},   // _
    {0x00, 0x03, 0x05, 0x00, 0x00},   // `
    {0x20, 0x54, 0x54, 0x54, 0x78},   // a
    {0x7F, 0x48, 0x44, 0x44, 0x38},   // b
    {0x38, 0x44, 0x44, 0x44, 0x20},   // c
    {0x38, 0x44, 0x44, 0x48, 0x7F},   // d
    {0x38, 0x54, 0x54, 0x54, 0x18},   // e
    {0x08, 0x7E, 0x09, 0x01, 0x02},   // f
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C},   // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},   // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},   // i
    {0x40, 0x80, 0x84, 0x7D, 0x00},   // j
    {0x7F, 0x10, 0x28, 0x44, 0x00},   // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},   // l
    {0x7C, 0x04, 0x18, 0x04, 0x78},   // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},   // n
    {0x38, 0x44, 0x44, 0x44, 0x38},   // o
    {0xFC, 0x24, 0x24, 0x24, 0x18},   // p
    {0x18, 0x24, 0x24, 0x18, 0xFC},   // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},   // r
    {0x48, 0x54, 0x54, 0x54, 0x20},   // s
    {0x04, 0x3F, 0x44, 0x40, 0x20},   // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},   // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},   // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},   // w
    {0x44, 0x28, 0x10, 0x28, 0x44},   // x
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C},   // y
    {0x44, 0x64, 0x54, 0x4C, 0x44},   // z
    {0x00, 0x10, 0x7C, 0x82, 0x00},   // {
    {0x00, 0x00, 0xFF, 0x00, 0x00},   // |
    {0x00, 0x82, 0x7C, 0x10, 0x00},   // }
    {0x00, 0x06, 0x09, 0x09, 0x06}    // ~ (Degrees)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
/****************************************************************************
 * Name: etx_spi_ssd1306_ResetDcInit
 *
 * Details : This function Initializes and Configures the Reset and DC Pin
 ****************************************************************************/
static int etx_spi_ssd1306_ResetDcInit( void )
{
  int     ret      = -1;
  uint8_t gpio_val = 1u;      //Initial value of the GPIO
  
  if( fd_gpio >= 0 )
  {
    /* Register the Reset GPIO */
    reset_pin.gpio_type  = ETX_GPIO_OUT;
    reset_pin.gpio_num   = SSD1306_RST_PIN;
    reset_pin.gpio_value = &gpio_val;
    reset_pin.data       = NULL;

    ret = ioctl(fd_gpio, GPIOC_REGISTER, (unsigned long)((uintptr_t)&reset_pin));
    if (ret < 0)
    {
      int errcode = errno;
      printf("ERROR: GPIOC_REGISTER ioctl failed: RST - %d\n", errcode);
    }

    /* Register the DC GPIO */
    dc_pin.gpio_type  = ETX_GPIO_OUT;
    dc_pin.gpio_num   = SSD1306_DC_PIN;
    dc_pin.gpio_value = &gpio_val;
    dc_pin.data       = NULL;
    
    if( ret >= 0 )
    {
      ret = ioctl(fd_gpio, GPIOC_REGISTER, (unsigned long)((uintptr_t)&dc_pin));
      if (ret < 0)
      {
        int errcode = errno;
        printf("ERROR: GPIOC_REGISTER ioctl failed: DC - %d\n", errcode);
      }
    }
  }

  return( ret );
}

/****************************************************************************
 * Name: etx_spi_ssd1306_setRst
 *
 * Details : This function writes the value to the Reset GPIO
 ****************************************************************************/
static int etx_spi_ssd1306_setRst( uint8_t value )
{
  int ret = -1;
  
  if( fd_gpio >= 0 )
  {
    reset_pin.gpio_value = &value;
    
    //write the Reset GPIO value to the driver
    ret = write( fd_gpio, (const void*)&reset_pin, sizeof(reset_pin) );
  }
  return( ret );
}

/****************************************************************************
 * Name: etx_spi_ssd1306_setDc
 *
 * Details : This function writes the value to the DC GPIO
 ****************************************************************************/
static int etx_spi_ssd1306_setDc( uint8_t value )
{
  int ret = -1;

  if( fd_gpio >= 0 )
  {
    dc_pin.gpio_value = &value;
    
    //write the DC GPIO value to the driver
    ret = write( fd_gpio, (const void*)&dc_pin, sizeof(dc_pin) );
  }
  
  return( ret );
}

/****************************************************************************
 * Name: etx_spi_ssd1306_write
 *
 * Details : This function sends the command/data to the Display
 *
 * Argument: is_cmd
 *              true  - if we need to send command
 *              false - if we need to send data
 *           value
 *              value to be transmitted
 ****************************************************************************/
static int etx_spi_ssd1306_write( bool is_cmd, uint8_t value )
{
  int     ret = 0;
  uint8_t pin_value;
  
  if( fd_spi < 0 )
  {
    ret = -1;
  }

  if( ret >= 0 )
  {
    if( is_cmd )
    {
      //DC pin has to be high, if this is command.
      pin_value = 0u;
    }
    else
    {
      //DC pin has to be low, if this is data.
      pin_value = 1u;
    }
    
    ret = etx_spi_ssd1306_setDc( pin_value );
  }

  //send the bytes
  if( ret >= 0 )
  {
    struct  spi_sequence_s seq;
    struct  spi_trans_s trans;
    uint8_t rx_buf[4] = { 0 };
    uint8_t tx_buf[4] = { 0 };
    
    tx_buf[0] = value;
    
    seq.dev       = SPIDEV_ID( SPIDEVTYPE_USER, 0u );
    seq.mode      = SPIDEV_MODE0;                 // See enum spi_mode_e
    seq.nbits     = 8;                            // Number of bits
    seq.frequency = 4000000;                      // SPI frequency (Hz)
    seq.ntrans    = 1;                            // Number of transactions
    seq.trans     = &trans;
    
    trans.deselect = true;                        // De-select after transfer
    trans.delay    = 0;                           // Microsecond delay after tx
    trans.nwords   = 1;                           // Number of words in transfer
    trans.txbuffer = tx_buf;                      // Tx buffer
    trans.rxbuffer = rx_buf;                      // Rx buffer
    
    // Transfer the data
    ret = ioctl( fd_spi, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq) );
  }
  
  return( ret );
}

/*
** This function is specific to the SSD_1306 OLED.
**
**  Arguments:
**      lineNo    -> Line Number
**      cursorPos -> Cursor Position
**   
*/
static void etx_spi_ssd1306_SetCursor( uint8_t lineNo, uint8_t cursorPos )
{
  /* Move the Cursor to specified position only if it is in range */
  if((lineNo <= SSD1306_MAX_LINE) && (cursorPos < SSD1306_MAX_SEG))
  {
    SSD1306_LineNum   = lineNo;                     // Save the specified line number
    SSD1306_CursorPos = cursorPos;                  // Save the specified cursor position
    etx_spi_ssd1306_write(true, 0x21);              // cmd for the column start and end address
    etx_spi_ssd1306_write(true, cursorPos);         // column start addr
    etx_spi_ssd1306_write(true, SSD1306_MAX_SEG-1); // column end addr
    etx_spi_ssd1306_write(true, 0x22);              // cmd for the page start and end address
    etx_spi_ssd1306_write(true, lineNo);            // page start addr
    etx_spi_ssd1306_write(true, SSD1306_MAX_LINE);  // page end addr
  }
}

/*
** This function is specific to the SSD_1306 OLED.
** This function move the cursor to the next line.
**
**  Arguments:
**      none
** 
*/
static void etx_spi_ssd1306_GoToNextLine( void )
{
  /*
  ** Increment the current line number.
  ** roll it back to first line, if it exceeds the limit. 
  */
  SSD1306_LineNum++;
  SSD1306_LineNum = (SSD1306_LineNum & SSD1306_MAX_LINE);
  etx_spi_ssd1306_SetCursor(SSD1306_LineNum,0); /* Finally move it to next line */
}

/*
** This function is specific to the SSD_1306 OLED.
** This function sends the single char to the OLED.
**
**  Arguments:
**      c   -> character to be written
** 
*/
static void etx_spi_ssd1306_PrintChar(unsigned char c)
{
  uint8_t data_byte;
  uint8_t temp = 0;
  /*
  ** If we character is greater than segment len or we got new line charcter
  ** then move the cursor to the new line
  */ 
  if( (( SSD1306_CursorPos + SSD1306_FontSize ) >= SSD1306_MAX_SEG ) ||
      ( c == '\n' )
  )
  {
    etx_spi_ssd1306_GoToNextLine();
  }
  // print charcters other than new line
  if( c != '\n' )
  {
  
    /*
    ** In our font array (SSD1306_font), space starts in 0th index.
    ** But in ASCII table, Space starts from 32 (0x20).
    ** So we need to match the ASCII table with our font table.
    ** We can subtract 32 (0x20) in order to match with our font table.
    */
    c -= 0x20;  //or c -= ' ';
    do
    {
      data_byte= SSD1306_font[c][temp];         // Get the data to be displayed from LookUptable
      etx_spi_ssd1306_write(false, data_byte);  // write data to the OLED
      SSD1306_CursorPos++;
      
      temp++;
      
    } while ( temp < SSD1306_FontSize);
    etx_spi_ssd1306_write(false, 0x00);         //Display the data
    SSD1306_CursorPos++;
  }
}

/*
** This function is specific to the SSD_1306 OLED.
** This function sends the string to the OLED.
**
**  Arguments:
**      str   -> string to be written
** 
*/
static void etx_spi_ssd1306_String(char *str)
{
  while(*str)
  {
    etx_spi_ssd1306_PrintChar(*str++);
  }
}

/*
** This function is specific to the SSD_1306 OLED.
** This function inverts the display.
**
**  Arguments:
**      need_to_invert   -> true  - invert display
**                          false - normal display        
** 
*/
static void etx_spi_ssd1306_InvertDisplay(bool need_to_invert)
{
  if(need_to_invert)
  {
    etx_spi_ssd1306_write(true, 0xA7); // Invert the display
  }
  else
  {
    etx_spi_ssd1306_write(true, 0xA6); // Normal display
  }
}

/*
** This function is specific to the SSD_1306 OLED.
** This function sets the brightness of  the display.
**
**  Arguments:
**      brightnessValue   -> brightness value
** 
*/
static void etx_spi_ssd1306_SetBrightness(uint8_t brightnessValue)
{
    etx_spi_ssd1306_write(true, 0x81);            // Contrast command
    etx_spi_ssd1306_write(true, brightnessValue); // Contrast value (default value = 0x7F)
}

/*
** This function is specific to the SSD_1306 OLED.
** This function Scrolls the data right/left in horizontally.
**
**  Arguments:
**      is_left_scroll   -> true  - left horizontal scroll
                            false - right horizontal scroll
        start_line_no    -> Start address of the line to scroll 
        end_line_no      -> End address of the line to scroll                 
** 
*/
static void etx_spi_ssd1306_StartScrollHorizontal( bool is_left_scroll,
                                                   uint8_t start_line_no,
                                                   uint8_t end_line_no
                                                 )
{
  if(is_left_scroll)
  {
    // left horizontal scroll
    etx_spi_ssd1306_write(true, 0x27);
  }
  else
  {
    // right horizontal scroll 
    etx_spi_ssd1306_write(true, 0x26);
  }
  
  etx_spi_ssd1306_write(true, 0x00);            // Dummy byte (dont change)
  etx_spi_ssd1306_write(true, start_line_no);   // Start page address
  etx_spi_ssd1306_write(true, 0x00);            // 5 frames interval
  etx_spi_ssd1306_write(true, end_line_no);     // End page address
  etx_spi_ssd1306_write(true, 0x00);            // Dummy byte (dont change)
  etx_spi_ssd1306_write(true, 0xFF);            // Dummy byte (dont change)
  etx_spi_ssd1306_write(true, 0x2F);            // activate scroll
}

/*
** This function is specific to the SSD_1306 OLED.
** This function Scrolls the data in vertically and right/left horizontally
** (Diagonally).
**
**  Arguments:
**      is_vertical_left_scroll -> true  - vertical and left horizontal scroll
**                                 false - vertical and right horizontal scroll
**      start_line_no           -> Start address of the line to scroll 
**      end_line_no             -> End address of the line to scroll 
**      vertical_area           -> Area for vertical scroll (0-63)
**      rows                    -> Number of rows to scroll vertically             
** 
*/
static void etx_spi_ssd1306_StartScrollVerticalHorizontal( 
                                                   bool is_vertical_left_scroll,
                                                   uint8_t start_line_no,
                                                   uint8_t end_line_no,
                                                   uint8_t vertical_area,
                                                   uint8_t rows
                                                 )
{
  
  etx_spi_ssd1306_write(true, 0xA3);            // Set Vertical Scroll Area
  etx_spi_ssd1306_write(true, 0x00);            // Check datasheet
  etx_spi_ssd1306_write(true, vertical_area);   // area for vertical scroll
  
  if(is_vertical_left_scroll)
  {
    // vertical and left horizontal scroll
    etx_spi_ssd1306_write(true, 0x2A);
  }
  else
  {
    // vertical and right horizontal scroll 
    etx_spi_ssd1306_write(true, 0x29);
  }
  
  etx_spi_ssd1306_write(true, 0x00);            // Dummy byte (dont change)
  etx_spi_ssd1306_write(true, start_line_no);   // Start page address
  etx_spi_ssd1306_write(true, 0x00);            // 5 frames interval
  etx_spi_ssd1306_write(true, end_line_no);     // End page address
  etx_spi_ssd1306_write(true, rows);            // Vertical scrolling offset
  etx_spi_ssd1306_write(true, 0x2F);            // activate scroll
}

/*
** This function is specific to the SSD_1306 OLED.
** This function disables the scroll.
**
**  Arguments:
**      none.       
** 
*/
static void etx_spi_ssd1306_DeactivateScroll( void )
{
  etx_spi_ssd1306_write(true, 0x2E); // Deactivate scroll
}

/****************************************************************************
 * Name: etx_spi_ssd1306_fill
 *
 * Details : This function fills the data to the Display
 ****************************************************************************/
static void etx_spi_ssd1306_fill( uint8_t data )
{
  // 8 pages x 128 segments x 8 bits of data
  unsigned int total  = ( SSD1306_MAX_SEG * (SSD1306_MAX_LINE + 1) );
  unsigned int i      = 0;
  
  //Fill the Display
  for(i = 0; i < total; i++)
  {
    etx_spi_ssd1306_write(false, data);
  }
}

/****************************************************************************
 * Name: etx_spi_ssd1306_ClearDisplay
 *
 * Details : This function clears the Display
 ****************************************************************************/
static void etx_spi_ssd1306_ClearDisplay( void )
{
  //Set cursor
  etx_spi_ssd1306_SetCursor(0,0);
  
  etx_spi_ssd1306_fill( 0x00 );
}

/****************************************************************************
 * Name: etx_spi_ssd1306_DisplayInit
 *
 * Details : This function Initializes the Display
 ****************************************************************************/
static int etx_spi_ssd1306_DisplayInit(void)
{
  int ret = 0;

  //Make the RESET Line to 0
  ret = etx_spi_ssd1306_setRst( 0u );
  
  if( ret >= 0 )
  {
    //usleep(100000);                          // delay
    //Make the RESET Line to 1
    ret = etx_spi_ssd1306_setRst( 1u );
    //usleep(100000);                          // delay
  }

  /*
  ** Commands to initialize the SSD_1306 OLED Display
  */
  etx_spi_ssd1306_write(true, 0xAE); // Entire Display OFF
  etx_spi_ssd1306_write(true, 0xD5); // Set Display Clock Divide Ratio and Oscillator Frequency
  etx_spi_ssd1306_write(true, 0x80); // Default Setting for Display Clock Divide Ratio and Oscillator Frequency that is recommended
  etx_spi_ssd1306_write(true, 0xA8); // Set Multiplex Ratio
  etx_spi_ssd1306_write(true, 0x3F); // 64 COM lines
  etx_spi_ssd1306_write(true, 0xD3); // Set display offset
  etx_spi_ssd1306_write(true, 0x00); // 0 offset
  etx_spi_ssd1306_write(true, 0x40); // Set first line as the start line of the display
  etx_spi_ssd1306_write(true, 0x8D); // Charge pump
  etx_spi_ssd1306_write(true, 0x14); // Enable charge dump during display on
  etx_spi_ssd1306_write(true, 0x20); // Set memory addressing mode
  etx_spi_ssd1306_write(true, 0x00); // Horizontal addressing mode
  etx_spi_ssd1306_write(true, 0xA1); // Set segment remap with column address 127 mapped to segment 0
  etx_spi_ssd1306_write(true, 0xC8); // Set com output scan direction, scan from com63 to com 0
  etx_spi_ssd1306_write(true, 0xDA); // Set com pins hardware configuration
  etx_spi_ssd1306_write(true, 0x12); // Alternative com pin configuration, disable com left/right remap
  etx_spi_ssd1306_write(true, 0x81); // Set contrast control
  etx_spi_ssd1306_write(true, 0x80); // Set Contrast to 128
  etx_spi_ssd1306_write(true, 0xD9); // Set pre-charge period
  etx_spi_ssd1306_write(true, 0xF1); // Phase 1 period of 15 DCLK, Phase 2 period of 1 DCLK
  etx_spi_ssd1306_write(true, 0xDB); // Set Vcomh deselect level
  etx_spi_ssd1306_write(true, 0x20); // Vcomh deselect level ~ 0.77 Vcc
  etx_spi_ssd1306_write(true, 0xA4); // Entire display ON, resume to RAM content display
  etx_spi_ssd1306_write(true, 0xA6); // Set Display in Normal Mode, 1 = ON, 0 = OFF
  etx_spi_ssd1306_write(true, 0x2E); // Deactivate scroll
  etx_spi_ssd1306_write(true, 0xAF); // Display ON in normal mode

  // Clear the display
  etx_spi_ssd1306_ClearDisplay();
  
  printf("ETX_SPI_SSD1306: OLED Iitialized\r\n");
  
  return( ret );
}


/****************************************************************************
 * Name: etx_spi_ssd1306_task
 ****************************************************************************/

static int etx_spi_ssd1306_task(int argc, char *argv[])
{
  int ret = 0;
  
  printf("ETX_SPI_SSD1306: Starting the Task\n");
 
  /* Initialize and configure the GPIOs ( Reset and DC pin) */
  
  // Open the GPIO driver
  fd_gpio = open( ETX_GPIO_DRIVER_PATH, O_WRONLY);
  if( fd_gpio < 0 )
  {
    printf("ERROR - Failed to open %s: %d\n", ETX_GPIO_DRIVER_PATH, errno);
    ret = -1;
  }
 
  // Configure the GPIOs
  if( ret >= 0 )
  {
    ret = etx_spi_ssd1306_ResetDcInit();
  }
 
  /* Initialize and configure the SPI*/
  
  if( ret >= 0 )
  {
    // Open the SPI driver
    fd_spi = open( ETX_SPI_DRIVER_PATH, O_WRONLY);
    if( fd_spi < 0 )
    {
      printf("ERROR - Failed to open %s: %d\n", ETX_SPI_DRIVER_PATH, errno);
      ret = -1;
    }

  }
  
  if( ret >= 0 )
  {
    // Initialize the SSD1306 OLED
    ret = etx_spi_ssd1306_DisplayInit();
  }

  /* Print the String */
  //Set cursor
  etx_spi_ssd1306_SetCursor(0,0);
  
  etx_spi_ssd1306_SetBrightness( 255 );           // Full brightness
  etx_spi_ssd1306_InvertDisplay( false );         // Invert the dispaly : OFF
  
  // Enable the Horizontal scroll for first 3 lines
  etx_spi_ssd1306_StartScrollHorizontal( true, 0, 2);
  
  //Write String to OLED
  etx_spi_ssd1306_String("Welcome\nTo\nEmbeTronicX\n\n");

  usleep(9000000);
  etx_spi_ssd1306_ClearDisplay();                 // Clear Display
  etx_spi_ssd1306_DeactivateScroll();
  
  // infinite while which does nothing
  while( ret >= 0 )
  { 
    /* Print the Image */
 
    //Set cursor
    etx_spi_ssd1306_SetCursor(0,0);
    etx_spi_ssd1306_InvertDisplay( true );         // Invert the dispaly : ON
    for(int i = 0; i < ( SSD1306_MAX_SEG * (SSD1306_MAX_LINE + 1) ); i++ )
    {
      ret = etx_spi_ssd1306_write(false, etx_logo[i]);
    }
    usleep( 500000 );
    etx_spi_ssd1306_InvertDisplay( false );         // Invert the dispaly : OFF
    usleep( 500000 );
  }
  
  /* Unregister the GPIOs */
  ret = ioctl(fd_gpio, GPIOC_UNREGISTER, (unsigned long)((uintptr_t)&reset_pin));
  if (ret < 0)
  {
    int errcode = errno;
    printf("ERROR: GPIOC_UNREGISTER ioctl failed: RST - %d\n", errcode);
  }
  
  ret = ioctl(fd_gpio, GPIOC_UNREGISTER, (unsigned long)((uintptr_t)&dc_pin));
  if (ret < 0)
  {
    int errcode = errno;
    printf("ERROR: GPIOC_UNREGISTER ioctl failed: DC - %d\n", errcode);
  }

  //Close the drivers
  close(fd_gpio);
  close(fd_spi);
  
  printf("ETX_SPI_SSD1306: ERROR - Task finishing ret = %d\n", ret);
p,,,,,
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  
  printf("ETX_SPI_SSD1306: Starting the Application\n");

  
  ret = task_create( "ETX_SPI_SSD1306",                       	   // Task Name
                     CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP_PRIORITY, // Task priority
                     CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP_STACKSIZE,// Task Stack size
                     etx_spi_ssd1306_task,                    	   // Task function
                     NULL
                   );
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_SPI_SSD1306: ERROR: Failed to start etx_spi_ssd1306_task: %d\n",
                                                                       errcode);
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}

#endif //#ifdef CONFIG_EXAMPLES_ETX_SPI_SSD1306_APP
