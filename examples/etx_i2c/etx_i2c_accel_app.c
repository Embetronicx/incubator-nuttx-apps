/****************************************************************************
 * examples/etx_i2c/etx_i2c_accel_app.c
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
#include <nuttx/i2c/i2c_master.h>

#include "etx_i2c_accel_app.h"

#ifdef CONFIG_EXAMPLES_ETX_I2C_ACCEL


/****************************************************************************
 * Private Data
 ****************************************************************************/
static int fd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ETX_I2C_Write
 *
 * Details : This function writes the 1byte data to the slave device
 ****************************************************************************/
static int16_t ETX_I2C_Write( uint8_t reg, uint8_t value )
{
  int16_t ex;  
  struct  i2c_msg_s       i2c_msg;
  struct  i2c_transfer_s  i2c_transfer;  
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred.  Two bytes:  The ADXL345 register
   * address followed by one byte of data.
   */

  txbuffer[0] = reg;
  txbuffer[1] = value;

  /* Setup 8-bit ADXL345 address write message */

  i2c_msg.addr   = ADXL345_ADDRESS;
  i2c_msg.flags  = 0;
  i2c_msg.buffer = txbuffer;
  i2c_msg.length = 2;
  i2c_msg.frequency = 400000;  /* 400K bsp */
  
  i2c_transfer.msgv = &i2c_msg;
  i2c_transfer.msgc = 1;
  
  ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);
  
  if( ex < 0 )
  {
      printf("ETX_I2C:Write Error : %d\n", ex);
  }
  
  return( ex );
}

/****************************************************************************
 * Name: ETX_I2C_Read
 *
 * Details : This function reads the data from the slave device
 ****************************************************************************/
static int16_t ETX_I2C_Read( uint8_t reg, uint8_t* value, int16_t len )
{
  int16_t ex;
  struct  i2c_msg_s       i2c_msg[2];
  struct  i2c_transfer_s  i2c_transfer;
  
  // Write the register address first
  i2c_msg[0].addr   = ADXL345_ADDRESS;
  i2c_msg[0].flags  = 0;
  i2c_msg[0].buffer = &reg;
  i2c_msg[0].length = 1;
  i2c_msg[0].frequency = 400000;  /* 400K bsp */

  // Read the data
  i2c_msg[1].addr   = ADXL345_ADDRESS;
  i2c_msg[1].flags  = I2C_M_READ;
  i2c_msg[1].buffer = value;
  i2c_msg[1].length = len;
  i2c_msg[1].frequency = 400000;  /* 400K bsp */
  
  i2c_transfer.msgv = i2c_msg;
  i2c_transfer.msgc = 2;
  
  ex = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);
  
  if( ex < 0 )
  {
      printf("ETX_I2C:Read Error : %d\n", ex);
  }
  
  return( ex );
}

/****************************************************************************
 * Name: ETX_ADXL345_Init
 *
 * Details : This function Initializes the ADXL345
 ****************************************************************************/
static int16_t ETX_ADXL345_Init( void )
{
  int16_t ex = -1;
  uint8_t dev_id;
  
  // Reset all power settings
  ETX_I2C_Write( ADXL345_RA_POWER_CTL, 0);
  
  // Read the devId
  ETX_I2C_Read( ADXL345_RA_DEVID, &dev_id, 1);
  
  // Check the devId
  if( dev_id == 0xE5 )
  {
    printf("Device ID : 0x%X (Success!!!)\r\n", dev_id);
    
    //Set Output Rate to 100 Hz
    ETX_I2C_Write( ADXL345_RA_BW_RATE, 0x0A);
    
    // ADXL345 into measurement mode
    ETX_I2C_Write( ADXL345_RA_POWER_CTL, 0x08);
    
    // Disable Interrupts
    ETX_I2C_Write( ADXL345_RA_INT_ENABLE, 0x00 );
    
    ex = 0;
  }
  
  return( ex );
}

/****************************************************************************
 * Name: etx_i2c_accel_task
 ****************************************************************************/

static int etx_i2c_accel_task(int argc, char *argv[])
{
  int ret = 0;
  int16_t X,Y,Z;
  float   Xg,Yg,Zg;
    
  printf("ETX_I2C_ACCEL: Task Starting\n");
  
  fd = open( ETX_I2C_DRIVER_PATH, O_WRONLY);
  if( fd < 0 )
  {
    printf("ETX_I2C_ACCEL:ERROR - Failed to open %s: %d\n",
                                                   ETX_I2C_DRIVER_PATH, errno);
    ret = -1;
  }
  
  ret = ETX_ADXL345_Init();
  
  while( ret >= 0 )
  {
    // Read the X
    ret = ETX_I2C_Read( ADXL345_RA_DATAX0, (uint8_t*)&X, 2);
    
    // Read the Y
    ret = ETX_I2C_Read( ADXL345_RA_DATAY0, (uint8_t*)&Y, 2);
    
    // Read the Z
    ret = ETX_I2C_Read( ADXL345_RA_DATAZ0, (uint8_t*)&Z, 2);
    
    Xg = ( X * ADXL345_MG2G_MULTIPLIER * STANDARD_GRAVITY );
    Yg = ( Y * ADXL345_MG2G_MULTIPLIER * STANDARD_GRAVITY );
    Zg = ( Z * ADXL345_MG2G_MULTIPLIER * STANDARD_GRAVITY );
        
    printf("Xg: %f, Yg: %f, Zg: %f\r\n", Xg, Yg, Zg);
  }

  close(fd);
  printf("ETX_I2C_ACCEL: Task finishing\n");
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
  
  printf("ETX_I2C_ACCEL: Starting the Application\n");

  
  ret = task_create( "ETX_I2C_ACCEL",                         // Task Name
                     CONFIG_EXAMPLES_ETX_I2C_ACCEL_PRIORITY,  // Task priority
                     CONFIG_EXAMPLES_ETX_I2C_ACCEL_STACKSIZE, // Task Stack size
                     etx_i2c_accel_task,                      // Task function
                     NULL
                   );
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_I2C_ACCEL: ERROR: Failed to start ETX I2C Accel task: %d\n",
                                                                       errcode);
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}

#endif //#ifdef CONFIG_EXAMPLES_ETX_I2C_ACCEL
