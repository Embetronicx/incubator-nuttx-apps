/****************************************************************************
 * examples/etx_led/etx_led_app.c
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

#ifdef CONFIG_EXAMPLES_ETX_LED

#define ETX_LED_DRIVER_PATH "/dev/etx_led"    // LED Driver path

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gpio
{
  uint8_t gpio_val;
  uint8_t gpio_num;
};

static struct gpio etx_gpio;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_led_task
 ****************************************************************************/

static int etx_led_task(int argc, char *argv[])
{
  int ret = 0;
  
  printf("ETX_LED: Task Starting\n");
  
  int fd = open( ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    printf("ETX_LED:ERROR - Failed to open %s: %d\n", ETX_LED_DRIVER_PATH, errno);
    ret = -1;
  }
  
  while( ret >= 0 )
  {
    printf("ETX_LED: GPIO  = %d Value %d\n", etx_gpio.gpio_num, etx_gpio.gpio_val);
    ret = write( fd, (const void*)&etx_gpio, sizeof(etx_gpio) );
    
    //toggle the gpio val
    etx_gpio.gpio_val = !etx_gpio.gpio_val;
    
    usleep(500*1000L);  
  }

  printf("ETX_LED: ERROR - Task finishing\n");
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
  
  printf("ETX_LED: Starting the Application\n");
  
  if( argc < 2 )
  {
    printf("Usage : etx_led needs one argument which is GPIO number to toggle\n");
    printf("Example : To toggle the GPIO 2, please enter \"etx_led 2\"\n");
    return EXIT_FAILURE;
  }
  
  etx_gpio.gpio_val = 1;              //Initialy turn ON the LED
  etx_gpio.gpio_num = atoi(argv[1]);  //Get the GPIO Number from user
  
  ret = task_create( "ETX_LED",                         // Task Name
                     CONFIG_EXAMPLES_ETX_LED_PRIORITY,  // Task priority
                     CONFIG_EXAMPLES_ETX_LED_STACKSIZE, // Task Stack size
                     etx_led_task,                      // Task function
                     NULL
                   );
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_LED: ERROR: Failed to start ETX LED task: %d\n", errcode);
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}

#endif //#ifdef CONFIG_EXAMPLES_ETX_LED
