/****************************************************************************
 * examples/etx_gpio/etx_gpio_app.c
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
#include <nuttx/signal.h>

#ifdef CONFIG_EXAMPLES_ETX_GPIO

#define ETX_GPIO_DRIVER_PATH "/dev/etx_gpio"    // GPIO Driver path

#define ETX_INPUT_GPIO_NUM   18                 // Input GPIO number
#define ETX_OUTPUT_GPIO_NUM  02                 // Output GPIO number

/****************************************************************************
 * Private Data
 ****************************************************************************/
#define GPIOC_REGISTER   _GPIOC(1)              // IOCTL command to register
#define GPIOC_UNREGISTER _GPIOC(2)              // IOCTL command to unregister

#define ETX_GPIO_SIGNO   SIGUSR1                // Signal number 

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
 
static bool is_signal_received = false;

static etx_gpio gpio_in;

static etx_gpio gpio_out;

static uint64_t oldTime;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_signal_handler
 ****************************************************************************/

static void etx_signal_handler(int signo, FAR siginfo_t *info, FAR void *ucontext)
{
  /* This if check is to eliminate the spurious interrupts.
  ** Added software debounce mechanism.
  */
  if(( g_system_timer - oldTime ) > 20 ) 
  {
    oldTime = g_system_timer;
    printf("Signal received from the etx_gpio driver\n");
    is_signal_received = true;
  }
}
/****************************************************************************
 * Name: etx_gpio_task
 ****************************************************************************/

static int etx_gpio_task(int argc, char *argv[])
{
  int ret = 0;
  struct sigaction act = { 0 };
  struct sigaction oact;
  uint8_t gpio_in_val;
  uint8_t gpio_out_val;
  
  printf("ETX_GPIO: Task Starting\n");
  
  int fd = open( ETX_GPIO_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    printf("ETX_GPIO:ERROR - Failed to open %s: %d\n", ETX_GPIO_DRIVER_PATH, errno);
    ret = -1;
  }
  
  /* Register signal handler */
  act.sa_sigaction = etx_signal_handler;
  act.sa_flags     = SA_SIGINFO;
  sigemptyset(&act.sa_mask);
  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, ETX_GPIO_SIGNO);

  ret = sigaction(ETX_GPIO_SIGNO, &act, &oact);
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_GPIO: ERROR: sigaction failed: %d\n", errcode);
  }
  
  /* Register the Input GPIO */
  gpio_in.gpio_type  = ETX_GPIO_IN_INT;
  gpio_in.gpio_num   = ETX_INPUT_GPIO_NUM;
  gpio_in.gpio_value = &gpio_in_val;
  gpio_in.data       = NULL;
  
  if( ret >= 0 )
  {
    ret = ioctl(fd, GPIOC_REGISTER, (unsigned long)((uintptr_t)&gpio_in));
    if (ret < 0)
    {
      int errcode = errno;
      printf("ETX_GPIO: ERROR: GPIOC_REGISTER ioctl failed: %d\n", errcode);
    }
  }
  
  /* Register the Output GPIO */
  gpio_out.gpio_type  = ETX_GPIO_OUT;
  gpio_out.gpio_num   = ETX_OUTPUT_GPIO_NUM;
  gpio_out.gpio_value = &gpio_out_val;
  gpio_out.data       = NULL;
  
  if( ret >= 0 )
  {
    ret = ioctl(fd, GPIOC_REGISTER, (unsigned long)((uintptr_t)&gpio_out));
    if (ret < 0)
    {
      int errcode = errno;
      printf("ETX_GPIO: ERROR: GPIOC_REGISTER ioctl failed: %d\n", errcode);
    }
  }
  
  while( ret >= 0 )
  {
    if( is_signal_received == true )
    {
      //toggle the gpio val
      gpio_out_val = !gpio_out_val;
      
      //write the GPIO value to the driver
      ret = write( fd, (const void*)&gpio_out, sizeof(gpio_out) );
      is_signal_received = false;
    }
    usleep(50);
  }
  
  /* Unregister the GPIOs */
  ret = ioctl(fd, GPIOC_UNREGISTER, (unsigned long)((uintptr_t)&gpio_in));
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_GPIO: ERROR: GPIOC_UNREGISTER ioctl failed: %d\n", errcode);
    ret = -1;
  }
  
  ret = ioctl(fd, GPIOC_REGISTER, (unsigned long)((uintptr_t)&gpio_out));
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_GPIO: ERROR: GPIOC_UNREGISTER ioctl failed: %d\n", errcode);
    ret = -1;
  }

  close(fd);
  printf("ETX_GPIO: ERROR - Task finishing\n");
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
  
  printf("ETX_GPIO: Starting the Application\n");

  
  ret = task_create( "ETX_GPIO",                         // Task Name
                     CONFIG_EXAMPLES_ETX_GPIO_PRIORITY,  // Task priority
                     CONFIG_EXAMPLES_ETX_GPIO_STACKSIZE, // Task Stack size
                     etx_gpio_task,                      // Task function
                     NULL
                   );
  if (ret < 0)
  {
    int errcode = errno;
    printf("ETX_GPIO: ERROR: Failed to start ETX GPIO task: %d\n", errcode);
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}

#endif //#ifdef CONFIG_EXAMPLES_ETX_GPIO
