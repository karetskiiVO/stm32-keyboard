#include <stm32l151xc.h>
#include "pinmacro.h"
#include "hardware.h"
#include "usb_lib.h"
#include "usb_hid.h"
#include "clock.h"

void __attribute__((weak)) _init(void){}
void __attribute__((weak)) SystemInit(void){}

void sleep (uint32_t time) {
    while (time--) asm volatile("nop");
}

int main (void) {
    HIDDevice usb;

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
  
    clock_HS(1);
    GPIO_config(GLED); GPIO_config(RLED);
    GPIO_config(LBTN); GPIO_config(JBTN);
    
    // USB_setup();
    usb.start();
    __enable_irq();

    while (1) {
        //usb_class_poll();
        usb.poll();
        sleep(100000);
    }
}

