#include "main.hpp"
#include "stm32l431xx.h"

void SystemInit(){
    RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN;
}

int main()
{
	while(true){

    }

    return 0;
}
