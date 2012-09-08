/*******************************************************************************
* File Name: main.c
*******************************************************************************/

#include "Lcd.h"
#include "stm32f10x_lib.h"
#include "usb_lib.h"
#include "HW_V1_Config.h"
#include "memory.h"

void __APP_Start(void);

void main(void)
{
/*--------------initialization-----------*/

  Set_System();
  NVIC_Configuration();
  GPIO_Config();
  SD_Card_Check();
  USB_Init();
  ADC_Configuration();
  Timer_Configuration();
  LCD_Initial();
  Clear_Screen(BLACK); 
  Display_Logo(110,150);  
 
/*----------Power ON Information----------*/ 

  Display_Str(80, 87, GRN,   PRN, "System Initializing");
  Display_Str(102, 71, GRN,   PRN, "Please Wait");
  Display_Str(8, 39, WHITE, PRN, "DSO Firmware Copyright (c) BenF 2010"); 
  Display_Str(8, 23, YEL,   PRN, "LIB ver 3.01");
  
  //WaitForKey();

  __APP_Start();
}
/********************************* END OF FILE ********************************/
