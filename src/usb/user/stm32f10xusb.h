//нехватающая структура для настройки битов USB в файле "stm32f10x.h"
//





#ifndef __STM32F10xUSB_H
#define __STM32F10xUSB_H
#include "stm32f10x.h"

#define USB_BASE ((uint32_t)0x40005C00)                 /*!<USB Registers Base Address               */ 
#define USB_PMA_ADDR    ((uint32_t)0x40006000)          /*!<USB Packet Memory Area Address           */

// типы статусов STAT_TX и STAT_RX конечной точки
#define SDIS			0
#define STALL			1
#define NAK				2
#define VALID			3
// биты конфигурирования конечной точки
#define CTR_RX			0x8000
#define CTR_TX			0x80
// типы конечной точки
#define EP_BULK			0
#define EP_CONTROL	0x200
#define EP_ISO			0x400
#define EP_INT			0x600




// для создания структуры используем карту USB регистров стр652,поля структуры по 32 бита(регистры 32 битные)
// в случае ошибок,неправильно вычислятся адреса битов
typedef struct {
	uint32_t EPR[8];              //Address offset: 0x00 to 0x1C
	uint32_t RESERVED[8];         //Address offset: 0x20 to 0x3F
	uint32_t CNTR;                //Address offset: 0x40
	uint32_t ISTR;                //Address offset: 0x44
	uint32_t FNR;                 //Address offset: 0x48
	uint32_t DADDR;               //Address offset: 0x4C
	uint32_t BTABLE;              //Address offset: 0x50
}USB_TypeDef;


// структура таблицы адресов памяти пакетов 32 поля по 32 бита(4 байта),доступны для записи.чтения 2 младших байта
typedef struct {//4 байтах4х8=128 байт физической памяти и 64 байт с тз контроллера usb
   volatile  uint32_t ADR0TX;
   volatile  uint32_t COUNT0TX;
   volatile  uint32_t ADR0RX;
   volatile  uint32_t COUNT0RX;

   volatile  uint32_t ADR1TX;
   volatile  uint32_t COUNT1TX;
   volatile  uint32_t ADR1RX;
   volatile  uint32_t COUNT1RX;

   volatile  uint32_t ADR2TX;
   volatile  uint32_t COUNT2TX;
   volatile  uint32_t ADR2RX;
   volatile  uint32_t COUNT2RX;

   volatile  uint32_t ADR3TX;
   volatile  uint32_t COUNT3TX;
   volatile  uint32_t ADR3RX;
   volatile  uint32_t COUNT3RX;

   volatile  uint32_t ADR4TX;
   volatile  uint32_t COUNT4TX;
   volatile  uint32_t ADR4RX;
   volatile  uint32_t COUNT4RX;

   volatile  uint32_t ADR5TX;
   volatile  uint32_t COUNT5TX;
   volatile  uint32_t ADR5RX;
   volatile  uint32_t COUNT5RX;
	
	 volatile  uint32_t ADR6TX;
   volatile  uint32_t COUNT6TX;
   volatile  uint32_t ADR6RX;
   volatile  uint32_t COUNT6RX;

   volatile  uint32_t ADR7TX;
   volatile  uint32_t COUNT7TX;
   volatile  uint32_t ADR7RX;
   volatile  uint32_t COUNT7RX;
			
} DiscrTable_TypeDef;

#define USB ((USB_TypeDef *)USB_BASE) 


#define DiscrTable ((DiscrTable_TypeDef *) USB_PMA_ADDR )//при условии,что BTABLE=0

//Либо способ в лоб:
//__IO uint32_t * register_address = (uint32_t *)  0x40005C40U; // Адрес нашего регистра USB_CNTR в памяти	
//*(__IO uint32_t *)register_address &=~ 0x0001; //сбросить 0 бит FRES	
	

void setStatTx(uint8_t ep, uint16_t stat) ;//установка статуса передачи
void setStatRx(uint8_t ep, uint16_t stat) ;//установка статуса приема
void setEPType(uint8_t ep, uint16_t type) ;//установка типа конечной точки
void ResetDtogTx(uint8_t ep);//сброс бита DTOG_TX

#endif /* __STM32F10xUSB_H */
