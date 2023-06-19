//����������� ��������� ��� ��������� ����� USB � ����� "stm32f10x.h"
//





#ifndef __STM32F10xUSB_H
#define __STM32F10xUSB_H
#include "stm32f10x.h"

#define USB_BASE ((uint32_t)0x40005C00)                 /*!<USB Registers Base Address               */ 
#define USB_PMA_ADDR    ((uint32_t)0x40006000)          /*!<USB Packet Memory Area Address           */

// ���� �������� STAT_TX � STAT_RX �������� �����
#define SDIS			0
#define STALL			1
#define NAK				2
#define VALID			3
// ���� ���������������� �������� �����
#define CTR_RX			0x8000
#define CTR_TX			0x80
// ���� �������� �����
#define EP_BULK			0
#define EP_CONTROL	0x200
#define EP_ISO			0x400
#define EP_INT			0x600




// ��� �������� ��������� ���������� ����� USB ��������� ���652,���� ��������� �� 32 ����(�������� 32 ������)
// � ������ ������,����������� ���������� ������ �����
typedef struct {
	uint32_t EPR[8];              //Address offset: 0x00 to 0x1C
	uint32_t RESERVED[8];         //Address offset: 0x20 to 0x3F
	uint32_t CNTR;                //Address offset: 0x40
	uint32_t ISTR;                //Address offset: 0x44
	uint32_t FNR;                 //Address offset: 0x48
	uint32_t DADDR;               //Address offset: 0x4C
	uint32_t BTABLE;              //Address offset: 0x50
}USB_TypeDef;


// ��������� ������� ������� ������ ������� 32 ���� �� 32 ����(4 �����),�������� ��� ������.������ 2 ������� �����
typedef struct {//4 ������4�8=128 ���� ���������� ������ � 64 ���� � �� ����������� usb
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


#define DiscrTable ((DiscrTable_TypeDef *) USB_PMA_ADDR )//��� �������,��� BTABLE=0

//���� ������ � ���:
//__IO uint32_t * register_address = (uint32_t *)  0x40005C40U; // ����� ������ �������� USB_CNTR � ������	
//*(__IO uint32_t *)register_address &=~ 0x0001; //�������� 0 ��� FRES	
	

void setStatTx(uint8_t ep, uint16_t stat) ;//��������� ������� ��������
void setStatRx(uint8_t ep, uint16_t stat) ;//��������� ������� ������
void setEPType(uint8_t ep, uint16_t type) ;//��������� ���� �������� �����
void ResetDtogTx(uint8_t ep);//����� ���� DTOG_TX

#endif /* __STM32F10xUSB_H */
