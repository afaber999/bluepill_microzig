#include "stm32f10x.h"
#include "stm32f10xusb.h"




void setStatTx(uint8_t ep, uint16_t stat) 
	{
	register uint16_t tmp = USB->EPR[ep] & 0x73f;//������������ register ��������� ����������� ������� ��������
	tmp ^= (stat << 4);                          // ���������� �� � ������, � � �������� ���������� ��� �������� �������
	USB->EPR[ep] = tmp ;//| CTR_RX | CTR_TX;-���� ��������� ������ ������  0..
  }
void setStatRx(uint8_t ep, uint16_t stat) 
	{
	register uint16_t tmp = USB->EPR[ep] & 0x370f;
	tmp ^= (stat << 12);
	USB->EPR[ep] = tmp ;  //| CTR_RX | CTR_TX;//#define CTR_RX			0x8000 #define CTR_TX			0x80
   }

void setEPType(uint8_t ep, uint16_t type) 
	{
		
		
		
	register uint16_t tmp = USB->EPR[ep] & 0x10f;
	tmp |= type ;//| CTR_RX | CTR_TX;
	USB->EPR[ep] = tmp|ep;//� ������� 3 ���� ���������� ����� ���� ����� ��� �����
  }

void ResetDtogTx(uint8_t ep)
{
register uint16_t tmp = USB->EPR[ep]&0x40; //������ �������� ����
if(tmp)  USB->EPR[ep] |=(1<<6);

}


