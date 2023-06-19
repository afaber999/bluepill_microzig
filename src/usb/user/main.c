//������� ������������ ����� usb
//��������� ��������� �� 8��� � �� �������� �� ������� 72���,  USB ������ �� 48���-�����������!!.

#include "stm32f10x.h"
#include "stm32f10xusb.h"//����������� ��������� ��������� USB � stm32f10x.h+��������� ������� �������+���������
#include "descriptor.h"//������� �������� ���������,������������,�������...
//#include "stdbool.h"//����������� ���������� ��� ������ � ����� bool

unsigned char Buff1[64];//������ ��� ��������� �������� ������
unsigned char StringDescriptor5[4] = {0, 0, 0, 0};
unsigned char toHost[20] = {0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90};

	//int ln=0;
//bool status=false;
//int g=0;
//���������:
void init_GPIO(void);
void init_RCC(void);
void init_USB(void);
void init_EP0(void);
//int  lenMas( unsigned char *src);//���������� ����� �������


void init_RCC()//reset & clock control
{

RCC->CR |= RCC_CR_HSEON;//��������� ��������� HSE,������� �����
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // ���� ����������

RCC->CR |= RCC_CR_HSION;//��������� ��������� HSI,���������� rc ��������� ��� ������ � �������� ����
while (!(RCC->CR & RCC_CR_HSIRDY)) {}; // ���� ����������

RCC->CFGR &= ~RCC_CFGR_SW; //������� ����� ������ ��������� ��������� �������(����������� �� ����������� rc)
while((RCC->CFGR & RCC_CFGR_SWS)) {} //�������� ������������ �� HSI
		
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer,��� ��������� ������ ��������������� �������
while(!(FLASH->ACR & FLASH_ACR_PRFTBS)) {};	
FLASH->ACR &= ~FLASH_ACR_LATENCY; // �����������.
//FLASH->ACR |= FLASH_ACR_LATENCY_2; // Two wait states ������ 7796,�������)))
	FLASH->ACR |=2;//��� ����� ������� � ��������
//FLASH->ACR |=FLASH_ACR_HLFCYA;//Flash half cycle access enable, ��� ��������� ������� � ����������� �����

	
//PLLMul=x9,USB prescaler=1.5(�� ���������),AHB prc=1,APB1 psc=2,APB2 psc=1,����� SYSCLK=APB=APB2=72,APB1=36 M��,USB=48	




	
 RCC->CFGR |= RCC_CFGR_PLLSRC; //���������� ������� ��� PLL ������ HSE (������� - ����� �� 8 ���)	
 RCC->CFGR &=~RCC_CFGR_SW; // �������� ���� SW0, SW
	RCC->CR &= ~RCC_CR_PLLON; //��������� ��������� PLL
 RCC->CFGR &= ~RCC_CFGR_PLLMULL; //�������� PLLMULL
	RCC->CFGR |= RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2; //���������� ��������� 9  (����� 72 ���)	
 RCC->CFGR |= RCC_CFGR_PPRE1_2;	//APB1 psc=2
	
	 
	
 RCC->CR |= RCC_CR_PLLON; //�������� ��������� PLL
 while(!(RCC->CR & RCC_CR_PLLRDY)) {} //�������� ���������� PLL

	
 RCC->CFGR |= RCC_CFGR_SW_PLL; //������� ���������� ��������� ������� PLL	 
while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {} //�������� ������������ �� PLL 
	
}




void init_GPIO()
{
// PA12(d+) � PA11(d-) USB
//D+ �� ����� ��� �������� ���������� 10� +3,3 �,� ������ AF ����������� �������� ����������	
//	��� FS/HS-��������� ������������� ����� D+, ��� LS-��������� ������������� ����� D-  
// ������������ ������ ������ ��� ������� �������� ����� (64 ��� HS, 8 � ��� FS � LS)
//Full Speed. ��������������� �����. �������� �������� 12 ����/�.
//High Speed. ���������������� �����. �������� ���� � ������������ 2.0. �������� �������� 480 ����/�.	
	
	
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;// enable clock for GPIOA
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
	//AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_1; // ������ SWD, JTAG ��������
	
	RCC->APB1ENR|=RCC_APB1ENR_USBEN; // enable clock for USB
	
	//����-����� USB:
	//��� ������ ���������� ��� ���������������� ���� � ����
  GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 
                | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
  
	GPIOA->CRH |=  (GPIO_CRH_CNF12_1|GPIO_CRH_CNF11_1);//AF PP 
	GPIOA->CRH |=  (GPIO_CRH_MODE12| GPIO_CRH_MODE11);//�������� �� ��������
	
	
	//����� ���:
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
	GPIOA->CRH |=  GPIO_CRH_CNF8_1;//AF PP
	GPIOA->CRH |= GPIO_CRH_MODE8;//11-�������� �� ��������
	RCC->CFGR |= RCC_CFGR_MCO;//|RCC_CFGR_MCO_1|RCC_CFGR_MCO_2);	//����� ��� � PLLCLK/2
	
	

}

void init_USB()
{
//������� ������������ USB. ������ ��� PWDN �� USB->CNTR. ��� 1 ������������ (�� �������� ����� ��� ������������ ��������� �������� ����������),
//	���� ���� � CNTR, BTABLE � ISTR. �������� ���������� USB_FS_WKUP � CAN1_RX0. 
//	���� � CNTR �������� USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_CTRM.
RCC->CFGR &=~ RCC_CFGR_USBPRE; // ����������� ������� USB (= ������� ���� / 1.5)
RCC->APB1ENR |=	RCC_APB1ENR_USBEN; // �������� ������������ USB �� ���� APB1

	USB->CNTR &= ~USB_CNTR_PDWN;//1-���� � ����� Power down;0-����� �� ������ ���������� (Exit Power Down)
USB->CNTR |= USB_CNTR_FRES;//	1-�������������� ����� ��������� USB, ����� ��� ��, ��� ��� ���������� �� ������� RESET �� ���� USB
USB->CNTR &= ~USB_CNTR_FRES; //FRES-���������� ���� �������� �������� �����
//USB->ISTR = 0;
//NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
USB->CNTR |= USB_CNTR_RESETM //1-��������� ���������� RESET, ������������ ������ ����������, ����� ����������� 
	                           //   ��������������� ��� � �������� USB_ISTR.	                           
        	| USB_CNTR_CTRM    //1-���������� CTR ���������, ������������ ������ ����������, ����� ����������� 
	                           // ��������������� ��� � �������� USB_ISTR.���������� �������� (Correct Transfer, CTR)
//          | USB_CNTR_SUSPM 
//	        | USB_CNTR_WKUPM            
	
	        | 0; // | USB_CNTR_ERRM; //;

//������ �������	
//USB->BTABLE = 0;//��������� ����� ������� ��������� ������ � ���������� ������ �������
	//( ����� ������� ������������ ������ �������� ������)
//USB->CNTR = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_CTRM;
	
	}

//---------------------------------------------------------------------------------------------------------
//*P1-��������� �� ������������ ������ ������,P2-���������� ����� ������ ������� ������������ ������� (BDT)
// ������ ������� (Packet Memory): ��� ��������� ������, ��� ��������� ���������� ������ ������� (Packet Buffers).
//	��� ����� �������������� ����������� ������ �������, ������� ������� ��������� ������ � � �������� ����� �������� 
//�������� ������ �� ��������� ����������. ������ Packet Memory ���������� 512 ����, ��� ��������������� ��� 256 ���� �� 16 ���.
//��������� ������ ������� ��������	1024 ����,�� �������� �� �������� 
//---------------------------------------------------------------------------------------------------------------	
	void TO_WRITE_PMA(unsigned char *src, unsigned int P, int N)
	 //src -��������� �� ������, ������������ ������
   //P-��������� ����� ������ ������� ������� � �������� ����������� USB
	 //N-���������� ��������� ��� ������ � ����� ��������
	{
	P=(P*2)+0x40006000;//��������� ���������� ����� ������ ��� �� �������� ��� ����
		
for(int i=0; i<N; i+=2)//��� =2
	{		
		*(__IO uint16_t*)P = *(src+i)+(*(src+i+1)<<8);
   P=P+4;
	}
	
			
}
//--------------------------------------------------------------------------------------------------------------
void TO_READ_PMA(unsigned int P, unsigned char *src, int N)
  //P-��������� ����� ������ ������� ������� � �������� ����������� USB
	//src -��������� �� ������, ���� ���������
  //N-���������� ��������� ��� ������ � ����� src
  {
P=(P*2)+0x40006000;//��������� ���������� ����� ������ ��� �� �������� ��� ����
	for(int i=0; i<N; i+=2)
	{	
//		if(P&1)//���� ����� ������
//		
//    else
		*(src+i	)= (*(__IO uint16_t*)P);
		*(src+i+1	)= (*(__IO uint16_t*)P)>>8;
   P=P+4;
	}
}
//---------------------------------------------------------------------------------------------------------------

	void init_EP(void)// ������������� �������� �����
	{
		// �������� ����� ������, ���������� ��� ����� CONTROL, ��������� ��������
		USB->EPR[0]=0; //�������� ��� ���� ??
    setEPType( 0,EP_CONTROL	 ) ;//��������� ���� �������� �����
   	setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
			
		setEPType( 1, EP_INT	 ) ;//��������� ���� �������� ����� interrupt
		setStatTx(1, NAK	);// ��� �������� ����� ������
	//	 setStatRx(1, VALID	);
//    setEPType( 2, EP_INT	 ) ;//��������� ���� �������� ����� interrupt
//   	setStatRx(2, NAK	);// ��� �������� ����� ������

		}
	
//---------------------------------------------------------------------------------------------------------------		
	void tableInit() 
	{  
		//[USB_BTABLE] + n*16    ADDRn_TX
		//[USB_BTABLE] + n*16 + 4   COUNTn_TX
		//[USB_BTABLE] + n*16 + 8   ADDRn_RX
		//[USB_BTABLE] + n*16 + 12  COUNTn_RX
		//__IO uint32_t * register_address = (uint32_t *)  0x40005C40U; // ����� ������ �������� USB_CNTR � ������	
     //*(__IO uint32_t *)register_address &=~ 0x0001; //�������� 0 ��� FRES	
		USB->BTABLE = 0;//��������� ����� ������� ��������� ������ � ���������� ������ �������
		//��������� ������� EP0,EP1,EP2:
   DiscrTable->ADR0TX=0x40;   //�� ����� ������ ���� ������� �������
   DiscrTable->COUNT0TX=0;//0x40; //������������ ����� ������������ �������
   DiscrTable->ADR0RX=0x80;   //��������� ����� ��������� EP0
   DiscrTable->COUNT0RX=0x8400; //� RX �� ��� ��� ����������..BL_SIZE=1,NUM_BLOCK=1,������� ����� ��������-������� �������� ����
		//������� ������������ ��� ����������� ������� ������������ ������
	 DiscrTable->ADR1TX=0xc0;// EP1 ����� �������� �� ��������,c0-��������� �����
	 DiscrTable->COUNT1TX=0x00;//0x40;// � ��� ����� ���� ����� 64�
	 DiscrTable->ADR2RX=0x100;// EP2 ����� �� �����
	 DiscrTable->COUNT2RX=0x8400;//������ ������ 64 ����
		// ����� 320 ���� �������� ������
	
}
	


void USB_LP_CAN1_RX0_IRQHandler() //���������� ���������� 
	{	
		register unsigned char length,l=0;
		
		
		if (USB->ISTR & USB_ISTR_RESET)//��������� USB ����������� �������� ������ USB RESET
			{
			
	   init_EP();//�������������� �������� �����
		 tableInit() ;//??
		USB->DADDR = 0;
    USB->DADDR |= USB_DADDR_EF;	// �������� ������ USB, ����� ���������� 0		
				
			USB->ISTR &= ~USB_ISTR_RESET;
		return;
			}
	
	if(USB->ISTR & USB_ISTR_CTR)// �������� ����� ������� ��������� ����������,��� ��������� ������� ���� �������� DIR � EP_ID
	{
	  switch ( USB->ISTR & 0xf)//�������� ��������� 4 ���� EP_ID
		{			
			case 0://���� ���������� ������� EP0
				TO_READ_PMA(0x80,	 Buff1,64);	
			      switch(Buff1[0]+(Buff1[1]<<8))
								 {case 0x0680:						// ������ ������ �� �����������
									 
									 	{
											switch(Buff1[3]){
			              	case 0x01:							// ������ ����������� ����������
									 
                   TO_WRITE_PMA(DeviceDiscriptor,0x40,18);//��������� ����� �������� ������������ ����������
									 DiscrTable->COUNT0TX= Buff1[6]; // ���������� ���������� ��������� ����� ����               								 }
			           
									   setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� ��������
									   while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
										 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
										 while((USB->EPR[0] & 0x8000) == 0){}	
										 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
									 
						         break;// ������ ����������� ����������
										
										case 0x02:							// ������ ����������� ������������
										
										 l=sizeof(ConfigDescriptor)/sizeof(*ConfigDescriptor);
									
										TO_WRITE_PMA(ConfigDescriptor,0x40,l);//��������� ����� �������� ������������ ������������
									 
											 length =Buff1[6]+(Buff1[7]<<8);
				            	if(length!=9)length=l;				// ��� ���� ������� �� ��������,���� ������ ��� ����������� 9 ����
										    DiscrTable->COUNT0TX= length; 
										
									  	setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� ��������
									    while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
											setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
											while((USB->EPR[0] & 0x8000) == 0){}	
										  setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������				
									
										 break;// ������ ����������� ������������
									
                  			case 0x03:								//������ ����������� ������
                       l=sizeof(StringDescriptor0)/sizeof(*StringDescriptor0);
												//register int l=lenMas(ConfigDescriptor);
												TO_WRITE_PMA(StringDescriptor0,0x40,l);//��������� ����� �������� ������������ ������������
                      DiscrTable->COUNT0TX= l;
                     
											 setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                     	 while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                       setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
											 while((USB->EPR[0] & 0x8000) == 0){}	
											 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 
												
												
												break;// ������ ����������� ������

										    default:
              l=sizeof(StringDescriptor3)/sizeof(*StringDescriptor3);
							TO_WRITE_PMA(StringDescriptor3, 64,l);		// ��������� ����� ��������  ��������� ������������
							 DiscrTable->COUNT0TX=l ;

											setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                     	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                      setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
										  while((USB->EPR[0] & 0x8000) == 0){}	
										  setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 	
												
												
												
												break;//default
												 
											
												 
												 
												 
												 
												 
									
											}
										
										
										
										
										break;//������ ������ �� �����������
										}
											
											
											case 0x0500:						// SET_ADDRESS 
							
								   DiscrTable->COUNT0TX=0;
									 setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� ��������
									 while((USB->EPR[0] & 0x80) == 0){}			// !!!����� ��������� CTR_TX (���� ���������� ��������)
								   // ���������� ��� ������ ����� � 0 ������,� ����� ������������� �����������
									USB->DADDR  = Buff1[2] | USB_DADDR_EF;	
									setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
									
										 
										 
										 
										 break; 	// SET_ADDRESS 
									 
									   case 0x0900:						//SET_CONFIGURATION
										 DiscrTable->COUNT0TX=0;
									 	 
										  setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                     	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                      setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
										 						 
										 	break; 	// SET_CONFIGURATION
										
                    case 0x0A21:							//SET_IDLE
                   
							  			 DiscrTable->COUNT0TX=0;
									  	 setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                      	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                       setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������

                    	break; 	// SET_IDLE

                  
								
								 case 0x0681:								//������ �������������� �����������
								    switch(Buff1[3]){
			                 	case 0x22:							//������ ����������� �������
                     l=sizeof(ReportDescriptor)/sizeof(*ReportDescriptor);         
				            	TO_WRITE_PMA(ReportDescriptor,0x40,l );		// ��������� ����� �������� ������� ������������ �������
								      DiscrTable->COUNT0TX=l;
								 
						        	 setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                     	 while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                       setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
											 while((USB->EPR[0] & 0x8000) == 0){}	
											 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 
											// status=true;	//� ������ � ���� ������ �� ��������� ��������������
												
												
								 
					           break; 	// ������ ����������� �������		 
		              	
										 default:////������ �������������� �����������
										
										 
										 break; 	// 	 default:	//������ �������������� �����������
										
										}			 
		            	 break; 		//������ �������������� �����������					 
								 	 
								
		case 0x0921:								//HID ������ SET_REPORT (Feature),����� ���� �� ������ ������

	
		                   setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
											 while((USB->EPR[0] & 0x8000) == 0){}	
			                 DiscrTable->COUNT0TX=0;
									 	 												 
												TO_READ_PMA(0x80,	 Buff1,64);	 
												 
												 
										  setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                   	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                      setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
																						 
			 break;//end 	HID ������ SET_REPORT
	     
//			                 case 0x01A1:								//HID ������ GET_REPORT (Feature),�������� ������	
//			   �� ������� �����...����� ��� ������������ ����� EP0,�� �������� �� ��������..

//                      l=sizeof(toHost)/sizeof(*toHost);         
//				            	TO_WRITE_PMA(toHost,0xc0,l );		// ��������� ����� �������� ������� ������������ �������
//								      DiscrTable->COUNT1TX=l;
//                      
//			                 setStatTx(1, VALID	);// ��� �������� ����� ��������� ��� �������� 
//                     	 while((USB->EPR[1] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
//												 //���� ������� ������ �������������:
//			                 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������
//											 while((USB->EPR[0] & 0x8000) == 0){}	
//											 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 





//		                   	break;

									
												
												
												
												
												
												
												
												
												
												
												
		  case 0x0102:							 

				break;				
			
										 default:  //��� ��� ����� �� ������ �������� ������ ����� �������������
											 DiscrTable->COUNT0TX=0;
									 	 
										  setStatTx(0, VALID	);// ��� �������� ����� ��������� ��� �������� 
                     	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
                      setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 
//										  	while((USB->EPR[0] & 0x80) == 0){}			// ��������� CTR_TX (���� ���������� ��������)
//                     setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������ 
										 
										 break; 	// 	 default:	
		




								 } //����� ������� ������ �� �����������
									 
									 
												 
						 
										 				 
									 
			
			USB->EPR[1] &= ~USB_EP1R_CTR_TX;		//����� �� ������ ���� 2 ���������� CTR �� EP1 � EPO,��������� �� ������ 						 
      	setStatRx(0, VALID	);//swith case...
		break;//case 0://���� ���������� ������� EP0
		
			  case 1://���� ���������� ������� EP1
			//���������� ��� EP1
		  			
			USB->EPR[1] &= ~USB_EP1R_CTR_TX;	
		  USB->EPR[0] &= ~USB_EP1R_CTR_RX;
			USB->ISTR &= ~USB_ISTR_CTR;
		
				break;					 
								 
								 
								 
								 
								 
								 
								 
								 
			
			 
			case 2://���� ���������� ������� EP2
			//���������� ��� EP2
			break;
		
			 default:
			     
				break;
		
		
		
	}
	
	USB->ISTR &= ~USB_ISTR_CTR;
	
	}  //����� ��������� �������� ������� ����������
	

//if(USB->ISTR & USB_CNTR_SUSPM)			
//{ 	USB->ISTR &= ~USB_ISTR_SUSP;  }		
//		
//if(USB->ISTR & USB_CNTR_WKUPM)			
//{ 	USB->ISTR &= ~USB_ISTR_WKUP;  }		


}	//����� ����������� ����������

//----------------------------------------------------------------------------------------------------------------
int main(void)
{
init_RCC();	
init_GPIO();
init_USB();
init_EP();	

USB->ISTR = 0;
NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);	
USB->ISTR = 0;		

	
	while(1)// ���� ����������
	{ 
		         
	                   if (!( USB->EPR[1] & (1<<4)))//���� ������ NAK
										 { //ln++;
											 TO_WRITE_PMA(toHost,0xc0,20 );		// ��������� ����� �������� 
								      DiscrTable->COUNT1TX=20; //������ ��������������� �������   
                     ResetDtogTx(1);//�� ���� ���� ������ ��� �������� ����� ���� INT
		                setStatTx(1, VALID	);// ��� �������� ����� ��������� ��� �������� }
										 setStatRx(0, VALID	);// ��� �������� ����� ��������� ��� ������  	 
									
										 }	



 for (int i=0;i<1535;i++ ){}			// ��������� CTR_TX (���� ���������� ��������)

	}

}
