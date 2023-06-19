//попытка подключиться через usb
//кварцевый генератор на 8МГц и МК работает на частоте 72МГц,  USB модуль на 48МГц-обязательно!!.

#include "stm32f10x.h"
#include "stm32f10xusb.h"//недостающая структура регистров USB в stm32f10x.h+структура таблицы буферов+процедуры
#include "descriptor.h"//массивы описаний устройств,конфигураций,репорта...
//#include "stdbool.h"//стандартная библиотека для работы с типом bool

unsigned char Buff1[64];//массив для обработки принятых данных
unsigned char StringDescriptor5[4] = {0, 0, 0, 0};
unsigned char toHost[20] = {0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90,0x12,0x34, 0x56,0x78,0x90};

	//int ln=0;
//bool status=false;
//int g=0;
//прототипы:
void init_GPIO(void);
void init_RCC(void);
void init_USB(void);
void init_EP0(void);
//int  lenMas( unsigned char *src);//определить длину массива


void init_RCC()//reset & clock control
{

RCC->CR |= RCC_CR_HSEON;//Запускаем генератор HSE,внешний кварц
while (!(RCC->CR & RCC_CR_HSERDY)) {}; // Ждем готовности

RCC->CR |= RCC_CR_HSION;//Запускаем генератор HSI,внутренний rc генератор для записи в регистры флеш
while (!(RCC->CR & RCC_CR_HSIRDY)) {}; // Ждем готовности

RCC->CFGR &= ~RCC_CFGR_SW; //Очистка битов выбора источника тактового сигнала(тактируемся от внутреннего rc)
while((RCC->CFGR & RCC_CFGR_SWS)) {} //Ожидание переключения на HSI
		
FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer,бит включения буфера предварительной выборки
while(!(FLASH->ACR & FLASH_ACR_PRFTBS)) {};	
FLASH->ACR &= ~FLASH_ACR_LATENCY; // Предочистка.
//FLASH->ACR |= FLASH_ACR_LATENCY_2; // Two wait states СТРОКА 7796,КОСЯЧЕК)))
	FLASH->ACR |=2;//так более вяжется с мануалом
//FLASH->ACR |=FLASH_ACR_HLFCYA;//Flash half cycle access enable, бит включения доступа к половинному циклу

	
//PLLMul=x9,USB prescaler=1.5(по умолчанию),AHB prc=1,APB1 psc=2,APB2 psc=1,тогда SYSCLK=APB=APB2=72,APB1=36 MГц,USB=48	




	
 RCC->CFGR |= RCC_CFGR_PLLSRC; //Источником сигнала для PLL выбран HSE (внешний - кварц на 8 МГц)	
 RCC->CFGR &=~RCC_CFGR_SW; // Очистить биты SW0, SW
	RCC->CR &= ~RCC_CR_PLLON; //Отключить генератор PLL
 RCC->CFGR &= ~RCC_CFGR_PLLMULL; //Очистить PLLMULL
	RCC->CFGR |= RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2; //Коефициент умножения 9  (будет 72 МГЦ)	
 RCC->CFGR |= RCC_CFGR_PPRE1_2;	//APB1 psc=2
	
	 
	
 RCC->CR |= RCC_CR_PLLON; //Включить генератор PLL
 while(!(RCC->CR & RCC_CR_PLLRDY)) {} //Ожидание готовности PLL

	
 RCC->CFGR |= RCC_CFGR_SW_PLL; //Выбрать источником тактового сигнала PLL	 
while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {} //Ожидание переключения на PLL 
	
}




void init_GPIO()
{
// PA12(d+) И PA11(d-) USB
//D+ на плате уже подтянут резистором 10к +3,3 В,в режиме AF программная подтяжка невозможна	
//	для FS/HS-устройств подтягивается линия D+, Для LS-устройств подтягивается линия D-  
// Максимальный размер пакета для нулевой конечной точки (64 для HS, 8 — для FS и LS)
//Full Speed. Полноскоростной режим. Скорость передачи 12 Мбит/с.
//High Speed. Высокоскоростной режим. Появился лишь в спецификации 2.0. Скорость передачи 480 Мбит/с.	
	
	
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;// enable clock for GPIOA
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // enable clock for Altirnate function
	//AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_1; // только SWD, JTAG отключен
	
	RCC->APB1ENR|=RCC_APB1ENR_USBEN; // enable clock for USB
	
	//Вход-выход USB:
	//Для начала сбрасываем все конфигурационные биты в нули
  GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 
                | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
  
	GPIOA->CRH |=  (GPIO_CRH_CNF12_1|GPIO_CRH_CNF11_1);//AF PP 
	GPIOA->CRH |=  (GPIO_CRH_MODE12| GPIO_CRH_MODE11);//скорость на максимум
	
	
	//Выход МСО:
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
	GPIOA->CRH |=  GPIO_CRH_CNF8_1;//AF PP
	GPIOA->CRH |= GPIO_CRH_MODE8;//11-скорость на максимум
	RCC->CFGR |= RCC_CFGR_MCO;//|RCC_CFGR_MCO_1|RCC_CFGR_MCO_2);	//выход МСО к PLLCLK/2
	
	

}

void init_USB()
{
//Включаю тактирование USB. Убираю бит PWDN из USB->CNTR. Жду 1 микросекунду (по даташиту нужно для стабилизации источника опорного напряжения),
//	пишу нули в CNTR, BTABLE и ISTR. Разрешаю прерывания USB_FS_WKUP и CAN1_RX0. 
//	Пишу в CNTR значение USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_CTRM.
RCC->CFGR &=~ RCC_CFGR_USBPRE; // Настраиваем частоту USB (= частота ядра / 1.5)
RCC->APB1ENR |=	RCC_APB1ENR_USBEN; // Включаем тактирование USB от шины APB1

	USB->CNTR &= ~USB_CNTR_PDWN;//1-вход в режим Power down;0-выход из режима выключения (Exit Power Down)
USB->CNTR |= USB_CNTR_FRES;//	1-принудительный сброс периферии USB, точно так же, как это происходит по сигналу RESET по шине USB
USB->CNTR &= ~USB_CNTR_FRES; //FRES-сбрасывает биты настроек конечных точек
//USB->ISTR = 0;
//NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
USB->CNTR |= USB_CNTR_RESETM //1-разрешено прерывание RESET, генерируется запрос прерывания, когда установится 
	                           //   соответствующий бит в регистре USB_ISTR.	                           
        	| USB_CNTR_CTRM    //1-прерывание CTR разрешено, генерируется запрос прерывания, когда установится 
	                           // соответствующий бит в регистре USB_ISTR.корректной передачи (Correct Transfer, CTR)
//          | USB_CNTR_SUSPM 
//	        | USB_CNTR_WKUPM            
	
	        | 0; // | USB_CNTR_ERRM; //;

//другой вариант	
//USB->BTABLE = 0;//начальный адрес таблицы выделения буфера в выделенной памяти пакетов
	//( Адрес таблицы дискрипторов внутри пакетной памяти)
//USB->CNTR = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_CTRM;
	
	}

//---------------------------------------------------------------------------------------------------------
//*P1-указатель на записываемый массив данных,P2-порядковый номер ячейки Таблица дескрипторов буферов (BDT)
// Память пакетов (Packet Memory): это локальная память, где физически содержатся буферы пакетов (Packet Buffers).
//	Она может использоваться интерфейсом буфера пакетов, который создает структуру данных и к которому можно напрямую 
//получить доступ из программы приложения. Размер Packet Memory составляет 512 байт, что структурировано как 256 слов по 16 бит.
//физически память пакетов занимает	1024 байт,но половина не доступна 
//---------------------------------------------------------------------------------------------------------------	
	void TO_WRITE_PMA(unsigned char *src, unsigned int P, int N)
	 //src -указатель на ресурс, передаваемый массив
   //P-начальный адрес памяти буферов пакетов в единицах контроллера USB
	 //N-количество элементов для записи в буфер передачи
	{
	P=(P*2)+0x40006000;//вычисляем физический адрес памяти как он выглядит для ядра
		
for(int i=0; i<N; i+=2)//шаг =2
	{		
		*(__IO uint16_t*)P = *(src+i)+(*(src+i+1)<<8);
   P=P+4;
	}
	
			
}
//--------------------------------------------------------------------------------------------------------------
void TO_READ_PMA(unsigned int P, unsigned char *src, int N)
  //P-начальный адрес памяти буферов пакетов в единицах контроллера USB
	//src -указатель на ресурс, куда считываем
  //N-количество элементов для записи в буфер src
  {
P=(P*2)+0x40006000;//вычисляем физический адрес памяти как он выглядит для ядра
	for(int i=0; i<N; i+=2)
	{	
//		if(P&1)//если адрес четный
//		
//    else
		*(src+i	)= (*(__IO uint16_t*)P);
		*(src+i+1	)= (*(__IO uint16_t*)P)>>8;
   P=P+4;
	}
}
//---------------------------------------------------------------------------------------------------------------

	void init_EP(void)// инициализация конечных точек
	{
		// Разрешен прием данных, установлен тип точки CONTROL, запрещена передача
		USB->EPR[0]=0; //сбросить все биты ??
    setEPType( 0,EP_CONTROL	 ) ;//установка типа конечной точки
   	setStatRx(0, VALID	);// эта конечная точка разрешена для приема
			
		setEPType( 1, EP_INT	 ) ;//установка типа конечной точки interrupt
		setStatTx(1, NAK	);// эта конечная точка занята
	//	 setStatRx(1, VALID	);
//    setEPType( 2, EP_INT	 ) ;//установка типа конечной точки interrupt
//   	setStatRx(2, NAK	);// эта конечная точка занята

		}
	
//---------------------------------------------------------------------------------------------------------------		
	void tableInit() 
	{  
		//[USB_BTABLE] + n*16    ADDRn_TX
		//[USB_BTABLE] + n*16 + 4   COUNTn_TX
		//[USB_BTABLE] + n*16 + 8   ADDRn_RX
		//[USB_BTABLE] + n*16 + 12  COUNTn_RX
		//__IO uint32_t * register_address = (uint32_t *)  0x40005C40U; // Адрес нашего регистра USB_CNTR в памяти	
     //*(__IO uint32_t *)register_address &=~ 0x0001; //сбросить 0 бит FRES	
		USB->BTABLE = 0;//начальный адрес таблицы выделения буфера в выделенной памяти пакетов
		//Настройка буферов EP0,EP1,EP2:
   DiscrTable->ADR0TX=0x40;   //до этого адреса идет таблица адресов
   DiscrTable->COUNT0TX=0;//0x40; //максимальная длина передаваемых пакетов
   DiscrTable->ADR0RX=0x80;   //начальный адрес приемника EP0
   DiscrTable->COUNT0RX=0x8400; //с RX не так все однозначно..BL_SIZE=1,NUM_BLOCK=1,младшие байты регистра-счетчик принятых байт
		//старшие используются для определения события переполнения буфера
	 DiscrTable->ADR1TX=0xc0;// EP1 будет работать на передачу,c0-начальный адрес
	 DiscrTable->COUNT1TX=0x00;//0x40;// у нее буфер тоже будет 64б
	 DiscrTable->ADR2RX=0x100;// EP2 будет на прием
	 DiscrTable->COUNT2RX=0x8400;//размер буфера 64 байт
		// ИТОГО 320 БАЙТ ПАКЕТНОЙ ПАМЯТИ
	
}
	


void USB_LP_CAN1_RX0_IRQHandler() //обработчик прерываний 
	{	
		register unsigned char length,l=0;
		
		
		if (USB->ISTR & USB_ISTR_RESET)//периферия USB детектирует активный сигнал USB RESET
			{
			
	   init_EP();//инициализируем конечные точки
		 tableInit() ;//??
		USB->DADDR = 0;
    USB->DADDR |= USB_DADDR_EF;	// Включаем модуль USB, адрес устройства 0		
				
			USB->ISTR &= ~USB_ISTR_RESET;
		return;
			}
	
	if(USB->ISTR & USB_ISTR_CTR)// конечная точка успешно завершила транзакцию,для выяснения причины надо смотреть DIR и EP_ID
	{
	  switch ( USB->ISTR & 0xf)//отделяем последние 4 бита EP_ID
		{			
			case 0://если прерывание вызвано EP0
				TO_READ_PMA(0x80,	 Buff1,64);	
			      switch(Buff1[0]+(Buff1[1]<<8))
								 {case 0x0680:						// Запрос какого то дискриптора
									 
									 	{
											switch(Buff1[3]){
			              	case 0x01:							// Запрос дискриптора устройства
									 
                   TO_WRITE_PMA(DeviceDiscriptor,0x40,18);//заполняем буфер передачи дискриптором устройства
									 DiscrTable->COUNT0TX= Buff1[6]; // выставляем количество требуюмых хосту байт               								 }
			           
									   setStatTx(0, VALID	);// эта конечная точка разрешена для передачи
									   while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
										 setStatRx(0, VALID	);// эта конечная точка разрешена для приема
										 while((USB->EPR[0] & 0x8000) == 0){}	
										 setStatRx(0, VALID	);// эта конечная точка разрешена для приема
									 
						         break;// Запрос дискриптора устройства
										
										case 0x02:							// Запрос дескриптора конфигурации
										
										 l=sizeof(ConfigDescriptor)/sizeof(*ConfigDescriptor);
									
										TO_WRITE_PMA(ConfigDescriptor,0x40,l);//заполняем буфер передачи дискриптором конфигурации
									 
											 length =Buff1[6]+(Buff1[7]<<8);
				            	if(length!=9)length=l;				// Без этой строчки не работает,хост первый раз запрашивает 9 байт
										    DiscrTable->COUNT0TX= length; 
										
									  	setStatTx(0, VALID	);// эта конечная точка разрешена для передачи
									    while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
											setStatRx(0, VALID	);// эта конечная точка разрешена для приема
											while((USB->EPR[0] & 0x8000) == 0){}	
										  setStatRx(0, VALID	);// эта конечная точка разрешена для приема				
									
										 break;// Запрос дискриптора конфигурации
									
                  			case 0x03:								//Запрос дескриптора строки
                       l=sizeof(StringDescriptor0)/sizeof(*StringDescriptor0);
												//register int l=lenMas(ConfigDescriptor);
												TO_WRITE_PMA(StringDescriptor0,0x40,l);//заполняем буфер передачи дискриптором конфигурации
                      DiscrTable->COUNT0TX= l;
                     
											 setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                     	 while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                       setStatRx(0, VALID	);// эта конечная точка разрешена для приема
											 while((USB->EPR[0] & 0x8000) == 0){}	
											 setStatRx(0, VALID	);// эта конечная точка разрешена для приема 
												
												
												break;// Запрос дискриптора строки

										    default:
              l=sizeof(StringDescriptor3)/sizeof(*StringDescriptor3);
							TO_WRITE_PMA(StringDescriptor3, 64,l);		// Заполняем буфер передачи  строковым дискриптором
							 DiscrTable->COUNT0TX=l ;

											setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                     	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                      setStatRx(0, VALID	);// эта конечная точка разрешена для приема
										  while((USB->EPR[0] & 0x8000) == 0){}	
										  setStatRx(0, VALID	);// эта конечная точка разрешена для приема 	
												
												
												
												break;//default
												 
											
												 
												 
												 
												 
												 
									
											}
										
										
										
										
										break;//Запрос какого то дискриптора
										}
											
											
											case 0x0500:						// SET_ADDRESS 
							
								   DiscrTable->COUNT0TX=0;
									 setStatTx(0, VALID	);// эта конечная точка разрешена для передачи
									 while((USB->EPR[0] & 0x80) == 0){}			// !!!ВАЖНО Проверяем CTR_TX (Ждем завершения передачи)
								   // отправляем еще пустой пакет с 0 адреса,а потом устанавливаем присвоенный
									USB->DADDR  = Buff1[2] | USB_DADDR_EF;	
									setStatRx(0, VALID	);// эта конечная точка разрешена для приема
									
										 
										 
										 
										 break; 	// SET_ADDRESS 
									 
									   case 0x0900:						//SET_CONFIGURATION
										 DiscrTable->COUNT0TX=0;
									 	 
										  setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                     	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                      setStatRx(0, VALID	);// эта конечная точка разрешена для приема
										 						 
										 	break; 	// SET_CONFIGURATION
										
                    case 0x0A21:							//SET_IDLE
                   
							  			 DiscrTable->COUNT0TX=0;
									  	 setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                      	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                       setStatRx(0, VALID	);// эта конечная точка разрешена для приема

                    	break; 	// SET_IDLE

                  
								
								 case 0x0681:								//Запрос нестандартного дескриптора
								    switch(Buff1[3]){
			                 	case 0x22:							//Запрос дескриптора репорта
                     l=sizeof(ReportDescriptor)/sizeof(*ReportDescriptor);         
				            	TO_WRITE_PMA(ReportDescriptor,0x40,l );		// Заполняем буфер передачи нулевым дискриптором репорта
								      DiscrTable->COUNT0TX=l;
								 
						        	 setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                     	 while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                       setStatRx(0, VALID	);// эта конечная точка разрешена для приема
											 while((USB->EPR[0] & 0x8000) == 0){}	
											 setStatRx(0, VALID	);// эта конечная точка разрешена для приема 
											// status=true;	//в теории в этот момент мы нормально законнектелись
												
												
								 
					           break; 	// Запрос дескриптора репорта		 
		              	
										 default:////Запрос нестандартного дескриптора
										
										 
										 break; 	// 	 default:	//Запрос нестандартного дескриптора
										
										}			 
		            	 break; 		//Запрос нестандартного дескриптора					 
								 	 
								
		case 0x0921:								//HID запрос SET_REPORT (Feature),через него мы читаем данные

	
		                   setStatRx(0, VALID	);// эта конечная точка разрешена для приема
											 while((USB->EPR[0] & 0x8000) == 0){}	
			                 DiscrTable->COUNT0TX=0;
									 	 												 
												TO_READ_PMA(0x80,	 Buff1,64);	 
												 
												 
										  setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                   	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                      setStatRx(0, VALID	);// эта конечная точка разрешена для приема
																						 
			 break;//end 	HID запрос SET_REPORT
	     
//			                 case 0x01A1:								//HID запрос GET_REPORT (Feature),отправка данных	
//			   по запросу хоста...тогда они отправляются через EP0,не проверял на практике..

//                      l=sizeof(toHost)/sizeof(*toHost);         
//				            	TO_WRITE_PMA(toHost,0xc0,l );		// Заполняем буфер передачи нулевым дискриптором репорта
//								      DiscrTable->COUNT1TX=l;
//                      
//			                 setStatTx(1, VALID	);// эта конечная точка разрешена для передачи 
//                     	 while((USB->EPR[1] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
//												 //ждем пустого пакета подтверждения:
//			                 setStatRx(0, VALID	);// эта конечная точка разрешена для приема
//											 while((USB->EPR[0] & 0x8000) == 0){}	
//											 setStatRx(0, VALID	);// эта конечная точка разрешена для приема 





//		                   	break;

									
												
												
												
												
												
												
												
												
												
												
												
		  case 0x0102:							 

				break;				
			
										 default:  //это для каких то других запросов пустой пакет подтверждения
											 DiscrTable->COUNT0TX=0;
									 	 
										  setStatTx(0, VALID	);// эта конечная точка разрешена для передачи 
                     	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
                      setStatRx(0, VALID	);// эта конечная точка разрешена для приема 
//										  	while((USB->EPR[0] & 0x80) == 0){}			// Проверяем CTR_TX (Ждем завершения передачи)
//                     setStatRx(0, VALID	);// эта конечная точка разрешена для приема 
										 
										 break; 	// 	 default:	
		




								 } //конец запроса какого то дискриптора
									 
									 
												 
						 
										 				 
									 
			
			USB->EPR[1] &= ~USB_EP1R_CTR_TX;		//затык на случай если 2 прерывания CTR от EP1 и EPO,приоритет по номеру 						 
      	setStatRx(0, VALID	);//swith case...
		break;//case 0://если прерывание вызвано EP0
		
			  case 1://если прерывание вызвано EP1
			//обработчик для EP1
		  			
			USB->EPR[1] &= ~USB_EP1R_CTR_TX;	
		  USB->EPR[0] &= ~USB_EP1R_CTR_RX;
			USB->ISTR &= ~USB_ISTR_CTR;
		
				break;					 
								 
								 
								 
								 
								 
								 
								 
								 
			
			 
			case 2://если прерывание вызвано EP2
			//обработчик для EP2
			break;
		
			 default:
			     
				break;
		
		
		
	}
	
	USB->ISTR &= ~USB_ISTR_CTR;
	
	}  //конец обработки признака удачной транзакции
	

//if(USB->ISTR & USB_CNTR_SUSPM)			
//{ 	USB->ISTR &= ~USB_ISTR_SUSP;  }		
//		
//if(USB->ISTR & USB_CNTR_WKUPM)			
//{ 	USB->ISTR &= ~USB_ISTR_WKUP;  }		


}	//конец обработчика прерываний

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

	
	while(1)// ниже отсебятина
	{ 
		         
	                   if (!( USB->EPR[1] & (1<<4)))//если статус NAK
										 { //ln++;
											 TO_WRITE_PMA(toHost,0xc0,20 );		// Заполняем буфер передачи 
								      DiscrTable->COUNT1TX=20; //должно соответствовать репорту   
                     ResetDtogTx(1);//по идее надо делать для конечных точек типа INT
		                setStatTx(1, VALID	);// эта конечная точка разрешена для передачи }
										 setStatRx(0, VALID	);// эта конечная точка разрешена для приема  	 
									
										 }	



 for (int i=0;i<1535;i++ ){}			// Проверяем CTR_TX (Ждем завершения передачи)

	}

}
