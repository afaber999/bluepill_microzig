#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H
#include "stm32f10x.h"

//макроопределения для дескриптора устройства
#define  USB_MAX_EP0_SIZE 64//максимальный размер пакетов для Endpoint 0 (при конфигурировании)
#define LOBYTE(USBD_VID)   0x83
#define HIBYTE(USBD_VID)   0x04//idVendor (0x0483)
#define  _LOBYTE(USBD_PID_FS) 0x11
#define  _HIBYTE(USBD_PID_FS) 0x57 //idProduct (0x5711)
#define  USBD_MAX_NUM_CONFIGURATION 1//количество возможных конфигураций. У нас одна.
//
#define USB_CUSTOM_HID_CONFIG_DESC_SIZE 41 // общий размер дескриптора конфигурации в байтах
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE 36//длина report-дескриптора	58
 



#define  CUSTOM_HID_EPIN_ADDR 	0x81//адрес й конечной точки и направление (IN1)
#define  CUSTOM_HID_EPIN_SIZE    20//максимальная длина пакета
#define CUSTOM_HID_EPOUT_ADDR 0x02//адрес 2й конечной точки и направление (OUT2)
#define CUSTOM_HID_EPOUT_SIZE    20//максимальная длина пакета

/* USB Standard Device Descriptor */
 unsigned char DeviceDiscriptor[18]=
  {
    0x12,                       /*bLength */
    1,       /*bDescriptorType*/
    0x00, 0x02,                       /*bcdUSB */
   
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,          /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    _LOBYTE(USBD_PID_FS),           /*idVendor*/
    _HIBYTE(USBD_PID_FS),           /*idVendor*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,                       // bcdDevice rel. DEVICE_VER_H.DEVICE_VER_L  номер релиза устройства
    1,           /*Index of manufacturer  string*/
    2,       /*Index of product string*/
    3,        /*Index of serial number string*/
    USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
  } ; /* USB_DeviceDescriptor */


/* USB CUSTOM_HID device Configuration Descriptor */
 unsigned char ConfigDescriptor[USB_CUSTOM_HID_CONFIG_DESC_SIZE]=
{
  0x09, /* bLength: Configuration Descriptor size */
  0x02, /* bDescriptorType: Configuration */
  USB_CUSTOM_HID_CONFIG_DESC_SIZE,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface,в конфигурации всего один интерфейс*/
  0x01,         /*bConfigurationValue: Configuration value,индекс данной конфигурации*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration,индекс строки, которая описывает эту конфигурацию*/
  0xe0,         /*bmAttributes: bus powered признак того, что устройство будет питаться от шины USB*/
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
 
  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  0x04,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints,количество эндпоинтов*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  0x21 , /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  0x05, /*bDescriptorType:тип дескриптора - endpoints*/
 
  CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint,тип конечной точки - Interrupt endpoint*/
  CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  0x32,          /*bInterval: Polling Interval (20 ms)*/
  /* 34 */
 
  0x07,	         /* bLength: Endpoint Descriptor size */
  0x05,	/* bDescriptorType:тип дескриптора - endpoints */
  CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03,	/* bmAttributes: Interrupt endpoint,тип конечной точки - Interrupt endpoint*/
  CUSTOM_HID_EPOUT_SIZE,	/* wMaxPacketSize: 2 Bytes max  */
  0x00,
  0x32,	/* bInterval: Polling Interval (20 ms) */
  /* 41 */
} ;

//uint32_t lengsh=sizeof(ReportDescriptor)/sizeof(*ReportDescriptor);


unsigned char ReportDescriptor[USBD_CUSTOM_HID_REPORT_DESC_SIZE] =
{
  
//	 0x06, 0x00, 0xff,		// USAGE_PAGE (Generic Desktop)
//		  0x09, 0x01,			// USAGE (Vendor Usage 1)
//		  0xa1, 0x01,			// COLLECTION (Application)
//		   0x09, 0x01,                    //   USAGE (Vendor Usage 1)
//	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//   0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//		  0x75, 0x08,			// REPORT_SIZE (8)
//		  0x95, 2,				// REPORT_COUNT(64)...количество передаваемых данных,одно поле из этого занято,те по факту-1 поле
//		  0xB1, 0x02,			//FEATURE (Data,Var,Abs)
		
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/* USER CODE BEGIN 0 */ 
    0x06, 0x00, 0xff,              // 	USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // 	USAGE (Vendor Usage 1)
    // System Parameters
    0xa1, 0x01,                    // 	COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 20, 	                   //   REPORT_COUNT (4)
    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
  //  0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
// ________________________________________________________________________________   
	  0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
//    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//    0x75, 0x08,                    //   REPORT_SIZE (8)
//    0x95, 0x04,                    //   REPORT_COUNT (1)
    //0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
//    0x85, 0x02,                    //   REPORT_ID (2)
//    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
  //  0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
//    0x85, 0x02,                    //   REPORT_ID (2)
//    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 20, 	                   //   REPORT_COUNT (4)
    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
  /* USER CODE END 0 */ 
 0xc0    /*     END_COLLECTION	             */
 
};

unsigned char StringDescriptor0[4] = {4, 3, 9, 13};


unsigned char StringDescriptor3[10] = {10, 3, 51, 0, 51, 0, 51, 0, 51, 0};

unsigned char StringDescriptor4[26] = { 26, /* bLength */
3, /* bDescriptorType */
'S', 0, 'V', 0, 'A', 0, 'T', 0, 'S', 0, 'M', 0, '0', 0, '0', 0, '0', 0, '0', 0,
		'1', 0, '0', 0 };









#endif
