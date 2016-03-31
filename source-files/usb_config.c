/*
********************************************************************************
                                                                                
Software License Agreement                                                      
                                                                                
Copyright © 2007-2008 Microchip Technology Inc.  All rights reserved.           
                                                                                
Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital signal controller  
that is integrated into your product or third party product (pursuant to the    
sublicense terms in the accompanying license agreement).                        
                                                                                
You should refer to the license agreement accompanying this Software for        
additional information regarding your rights and obligations.                   
                                                                                
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,   
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF        
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.  
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER       
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR    
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES         
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR     
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF        
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES          
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.     
                                                                                
********************************************************************************
*/

// Created by the Microchip USBConfig Utility, Version 2.0.0.0, 11/18/2008, 8:08:56

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "USB\usb.h"
#include "USB\usb_host_msd.h"
#include "USB\usb_host_msd_scsi.h"
#include "usb_host_hub.h"

// *****************************************************************************
// Media Interface Function Pointer Table for the Mass Storage client driver
// *****************************************************************************
// MODIFICATION 1/14/2016

//CLIENT_DRIVER_TABLE usbMediaInterfaceTable =
//{                                           
//    USBHostMSDSCSIInitialize,
//    USBHostMSDSCSIEventHandler,
//    0
//};

// *****************************************************************************
// Client Driver Function Pointer Table for the USB Embedded Host foundation
// *****************************************************************************

CLIENT_DRIVER_TABLE usbClientDrvTable[] =
{                                        
    {
        USBHostMSDInitialize,
        USBHostMSDEventHandler,
        0
    },
    {
        USBHostHubInitialize,
        USBHostHubEventHandler,
        0
    }
};

// *****************************************************************************
// USB Embedded Host Targeted Peripheral List (TPL)
// *****************************************************************************

USB_TPL usbTPL[] =
{
    // idVendor, idProduct, bConfiguration, index of ClientDriver, flags(HNP,Class,Config)
    { INIT_VID_PID( 0x04B4, 0x6570 ), 0, 1, {0} }, // CY4607
    
    // bClass, bSubClass, bProtocol, bConfiguration, index of ClientDriver, flags(HNP,Class,Config)
    { INIT_CL_SC_P( 8ul, 6ul, 0x50ul ), 0, 0, {TPL_CLASS_DRV} }, // Thumbdrives
    { INIT_CL_SC_P( 0x09ul, 0x00ul, 0x00ul ), 0, 1, {TPL_CLASS_DRV} } // Hub Class
};

