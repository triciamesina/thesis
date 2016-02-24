/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.
 Software License Agreement:
 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.
 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************
 File Description:
 Change History:
  Rev   Description
  1.0   Initial release
  2.1   Updated for simplicity and to use common
                     coding style
********************************************************************/
#include "USB/usb.h"
#include "USB/usb_host_msd.h"
#include "USB/usb_host_msd_scsi.h"
#include "usb_host_hub.h"
#include "fatfs/ff.h"

#include <stdio.h>
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

#ifdef __C30__
    #define PLL_96MHZ_OFF   0xFFFF
    #define PLL_96MHZ_ON    0xF7FF


    // Configuration Bit settings  for an Explorer 16 with USB PICtail Plus
    //      Primary Oscillator:             HS
    //      Internal USB 3.3v Regulator:    Disabled
    //      IOLOCK:                         Set Once
    //      Primary Oscillator Output:      Digital I/O
    //      Clock Switching and Monitor:    Both disabled
    //      Oscillator:                     Primary with PLL
    //      USB 96MHz PLL Prescale:         Divide by 2
    //      Internal/External Switch Over:  Enabled
    //      WDT Postscaler:                 1:32768
    //      WDT Prescaler:                  1:128
    //      WDT Window:                     Non-window Mode
    //      Comm Channel:                   EMUC2/EMUD2
    //      Clip on Emulation Mode:         Reset into Operation Mode
    //      Write Protect:                  Disabled
    //      Code Protect:                   Disabled
    //      JTAG Port Enable:               Disabled

    #if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GB210__)
        _CONFIG2(FNOSC_PRIPLL & POSCMOD_HS & PLL_96MHZ_ON & PLLDIV_DIV2) // Primary HS OSC with PLL, USBPLL /2
        _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)   // JTAG off, watchdog timer off
    #elif defined(__PIC24FJ64GB004__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ256GB106__)
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
        _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV3 & IOL1WAY_ON)
    #elif defined(__PIC24FJ256DA210__)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
    #endif
#elif defined( __PIC32MX__ )
    #pragma config UPLLEN   = ON            // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_20        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_1         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    //#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = OFF           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
    #pragma config DEBUG    = OFF            // Background Debugger Enable

#else


    #error Cannot define configuration bits.

#endif

#define UART_MODULE_ID_1 UART1  // PIM is connected to Explorer through UART1 module
#define UART_MODULE_ID_2 UART2 // PIM is connected to Explorer through UART2 module
#define DESIRED_BAUDRATE (9600) //The desired BaudRate
#define _MAX_FILENAME 13
#define _MAX_PATHNAME 15

typedef struct _select_fn {

	const char root[3];
	const char selectfile[_MAX_FILENAME];
	const char selectpath[_MAX_PATHNAME];

} select_fn; 
    
// GLOBAL VARIABLES
BOOL HubAttached;  // flag if hub device is attached
BYTE DeviceNumber;  
USB_EVENT event;
BYTE HubStatus;
BYTE MSDAttached, MSD1Attached, MSD2Attached, MSD3Attached, MSD4Attached;	// MSD Device attached flag
BYTE MSD1Mounted, MSD2Mounted, MSD3Mounted, MSD4Mounted; // MSD Mount flag
BYTE deviceAddress[_VOLUMES];
BYTE volume;
const char root[3]; // source root volume
const char fn[_MAX_FILENAME]; // selected filename
const char destpath[3][_MAX_PATHNAME]; // destination pathname
int USB1Selected = 0;
int USB2Selected = 0;
int USB3Selected = 0;
int USB4Selected = 0;
char selectfile[10][_MAX_FILENAME]; // copy filename selection array
char selectpath[10][_MAX_PATHNAME]; // copy pathname selection array
int k = 0; // filename selection array index
int m = 0; // pathname selection array index
int d = 0; // destination drive array index
int copying = 0; // copy in progress flag
char rxstring[_MAX_FILENAME];
int s = 0;
char namebuff[_MAX_FILENAME] = "";
int RXdone = 0; // receive string done flag
//int RXnewname = 0; // receive newname string done flag
int renaming = 0; // rename flag
char newname[_MAX_FILENAME]; // copy filename selection array
char oldname[_MAX_PATHNAME]; // copy pathname selection array
char ext[5]; // filename extension
unsigned long int StartCount, FileTime;
const char destdrv[3][3]; // destination drive directory
select_fn selection[10]; //selected filenames
char dirfiles[20][_MAX_FILENAME];

// FUNCTION PROTOTYPES
void PassDirectory(const char choice);
FRESULT read_contents (char *path);
void BTINIT(void);
void SPIINIT(void);
void GetBTCommand(const char character);
void CheckStatus(const char choice);
void BTfunctions(const char select);
UINT32 GetMenuChoice(void);
void SendDataBuffer(const char *buffer, UINT32 size);
extern BYTE CurrentPort;
void TestPutFile(void);
void WriteString(const char *string);
char *findfilename (char* path, int index);
void PutCharacter(const char character);
void PutInteger(unsigned int integer);
FRESULT f_copy (const char sourcename[_MAX_PATHNAME], const char destname1[_MAX_PATHNAME], const char destname2[_MAX_PATHNAME], const char destname3[_MAX_PATHNAME]);
void ClearSelection(void);
void ClearDestination(void);
char UART_RxString(const char character);
void PutChar(const char character);
unsigned int writeSPI1(unsigned int a);

// CONSTANTS
#define USB_MAX_DEVICES 5
#define MAX_ALLOWED_CURRENT	(500)



int main(void)
{
    int i;
    FATFS fatfs[_VOLUMES];
    FRESULT res;

		SYSTEMConfigPerformance(80000000L); 
		 unsigned int cache_status;
		
		 mBMXDisableDRMWaitState();
		 mCheConfigure(3);
		 cache_status = mCheGetCon();
		 cache_status |= CHE_CONF_PF_ALL;
		 mCheConfigure(cache_status);
		 CheKseg0CacheOn();

	/**
    #if defined(__PIC32MX__)
        {
            int  value;
    
            value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );
    
            // Enable the cache for the best performance
            CheKseg0CacheOn();
    
            INTEnableSystemMultiVectoredInt();
    
            value = OSCCON;
            while (!(value & 0x00000020))
            {
                value = OSCCON;    // Wait for PLL lock to stabilize
            }
        }
			DBINIT();

    #endif
	****/

	BTINIT();
	DBINIT();

	// Initialize variables
    HubAttached = FALSE;
	MSD1Attached = 0;
	MSD2Attached = 0;
	MSD3Attached = 0;
	MSD4Attached = 0;
	MSDAttached = 0;
	MSD1Mounted = 0;
	MSD2Mounted = 0;
	MSD3Mounted = 0;
	MSD4Mounted = 0;
    
    //Initialize the stack
    USBInitialize(0);
    
    #if defined(DEBUG_MODE)
        // PPS - Configure U2RX - put on pin 49 (RP10)
        RPINR19bits.U2RXR = 10;

        // PPS - Configure U2TX - put on pin 50 (RP17)
        RPOR8bits.RP17R = 5;

//        UART2Init();
    #endif
	mJTAGPortEnable(1);

    while(1) {
        
	// 	DBPRINTF("USB FILE TRANSFER HUB\n\n\n");

		DeviceNumber = 0;
		for (i=0; i<_VOLUMES;i++) {
			deviceAddress[i] = 0;
		}

        //USB stack process function
        USBTasks(0);
            
        //if no hub and msd devices are plugged in
        while (!USBHostHubDeviceDetect(1)) {
		//	DBPRINTF("deviceAddress = %x\n", deviceAddress);
            USBTasks(0);
        } 

        //if hub is plugged in
        if(USBHostHubDeviceDetect(1)) {
            HubAttached = TRUE;
            event = EVENT_HUB_ATTACH;
           // DBPRINTF("Hub Device Attached\n");
                //Just sit here until the device is removed.
                while(HubAttached == TRUE) {
                    USBTasks(0);
					if (MSDAttached) {

					 if ((MSD1Mounted == 0) && (MSD1Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(0)) {
				//		DBPRINTF("MSD Device Attached in Port 1\n");
						volume = 0;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: USB1 Mounted\n", volume);
								writeSPI1(2);
								MSD1Mounted = 1;
							} // if res
					//	DBPRINTF("0:/%s\n", findfilename("0:", 1));
						}
					}

					 if ((MSD2Mounted == 0) && (MSD2Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(1)) {
					//	DBPRINTF("MSD Device Attached in Port 2\n");
						volume = 1;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: USB2 Mounted\n", volume);
								writeSPI1(4);
								MSD2Mounted = 1;
							} // if res
						}
					}

					 if ((MSD3Mounted == 0) && (MSD3Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(2)) {
					//	DBPRINTF("MSD Device Attached in Port 3\n");
						volume = 2;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: USB3 Mounted\n", volume);
								writeSPI1(6);
								MSD3Mounted = 1;
							} // if res
					//	DBPRINTF("2:/%s\n", findfilename("2:", 1));
						}
					}

					 if ((MSD4Mounted == 0) && (MSD4Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(3)) {
					//	DBPRINTF("MSD Device Attached in Port 4\n");
						volume = 3;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								DBPRINTF("%x: USB4 Mounted\n", volume);
								writeSPI1(8);
								MSD4Mounted = 1;
							} // if res
						}
					}
						
				//	} // if usbhostmsdscsi
				
				//	i = 0;
					} // if msdattached
				} // while HubAttached
        } // while USBHostHubDeviceDetect 
    return 0;
} // while (1)
} // main

/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                void *data, DWORD size )

  Summary:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.

  Description:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.  If the application is able to handle the event, it
    returns TRUE.  Otherwise, it returns FALSE.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of device where event occurred
    USB_EVENT event - Identifies the event that occured
    void *data      - Pointer to event-specific data
    DWORD size      - Size of the event-specific data

  Return Values:
    TRUE    - The event was handled
    FALSE   - The event was not handled

  Remarks:
    The application may also implement an event handling routine if it
    requires knowledge of events.  To do so, it must implement a routine that
    matches this function signature and define the USB_HOST_APP_EVENT_HANDLER
    macro as the name of that function.
  ***************************************************************************/

BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
	FRESULT res;
    switch( event )
    {
        case EVENT_VBUS_REQUEST_POWER:
            // The data pointer points to a byte that represents the amount of power
            // requested in mA, divided by two.  If the device wants too much power,
            // we reject it.
			if (((USB_VBUS_POWER_EVENT_DATA*)data)->current <= (MAX_ALLOWED_CURRENT / 2))
            {
                return TRUE;
            }
            else
            {
             //   DBPRINTF( "\n***** USB Error - device requires too much current *****\n" );
            }
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // Turn off Vbus power.
            // This means that the device was removed
            HubAttached = FALSE;
            return TRUE;
            break;
            
        case EVENT_HUB_ATTACH:
            // Hub device is attached
            HubAttached = TRUE;
		//	PutCharacter('v');
            return TRUE;
            break;

		case EVENT_ATTACH:
			// USB device is detached
		//	DBPRINTF("MSD Device at %x Attached\n", CurrentPort);
			switch (CurrentPort){
				case 1:
					MSD1Attached = 1;
					break;

				case 2:
					MSD2Attached = 1;
					break;

				case 3:
					MSD3Attached = 1;
					break;

				case 4:
					MSD4Attached = 1;
					break;

				default:
					break;					

			}

				deviceAddress[CurrentPort] = CurrentPort + 1;
				MSDAttached = 1;
			return TRUE;
			break;

		case EVENT_DETACH:
			// USB device is detached
		//	DBPRINTF("MSD Device at %x Detached\n", CurrentPort);
			volume = CurrentPort - 1;
			switch (CurrentPort){
				case 1:
					MSD1Attached = 0;
					writeSPI1(12);
					res = f_mount(0, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD1Mounted = 0;
					}
					break;

				case 2:
					MSD2Attached = 0;
					writeSPI1(12);
					res = f_mount(1, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD2Mounted = 0;
					}
					break;

				case 3:
					MSD3Attached = 0;
					writeSPI1(12);
					res = f_mount(2, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD3Mounted = 0;
					}				
					break;

				case 4:
					MSD4Attached = 0;
					writeSPI1(12);
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
						DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD4Mounted = 0;
					}
					break;

				default:
					break;					

			}

			if ((MSD1Attached == 0) && (MSD2Attached == 0) && (MSD3Attached == 0) && (MSD4Attached == 0)) {
				MSDAttached = 0;
			}

			return TRUE;
			break;

        case EVENT_UNSUPPORTED_DEVICE:
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            //UART2PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            //UART2PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            //UART2PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            //UART2PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;
}

static unsigned int _excep_code;
static unsigned int _excep_addr;

// this function overrides the normal _weak_ generic handler
void _general_exception_handler(void)
{

//	writeSPI1(12);
    asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

    _excep_code = (_excep_code & 0x0000007C) >> 2;

    while (1) {
        // Examine _excep_code to identify the type of exceptiona
        // Examine _excep_addr to find the address that caused the exception
    }
}

// FILE SYSTEM COMMANDS
/*
void TestPutFile(void) {
	
	FRESULT res;
    FILINFO fno;
    DIR dir;
	FIL fp;

		DBPRINTF("S\n");
	res = f_open(&fp, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    res = f_puts("Hello World!\n", &fp);
 	res = f_close(&fp);
		DBPRINTF("C\n");

}
*/

/*-----------------------------------------------------------------------*/
/* Read directory contents							                     */
/*-----------------------------------------------------------------------*/
    
FRESULT read_contents (
    char *path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i, j, a;
    //const char *fn;   /* This function assumes non-Unicode configuration */
	const char *fn;
//	char dirfiles[20][_MAX_FILENAME];

	for (i=0; i<j; i++) {
		memset(dirfiles[i], 0, sizeof(dirfiles[i]));
	}
	
	j = 0;

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlenpgm(path);
        for (;;) {

			res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */

			fn = fno.fname;
       
			strncpy(dirfiles[j], fn, strlen(fn)+1);

		//	DBPRINTF("%s/%s\n", path, dirfiles[j]);
			SendDataBuffer(dirfiles[j], strlenpgm(dirfiles[j]));
     		SendDataBuffer(" ", strlen(" "));

			j++;
		}
    }
		
	//	for (a = 0; a<=j; a++) {
		//	SendDataBuffer(dirfiles[a], strlenpgm(dirfiles[a]));
     	//	SendDataBuffer(" ", strlen(" "));
	//		DBPRINTF("%i %s/%s\n", a, path, dirfiles[a]);
	//	}

    return res;
}

/*-----------------------------------------------------------------------*/
/* Find object within directory (filename string)                        */
/*-----------------------------------------------------------------------*/
/*
char *findfilename (
    char* path, 
	int index
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i, a;
    int j = 0;
	const TCHAR *fn;
	char fi[13];
//	char *fi;
	char filenames[20][_MAX_FILENAME];   // This function assumes non-Unicode configuration	

    res = f_opendir(&dir, path);                       // Open the directory
    if (res == FR_OK) {
        i = strlenpgm(path);
        for (;;) {

			res = f_readdir(&dir, &fno);                   //* Read a directory item
            if (res != FR_OK || fno.fname[0] == 0) break;  //* Break on error or end of dir
            if (fno.fname[0] == '.') continue;             //* Ignore dot entry

			fn = fno.fname;

			strncpy(filenames[j], fn, strlen(fn)+1);

		//	DBPRINTF("%s/%s, %i, %i\n", path, filenames[j], j, index);
			j++;

		}
		strncpy(fi, filenames[index], strlen(filenames[index])+1);
    }
	return fi;
}
*/
/*-----------------------------------------------------------------------*/
/* Copy and paste files from one drive to another                        */
/*-----------------------------------------------------------------------*/

FRESULT f_copy (
	const char sourcename[_MAX_PATHNAME],
	const char destname1[_MAX_PATHNAME], 
	const char destname2[_MAX_PATHNAME], 
	const char destname3[_MAX_PATHNAME]
) {
    
    BYTE buffer[10752];   /* File copy buffer */
    FRESULT fr, fr1, fr2, fr3;          /* FatFs function common result code */
	FRESULT fropensrc;
    UINT br, bw;         /* File read/write count */
    static FIL fsrc, fdst1, fdst2, fdst3;

	copying = 1;
//	DBPRINTF("s");

//			DBPRINTF("%s %s %s %s\n", sourcename, destname1, destname2, destname3);

            fropensrc = f_open(&fsrc, sourcename, FA_READ | FA_OPEN_ALWAYS);
			if (fropensrc) {
				copying = 0; 
				return (int) fr;
			}	
			            
            fr1 = f_open(&fdst1, destname1, FA_WRITE | FA_CREATE_ALWAYS);

			if (destname2 != "") {
			fr2 = f_open(&fdst2, destname2, FA_WRITE | FA_CREATE_ALWAYS);
				if (destname3 != "") {
				fr3 = f_open(&fdst3, destname3, FA_WRITE | FA_CREATE_ALWAYS);
				}
			}
			
			if (fr1 && fr2 && fr3) {
				f_close(&fsrc);
				copying = 0;
				return (int) fr1;
			}

    /* Copy source to destination */
    for (;;) {
        fr = f_read(&fsrc, buffer, sizeof buffer, &br);  /* Read a chunk of source file */
        if (fr || br == 0) break; /* error or eof */

		fr = f_write(&fdst1, buffer, br, &bw);            /* Write it to the destination file */
		if ((fr || bw < br) && fr2 && fr3) break;
		f_sync(&fdst1);
		if (!fr2 && destname2 != "") {
			fr = f_write(&fdst2, buffer, br, &bw);            /* Write it to the destination file */
        	if ((fr || bw < br) && fr3) break; /* error or disk full */	
			f_sync(&fdst2);		
				if (!fr3 && destname3 != "") {
			        fr = f_write(&fdst3, buffer, br, &bw);            /* Write it to the destination file */
			        if (fr || bw < br) break; /* error or disk full */
					f_sync(&fdst3);
				}
		}
		
    }

    /* Close open files */
    f_close(&fsrc);
    f_close(&fdst1);  
	if (!fr2 && destname2 != "") {
		f_close(&fdst2);		
		if (!fr3 && destname3 != "") {
		f_close(&fdst3);
		}
	}  

//	DBPRINTF("d");
	copying = 0;
    return fr;
    
}

/*-----------------------------------------------------------------------*/
/* Read filename extension							                     */
/*-----------------------------------------------------------------------*/
 
char *getextension(char *fn) {

	char *ext;

		ext = strrchr(fn, '.');
		if (!ext) {
			return 0;
		    /* no extension */
		} else {
		//    printf("extension is %s\n", ext + 1);
			return ext;
		}

}

unsigned int writeSPI1(unsigned int a) {

         putcSPI1(a);                 //Sends hex data unsigned int data to slave
         int receive = SPI1BUF;            //Read SP1BUF (dummy read)
         SPI1BUF = 0x0;                  //Write SP1BUF- sets Tx flag, if not done read will not clock
         return getcSPI1();            //Generates clock and reads SDO
}

void BTINIT (void) {
	#if defined (__32MX220F032D__) || defined (__32MX250F128D__)
    PPSInput(2,U2RX,RPB5); // Assign RPB5 as input pin for U2RX
    PPSOutput(4,RPB0,U2TX); // Set RPB0 pin as output for U2TX
    #elif defined (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
    PPSInput(2,U1RX,RPF4); // Assign RPF4 as input pin for U1RX
    PPSOutput(2,RPF5,U1TX); // Set RPF5 pin as output for U1TX
    #endif

    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above.
//    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    //mJTAGPortEnable(DEBUG_JTAGPORT_OFF);

// RB0 = 72 - GREEN
// RB1 = 70 - RED
// RB2 = 68 - YELLOW

    mPORTBClearBits(BIT_0); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_0);	// Make RB0, RB1, RB2 as output.
    mPORTBClearBits(BIT_1); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_1);	// Make RB0, RB1, RB2 as output.
    mPORTBClearBits(BIT_2); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_2);	// Make RB0, RB1, RB2 as output.

/*
    // Configure UART1 module, set buad rate, turn on UART, etc.
    UARTConfigure(UART_MODULE_ID_1,UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART_MODULE_ID_1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID_1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID_1, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART_MODULE_ID_1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

	 // Configure UART1 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART_MODULE_ID_1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART_MODULE_ID_1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART_MODULE_ID_1), INT_SUB_PRIORITY_LEVEL_2);	
*/

   //SPI setup
   int rData = SPI1BUF;    //Clears receive buffer
   IFS0CLR = 0x03800000;   //Clears any existing event (rx / tx/ fault interrupt)
   SPI1STATCLR = 0x40;      //Clears overflow
   //Enables the SPI channel (channel, master mode enable | use 8 bit mode | turn on, clock divider)
   SpiChnOpen(1, SPI_CON_MSTEN | SPI_CON_MODE8 | SPI_CON_ON, 4l);   // divide fpb by 4, configure the I/O ports.

    //SpiChnOpen( 1, SPICON_MSTEN | SPICON_CKE | SPICON_ON, 20 );	
	
	 // Configure UART2 module, set buad rate, turn on UART, etc.
    UARTConfigure(UART_MODULE_ID_2,UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL);
    UARTSetFifoMode(UART_MODULE_ID_2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID_2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID_2, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART_MODULE_ID_2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART2 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART_MODULE_ID_2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART_MODULE_ID_2), INT_PRIORITY_LEVEL_3);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART_MODULE_ID_2), INT_SUB_PRIORITY_LEVEL_3);

    // Enable multi-vector interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

}

void InitSelected (void) {
	USB1Selected = 0;
	USB2Selected = 0;
	USB3Selected = 0;
	USB4Selected = 0;
}

void ClearSelection(void) {

	int j, l, c;
	for (j = 0; j < k; j++) {
		strcpy(selectfile[j], "");
	}
	for (l = 0; l < m; l++) {
		strcpy(selectpath[l], "");
	}
	for (c = 0; c < d; c++) {
		strcpy(destdrv[c], "");
		strcpy(destpath[c], "");
	}
	d = 0;
	k = 0;
	m = 0;
}

void GetBTCommand(const char character) { // INTERRUPT EVENT HANDLER

		FRESULT res;
		int n, b;
		int index = 0;

		DBPRINTF("%c", character);


		// GET DIRECTORY

		if (character == 'E' && renaming == 0) { // USB 1 Selected
			InitSelected();
			writeSPI1(2);
			if (MSD1Attached) {
			USB1Selected = 1;
			strncpy(root, "0:", 3);
			read_contents(root);
			}
			else {
		//	writeSPI1(12);
			}
		}

		else if (character == 'F' && renaming == 0) { // USB 2 Selected
			InitSelected();
			writeSPI1(4);
			if (MSD2Attached) {
		//	writeSPI1(4);
			USB2Selected = 1;
			strncpy(root, "1:", 3);
			read_contents(root);
			}
			else {
		//	writeSPI1(12);
			}
		}

		else if (character == 'G' && renaming == 0) { // USB 3 Selected
			InitSelected();
			writeSPI1(6);
			if (MSD3Attached) {
			USB3Selected = 1;
			strncpy(root, "2:", 3);
			read_contents(root);
			}
			else {
		//	writeSPI1(12);
			}
		}	

		else if (character == 'H' && renaming == 0) { // USB 4 Selected
			InitSelected();
			writeSPI1(8);
 			if (MSD4Attached) {
			USB4Selected = 1;
			strncpy(root, "3:", 3);
			read_contents(root);
			}
			else {
		//	writeSPI1(12);
			}
		}

		// STORE SELECTED FILES

		else if (((character>='0' && character<='9') || character == '-') && renaming == 0) { // Find the filename to copy
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {			
				index = atoi(rxstring);
				memset(rxstring, 0, sizeof(rxstring));
			//	strcpy(fn, findfilename(root, index-1));
			//	strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectfile[k], dirfiles[index-1], strlen(dirfiles[index-1])+1);
				strncpy(selectpath[m], root, strlen(root)+1);
			//	strncat(selectpath[m], fn, strlen(fn)+3);
				strncat(selectpath[m], selectfile[k], strlen(selectfile[k])+3);
				strncpy(selection[k].root, root, 3);
				strncpy(selection[k].selectfile, selectfile[k], strlen(selectfile[k])+1);
				strncpy(selection[k].selectpath, selectpath[m], strlen(selectpath[m])+1);
				k++;
				m++;
			//	DBPRINTF("%s, %s, %s, %i\n", selection[k].selectpath, selection[k].selectfile, selection[k].selectfile, k);
			//	DBPRINTF("%s, %s, %i, %s, %i, %s\n", root, fn, k, selectfile[k], m, selectpath[m]);
			RXdone = 0;
			}
		}

/*
		else if (((character>='0' && character<='9') || character == '-') && USB1Selected == 1 && renaming == 0) { // Find the filename to copy from USB1
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {			
			//	index = rxstring - '0';
				index = atoi(rxstring);
				memset(rxstring, 0, sizeof(rxstring));
				strcpy(fn, findfilename("0:", index-1));
				strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], fn, strlen(fn)+3);
				k++;
				m++;
				
("%s, %s, %i, %s, %i, %s\n", root, fn, k, selectfile[k], m, selectpath[m]);
			RXdone = 0;
			}
		}
		
		else if (((character>='0' && character<='9') || character == '-') && USB2Selected == 1 && renaming == 0) { // Find the filename to copy from USB2
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {
				index = rxstring - '0';
				memset(rxstring, 0, sizeof(rxstring));
				strcpy(fn, findfilename("1:", index-1));
				strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], fn, strlen(fn)+3);
				k++;
				m++;
			RXdone = 0;
			}
		//	DBPRINTF("%s, %s\n", rxstring, fn);
		}

		else if (((character>='0' && character<='9') || character == '-') && USB3Selected == 1 && renaming == 0) { // Find the filename to copy from USB3
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {
				index = rxstring - '0';
				memset(rxstring, 0, sizeof(rxstring));
				strcpy(fn, findfilename("2:", index-1));
				strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], fn, strlen(fn)+3);
				k++;
				m++;
			RXdone = 0;
			}
		//	DBPRINTF("%s, %s\n", pathname, fn);
		}

		else if (((character>='0' && character<='9') || character == '-') && USB4Selected == 1 && renaming == 0) { // Find the filename to copy from USB4
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {
				index = rxstring - '0';
				memset(rxstring, 0, sizeof(rxstring));
				strcpy(fn, findfilename("3:", index-1));
				strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], fn, strlen(fn)+3);
				k++;
				m++;
			RXdone = 0;
			}
		//	DBPRINTF("%s, %s\n", pathname, fn);
		}
*/
		// CHOOSE DESTINATION

		else if (character == 'm' && renaming == 0) { // Choose USB1 Destination
			strncpy(destdrv[d], "0:", 3);
			d++;
		}

		else if (character == 'n' && renaming == 0) { // Choose USB2 Destination
			strncpy(destdrv[d], "1:", 3);
			d++;
		}

		else if (character == 'o' && renaming == 0) { // Choose USB3 Destination
			strncpy(destdrv[d], "2:", 3);
			d++;
		}

		else if (character == 'p' && renaming == 0) { // Choose USB4 Destination
			strncpy(destdrv[d], "3:", 3);
			d++;
		}

		// PASTE FILES
/*
		else if (character == 'i' && RXnewname == 0) { // Paste files
		StartCount = ReadCoreTimer();
			for (n = 0; n < k; n++) {
				for (b=0; b<d; b++) {
					strncpy(destpath[b], destdrv[b], 3);
					strncat(destpath[b], selectfile[n], strlen(selectfile[n])+3);
				//	DBPRINTF("%s ", destpath[b]);
				}
				if (d==1) {
				res = f_copy(selectpath[n], destpath[0], "", "");
				}
				else if (d==2) {
				res = f_copy(selectpath[n], destpath[0], destpath[1], "");
				}
				else if (d==3) {
				res = f_copy(selectpath[n], destpath[0], destpath[1], destpath[2]);
				}
				if (res == FR_OK) {
					DBPRINTF("d");
					PutCharacter('n');
				}
				else {
					DBPRINTF("f");
					PutCharacter('r');
				}
			}
		FileTime = (ReadCoreTimer()-StartCount)/80000L;
		DBPRINTF("t %lu\n", FileTime);
		PutCharacter('u');
		ClearSelection();
		}
*/

		else if (character == 'i' && renaming == 0) { // Paste files
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
			for (n = 0; n < k; n++) {
			    mPORTBClearBits(BIT_0 | BIT_1); 
				for (b=0; b<d; b++) {
					strncpy(destpath[b], destdrv[b], 3);
					strncat(destpath[b], selection[n].selectfile, strlen(selection[n].selectfile)+3);
				//	DBPRINTF("%s ", destpath[b]);
				}
				if (d==1) {
				res = f_copy(selection[n].selectpath, destpath[0], "", "");
				}
				else if (d==2) {
				res = f_copy(selection[n].selectpath, destpath[0], destpath[1], "");
				}
				else if (d==3) {
				res = f_copy(selection[n].selectpath, destpath[0], destpath[1], destpath[2]);
				}
				if (res == FR_OK) {	
					DBPRINTF("d");
					mPORTASetBits(BIT_0);
				//	PutInteger(1);
				//	PutCharacter('J');
				}
				else {
					DBPRINTF("f");
					mPORTASetBits(BIT_1);
				//	PutInteger(2);
				//	PutCharacter('K');
				}
			//	mPORTBClearBits(BIT_0 | BIT_1); 
			}
		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		DBPRINTF("t %lu\n", FileTime);
		mPORTBClearBits(BIT_2);
		writeSPI1(10);
		ClearSelection();
		}


		// DELETE FILES

		else if (character == 'q' && renaming == 0) { // Delete files
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
	//	writeSPI1(14);
		for (n = 0; n < k; n++) {
	//		mPORTBClearBits(BIT_0 | BIT_1); 
			res = f_unlink(selection[n].selectpath);
			if (res == FR_OK) {
			//	LEDSuccess();
				mPORTBSetBits(BIT_0);
				DBPRINTF("d");
			//	PutInteger(1);
			//	PutCharacter('J');
			}
			else {
			//	LEDFail();
				mPORTBSetBits(BIT_1);
				DBPRINTF("f");
			//	PutInteger(2);
			//	PutCharacter('K');
			}
		//	mPORTBClearBits(BIT_0 | BIT_1); 
		}
		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		DBPRINTF("t %lu\n", FileTime);
		writeSPI1(10);
		mPORTBClearBits(BIT_2);
		ClearSelection();
		}

		// MOVE FILES

		else if (character == 'j' && renaming == 0) {
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
				mPORTBClearBits(BIT_0 | BIT_1); 
				for (b=0; b<d; b++) {
					strncpy(destpath[b], destdrv[b], 3);
					strncat(destpath[b], selection[n].selectfile, strlen(selection[n].selectfile)+3);
				//	DBPRINTF("%s ", destpath[b]);
				}
				if (d==1) {
				res = f_copy(selection[n].selectpath, destpath[0], "", "");
				}
				else if (d==2) {
				res = f_copy(selection[n].selectpath, destpath[0], destpath[1], "");
				}
				else if (d==3) {
				res = f_copy(selection[n].selectpath, destpath[0], destpath[1], destpath[2]);
				}
				if (res == FR_OK) {
					DBPRINTF("d");
					res = f_unlink(selection[n].selectpath);
					if (res == FR_OK) {
						mPORTBSetBits(BIT_0);
						DBPRINTF("d");
					//	PutInteger(1);
					//	PutCharacter('J');
					}
					else {
						mPORTBSetBits(BIT_1);
						DBPRINTF("f");
					}
				}
				else {
					mPORTBSetBits(BIT_1);
					DBPRINTF("f");
				//	PutInteger(2);
				//	PutCharacter('K');
				}
			//	mPORTBClearBits(BIT_0 | BIT_1); 
			}
		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		DBPRINTF("t %lu\n", FileTime);
		writeSPI1(10);
		mPORTBClearBits(BIT_2);
		ClearSelection();
		}

		// RENAME

		else if (character == 'r' && renaming == 0) {
		renaming = 1;
		strncpy(oldname, root, strlen(root)+1);
		strncat(oldname, selection[0].selectfile, strlen(selection[0].selectfile)+3);
		strcpy(ext, getextension(selection[0].selectfile));
		}

/*
		else if (character == 'r' && RXnewname == 0) {
		renaming = 1;
		}

		else if (((character>='0' && character<='9') || character == '-') && renaming == 1 && RXnewname == 0) { // Find the filename to copy from USB1
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {			
			//	index = rxstring - '0';
				index = atoi(rxstring);
				memset(rxstring, 0, sizeof(rxstring));
			//	strcpy(fn, findfilename(root, index-1));
				strncpy(fn, dirfiles[index-1], strlen(dirfiles[index-1])+1);
				strcpy(ext, getextension(fn));
			//	DBPRINTF("%s", ext);
				strncpy(oldname, root, 3);
				strncat(oldname, fn, strlen(fn)+3);
			//	DBPRINTF("%s, %s\n", fn, oldname);
			RXdone = 0;
			RXnewname = 1;
			}
		}
*/
		else if (character != ' ' && renaming == 1) {
			mPORTBSetBits(BIT_2);
			UART_RxString(character);
			if (RXdone) {
				strncpy(newname, rxstring, strlen(rxstring));
				strncat(newname, ext, strlen(ext)+1);
			//	DBPRINTF("%s, %s", newname, ext);
				memset(rxstring, 0, sizeof(rxstring));
				memset(ext, 0, sizeof(ext));
				StartCount = ReadCoreTimer();
				res = f_rename(oldname, newname);
				if (res == FR_OK) {
					mPORTBSetBits(BIT_0);
					DBPRINTF("d\n");
				//	PutInteger(1);
				//	PutCharacter('J');
				}
				else {
					mPORTBSetBits(BIT_1);
					DBPRINTF("f\n");
				//	PutInteger(2);
				//	PutCharacter('K');
				}
			FileTime = (ReadCoreTimer()-StartCount)/40000L;
			DBPRINTF("t %lu\n", FileTime);
			writeSPI1(10);
			mPORTBClearBits(BIT_2);
			RXdone = 0;
			renaming = 0;
			memset(oldname, 0, sizeof(oldname));
			memset(newname, 0, sizeof(newname));
			ClearSelection();
			}
		}

		else if (character == 's' && renaming == 0) { // RENAME CANCELLED
		renaming = 0;
		}

		// CLEAR SELECTION

		else if (character == 'k' && renaming == 0) {

			InitSelected();
			ClearSelection();

		}		
/*
		else if (character == 'l' && renaming == 0) {

			writeSPI1(16);
		}
*/
		DelayMs(10);
		mPORTBClearBits(BIT_0 | BIT_1); 
}

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void SendDataBuffer(const char *buffer, UINT32 size)
{
    while(size)
    {
        while(!UARTTransmitterIsReady(UART_MODULE_ID_2))
            ;

        UARTSendDataByte(UART_MODULE_ID_2, *buffer);

        buffer++;
        size--;
    }

    while(!UARTTransmissionHasCompleted(UART_MODULE_ID_2))
        ;
}
/*
void PutInteger(unsigned int integer)
{
        while(!UARTTransmitterIsReady(UART_MODULE_ID_2))
            ;

        UARTSendDataByte(UART_MODULE_ID_2, integer);


        while(!UARTTransmissionHasCompleted(UART_MODULE_ID_2))
            ;
}
*/
char UART_RxString(const char character){
	
	RXdone = 0;					// receive string flag
   if (character != '-') {            // while not end of string
       namebuff[s] = character;            // read next character 
       s++;                 // increment pointer to next character
		if (s == 13) {
		s = 0;
		}
   }
	else {
	strncpy(rxstring, namebuff, s+1);
	memset(namebuff, 0, sizeof(namebuff));
	s = 0;
	RXdone = 1;
	}

       return rxstring;    // return the contents of uart
}

/*
void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART_MODULE_ID_1))
            ;

        UARTSendDataByte(UART_MODULE_ID_1, character);


        while(!UARTTransmissionHasCompleted(UART_MODULE_ID_1))
            ;
}

// UART 1 interrupt handler, set at priority level 2

void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void)
{

	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_1)))
	{ 
        // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_1));\
          
	}

	// We don't care about TX interrupt
	if (INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_1)))
	{
	//	  PutCharacter(character);
		  INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_1));
			
	}
}
*/

// UART 2 interrupt handler, set at priority level 3
void __ISR(_UART2_VECTOR, ipl3) IntUart2Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_2)))
	{
            // Echo what we just received.
           GetBTCommand(UARTGetDataByte(UART_MODULE_ID_2));
		     // Clear the RX interrupt Flag
			    INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_2));
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_2)) )
	{
            INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_2));
	}
}
