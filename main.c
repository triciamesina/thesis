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

// CONSTANTS

#define UART_MODULE_ID_2 UART2 // PIM is connected to Explorer through UART2 module
#define DESIRED_BAUDRATE (9600) // The desired BaudRate
#define _MAX_FILENAME 13
#define _MAX_PATHNAME 15
#define _MAX_DIR_ENTRIES 50
#define _MAX_SELECTION 15
#define USB_MAX_DEVICES 5
#define MAX_ALLOWED_CURRENT	(500)

// MACRO DEFINITION
#define FILETIME_MS(x) ((x)/(SYS_FREQ/2000))

// STRUCT TYPEDEF

typedef struct _select_fn {

	const char root[3];
	const char selectfile[_MAX_FILENAME];
	const char selectpath[_MAX_PATHNAME];

} select_fn; 

typedef struct _directory_list {
	
	char dirfiles[_MAX_DIR_ENTRIES][_MAX_FILENAME];
	char dirpath[_MAX_DIR_ENTRIES][_MAX_PATHNAME];

} directory_list;

// GLOBAL VARIABLES
BOOL HubAttached;  // flag if hub device is attached
BYTE DeviceNumber;
USB_EVENT event; // application event handler value
BYTE HubStatus; // hub status
BYTE MSDAttached, MSD1Attached, MSD2Attached, MSD3Attached, MSD4Attached;	// MSD Device attached flag
BYTE MSD1Mounted, MSD2Mounted, MSD3Mounted, MSD4Mounted; // MSD Mount flag
BYTE volume; // MSD logical drive
const char root[3]; // source root volume
const char fn[_MAX_FILENAME]; // selected filename
const char destpath[3][_MAX_PATHNAME]; // destination pathname
int USB1Selected = 0;
int USB2Selected = 0;
int USB3Selected = 0;
int USB4Selected = 0;
char selectfile[_MAX_SELECTION][_MAX_FILENAME]; // copy filename selection array
char selectpath[_MAX_SELECTION][_MAX_PATHNAME]; // copy pathname selection array
int k = 0; // filename selection array index
int m = 0; // pathname selection array index
int d = 0; // destination drive array index
int copying = 0; // copy in progress flag
char rxstring[_MAX_FILENAME]; // receive string buffer
char namebuff[_MAX_FILENAME] = ""; // new name buffer
int s = 0;
int RXdone = 0; // receive string done flag
int renaming = 0; // rename flag
char newname[_MAX_FILENAME]; // new name
char oldname[_MAX_PATHNAME]; // old name
char ext[5]; // filename extension
//unsigned long int StartCount, FileTime; // Core timer values
const char destdrv[3][3]; // destination drive directory
select_fn selection[_MAX_SELECTION]; //selected filenames
directory_list directory[4];
char dirfiles[_MAX_DIR_ENTRIES][_MAX_FILENAME]; // loaded directory files
extern BYTE CurrentPort;
int dest[3]; // destination drive
int currentDrv; // current drive selected

// FUNCTION PROTOTYPES
FRESULT read_contents (char *path);
void SYSTEMINIT(void);
void PININIT(void);
void BTINIT(void);
void SPIINIT(void);
void GetBTCommand(const char character);
void SendDataBuffer(const char *buffer, UINT32 size);
void WriteString(const char *string);
char *findfilename (char* path, int index);
void ClearSelection(void);
void ClearDestination(void);
char UART_RxString(const char character);
void PutChar(const char character);
unsigned int writeSPI1(unsigned int a);
FRESULT read_directory(char *path, int drv);

int main(void)
{
    int i;
    FATFS fatfs[_VOLUMES];
    FRESULT res;

	// INITIALIZE SYSTEM

	SYSTEMINIT();	// Initialize system
	PININIT();		// Initialize digital outputs
	SPIINIT();		// Initialize SPI1 channel
	BTINIT();		// Initialize UART2
	DBINIT();		// Initialize debug functions

	// INITIALIZE VARIABLES
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
    
    // INITIALIZE USB STACK
    USBInitialize(0);

	// ENABLE JTAG DEBUGGER
	mJTAGPortEnable(1);

    while(1) {

		DeviceNumber = 0;

        //USB stack process function
        USBTasks(0);
            
        //Detect hub attached
        while (!USBHostHubDeviceDetect(1)) {
            USBTasks(0);
        } 

        //Hub device is detected
        if(USBHostHubDeviceDetect(1)) {
            HubAttached = TRUE;
            event = EVENT_HUB_ATTACH;
                while(HubAttached == TRUE) {
                    USBTasks(0);
					if (MSDAttached) {

					// Device attached in Port 1
					if ((MSD1Mounted == 0) && (MSD1Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(0)) {
						volume = 0;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								writeSPI1(8);
								MSD1Mounted = 1;
								read_directory("0:", 0);
							} // if res
						}
					}

					// Device attached in Port 2
					if ((MSD2Mounted == 0) && (MSD2Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(1)) {
						volume = 1;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								writeSPI1(8);
								MSD2Mounted = 1;
								read_directory("1:", 1);
							} // if res
						}
					}

					// Device attached in Port 3
					if ((MSD3Mounted == 0) && (MSD3Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(2)) {
						volume = 2;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								writeSPI1(8);
								MSD3Mounted = 1;
								read_directory("2:", 2);
							} // if res
						}
					}

					// Device attached in Port 4
					if ((MSD4Mounted == 0) && (MSD4Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(3)) {
						volume = 3;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
								writeSPI1(8);
								MSD4Mounted = 1;
								read_directory("3:", 3);
							} // if res
						}
					}

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
				MSDAttached = 1;
			return TRUE;
			break;

		case EVENT_DETACH:
			// USB device is detached
			volume = CurrentPort - 1;
			switch (CurrentPort){
				case 1:
					MSD1Attached = 0;
					writeSPI1(6);
					res = f_mount(0, NULL);
					if (res == FR_OK) {
						MSD1Mounted = 0;
					}
					break;

				case 2:
					MSD2Attached = 0;
					writeSPI1(6);
					res = f_mount(1, NULL);
					if (res == FR_OK) {
						MSD2Mounted = 0;
					}
					break;

				case 3:
					MSD3Attached = 0;
					writeSPI1(6);
					res = f_mount(2, NULL);
					if (res == FR_OK) {
						MSD3Mounted = 0;
					}				
					break;

				case 4:
					MSD4Attached = 0;
					writeSPI1(6);
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
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
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;
}


/*-----------------------------------------------------------------------*/
/* EXCEPTION ERROR HANDLER							                     */
/*-----------------------------------------------------------------------*/

static unsigned int _excep_code;
static unsigned int _excep_addr;

// this function overrides the normal _weak_ generic handler
void _general_exception_handler(void)
{

	mPORTBClearBits(BIT_2);
	mPORTBSetBits(BIT_1);
    asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

    _excep_code = (_excep_code & 0x0000007C) >> 2;

    while (1) {
        // Examine _excep_code to identify the type of exceptiona
        // Examine _excep_addr to find the address that caused the exception
    }
}

/*-----------------------------------------------------------------------*/
/* FILE SYSTEM FUNCTIONS							                     */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Load directory contents							                     */
/*-----------------------------------------------------------------------*/
    
FRESULT read_directory (
    char *path,        /* Start node to be scanned (also used as work area) */
	int drv		// drive number
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i, j, a;
	const char *fn;
	char filepath[_MAX_PATHNAME];

	for (i=0; i<_MAX_DIR_ENTRIES; i++) {
		memset(directory[drv].dirfiles[i], 0, sizeof(directory[drv].dirfiles[i]));
	}
	
	j = 0;

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlenpgm(path);
        for (;;) {

			res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.' || fno.fattrib == 0x16) continue;             /* Ignore dot entry */

			fn = fno.fname;
       
			strncpy(directory[drv].dirfiles[j], fn, strlen(fn)+1);
			strncpy(directory[drv].dirpath[j], path, strlen(path)+1);
			strncat(directory[drv].dirpath[j], fn, strlen(fn)+3);

			j++;
		}
    }
		
    return res;
}

/*-----------------------------------------------------------------------*/
/* Copy and paste files from one drive to another                        */
/*-----------------------------------------------------------------------*/

FRESULT f_copy (
	const char sourcefile[_MAX_FILENAME],
	const char sourcename[_MAX_PATHNAME],
	const char destname1[_MAX_PATHNAME], 
	const char destname2[_MAX_PATHNAME], 
	const char destname3[_MAX_PATHNAME]
) {
    
    BYTE buffer[65536];   /* File copy buffer */
    FRESULT fr, fr1, fr2, fr3;          /* FatFs function common result code */
	FRESULT fropensrc;
    UINT br, bw;         /* File read/write count */
    static FIL fsrc, fdst1, fdst2, fdst3;
	unsigned long starttime, readtime, writetime;
	int i;
	i = 0;
//	starttime = ReadCoreTimer();
	copying = 1;

            fropensrc = f_open(&fsrc, sourcename, FA_READ | FA_OPEN_ALWAYS);
			if (fropensrc) {
				copying = 0; 
				return (int) fr;
			}	
			            
			while (strlen(directory[dest[0]].dirfiles[i]) != 0)  {
				if (!memcmp(sourcefile, directory[dest[0]].dirfiles[i],strlen(sourcefile))) {
					i++;
				}
				else {
			//		DBPRINTF("o ");
					break;
				}
			}
            fr1 = f_open(&fdst1, destname1, FA_WRITE | FA_CREATE_ALWAYS);

			if (destname2 != "") {

				while (strlen(directory[dest[1]].dirfiles[i]) != 0)  {
					if (!memcmp(sourcefile, directory[dest[1]].dirfiles[i],strlen(sourcefile))) {
						i++;
					}
					else {
						DBPRINTF("o ");
						break;
					}
				}
				fr2 = f_open(&fdst2, destname2, FA_WRITE | FA_CREATE_ALWAYS);
				if (destname3 != "") {

					while (strlen(directory[dest[2]].dirfiles[i]) != 0)  {
						if (!memcmp(sourcefile, directory[dest[2]].dirfiles[i],strlen(sourcefile))) {
							i++;
						}
						else {
							DBPRINTF("o ");
							break;
						}
					}
					fr3 = f_open(&fdst3, destname3, FA_WRITE | FA_CREATE_ALWAYS);
				}
			}
			
			if (fr1 && fr2 && fr3) {
				f_close(&fsrc);
				copying = 0;
				return (int) fr1;
			}

//		readtime = FILETIME_MS((ReadCoreTimer()-starttime));
//		DBPRINTF(" %lu ", readtime);
    /* Copy source to destination */
    for (;;) {
//		starttime = ReadCoreTimer();
        fr = f_read(&fsrc, buffer, sizeof buffer, &br);  /* Read a chunk of source file */
        if (fr || br == 0) break; /* error or eof */
//		readtime = FILETIME_MS((ReadCoreTimer()-starttime));
//		DBPRINTF(" %lu ", readtime);
//		starttime = ReadCoreTimer();
		fr = f_write(&fdst1, buffer, br, &bw);            /* Write it to the destination file */
		if ((fr || bw < br) && fr2 && fr3) break;
//		writetime = FILETIME_MS((ReadCoreTimer()-starttime));
//		DBPRINTF(" %lu ", writetime);
		f_sync(&fdst1);
		if (!fr2 && destname2 != "") {
//			starttime = ReadCoreTimer();
			fr = f_write(&fdst2, buffer, br, &bw);            /* Write it to the destination file */
        	if ((fr || bw < br) && fr3) break; /* error or disk full */	
//			writetime = FILETIME_MS((ReadCoreTimer()-starttime));
//			DBPRINTF(" %lu ", writetime);
			f_sync(&fdst2);		
				if (!fr3 && destname3 != "") {
//					starttime = ReadCoreTimer();
			        fr = f_write(&fdst3, buffer, br, &bw);            /* Write it to the destination file */
			        if (fr || bw < br) break; /* error or disk full */
//					writetime = FILETIME_MS((ReadCoreTimer()-starttime));
//					DBPRINTF(" %lu\n", writetime);
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
			return 0; /* no extension */
		} 
		else {
			return ext;
		}

}

/*-----------------------------------------------------------------------*/
/* INITIALIZATION FUNCTIONS							                     */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* System initialize								                     */
/*-----------------------------------------------------------------------*/

void SYSTEMINIT (void) {

	SYSTEMConfigPerformance(80000000L); 
	
	unsigned int cache_status;
	
	mBMXDisableDRMWaitState();
	mCheConfigure(3);
	cache_status = mCheGetCon();
	cache_status |= CHE_CONF_PF_ALL;
	mCheConfigure(cache_status);
	CheKseg0CacheOn();
	
}

/*-----------------------------------------------------------------------*/
/* Digital outputs initialize						                     */
/*-----------------------------------------------------------------------*/

void PININIT(void) {

// RB0 = 72 - GREEN
// RB1 = 70 - RED
// RB2 = 68 - YELLOW

	mPORTDClearBits(BIT_0);
    mPORTBClearBits(BIT_0); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_0);	// Make RB0, RB1, RB2 as output.
    mPORTBClearBits(BIT_1); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_1);	// Make RB0, RB1, RB2 as output.
    mPORTBClearBits(BIT_2); 		// Turn off RB0, RB1, RB2 bits on startup.
    mPORTBSetPinsDigitalOut(BIT_2);	// Make RB0, RB1, RB2 as output.

}

/*-----------------------------------------------------------------------*/
/* SPI1 channel initialize							                     */
/*-----------------------------------------------------------------------*/

void SPIINIT(void) {

    //SPI setup
    int rData = SPI1BUF;    //Clears receive buffer
    IFS0CLR = 0x03800000;   //Clears any existing event (rx / tx/ fault interrupt)
    SPI1STATCLR = 0x40;      //Clears overflow
    //Enables the SPI channel (channel, master mode enable | use 8 bit mode | turn on, clock divider)
    SpiChnOpen(1, SPI_CON_MSTEN | SPI_CON_MODE8 | SPI_CON_ON, 10); // SCK = Fpb/4 = 80/4 = 8 MHz

}

/*-----------------------------------------------------------------------*/
/* UART2 initialize									                     */
/*-----------------------------------------------------------------------*/

void BTINIT (void) {

	 // Configure UART2 module, set baud rate, turn on UART, etc.
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


/*-----------------------------------------------------------------------*/
/* USB selectiton initialization						                 */
/*-----------------------------------------------------------------------*/

void InitSelected (void) {
	USB1Selected = 0;
	USB2Selected = 0;
	USB3Selected = 0;
	USB4Selected = 0;
}


/*-----------------------------------------------------------------------*/
/* File selection initialization					                     */
/*-----------------------------------------------------------------------*/

void ClearSelection(void) {

	int j, l, c;
	for (j = 0; j < k; j++) {
		memset(selection[j].root, 0, sizeof(selection[j].root));
		memset(selection[j].selectfile, 0, sizeof(selection[j].selectfile));
		memset(selection[j].selectpath, 0, sizeof(selection[j].selectpath));
	}
	
	for (c = 0; c < d; c++) {
		memset(destdrv[c], 0, sizeof(destdrv[c]));;
		memset(dest[c],0,sizeof(dest[c]));
	}
	d = 0;
	k = 0;
	m = 0;
}


/*-----------------------------------------------------------------------*/
/* Send Files over Bluetooth						                     */
/*-----------------------------------------------------------------------*/

void SendFiles(int drv) {

	int i;
	
	while (strlen(directory[drv].dirpath[i]) != 0 && i < _MAX_DIR_ENTRIES) {
		
			SendDataBuffer(directory[drv].dirpath[i], strlenpgm(directory[drv].dirpath[i]));
     		SendDataBuffer("/", strlen("/"));
			i++;
	}
	SendDataBuffer(" ", strlen(" "));
}


/*-----------------------------------------------------------------------*/
/* File selection initialization					                     */
/*-----------------------------------------------------------------------*/

void ReloadDirectories(void) {

		read_directory("0:", 0);
		read_directory("1:", 1);
		read_directory("2:", 2);
		read_directory("3:", 3);	

}

/*-----------------------------------------------------------------------*/
/* INTERRUPT EVENT HANDLER							                     */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Bluetooth command interrupt event handler		                     */
/*-----------------------------------------------------------------------*/

void GetBTCommand(const char character) {


		unsigned long int StartCount, FileTime; // Core timer values
		FRESULT res;
		int n, b;
		int index = 0;

		DBPRINTF("%c", character);


		// GET DIRECTORY

		// MSD1 Selected
		if (character == 'E' && renaming == 0) {
			InitSelected();
				if (MSD1Attached) {
				writeSPI1(4);
				USB1Selected = 1;
				currentDrv = 0;
				strncpy(root, "0:", 3);
				SendFiles(currentDrv);
			}
			else if (!MSD1Attached) {
				writeSPI1(6);
			}
		}

		// MSD2 Selected
		else if (character == 'F' && renaming == 0) {
			InitSelected();
			if (MSD2Attached) {
				writeSPI1(4);
				USB2Selected = 1;
				currentDrv = 1;
				strncpy(root, "1:", 3);
				SendFiles(currentDrv);
			}
			else if (!MSD2Attached) {
			writeSPI1(6);
			}
		}

		// MSD3 Selected
		else if (character == 'G' && renaming == 0) {
			InitSelected();
			if (MSD3Attached) {
				writeSPI1(4);
				USB3Selected = 1;
				currentDrv = 2;
				strncpy(root, "2:", 3);
				SendFiles(currentDrv);
			}
			else if (!MSD3Attached) {
				writeSPI1(6);
			}
		}	

		else if (character == 'H' && renaming == 0) { // USB 4 Selected
			InitSelected();
 			if (MSD4Attached) {
				writeSPI1(4);
				USB4Selected = 1;
				currentDrv = 3;
				strncpy(root, "3:", 3);
				SendFiles(currentDrv);
			}
			else if (!MSD4Attached) {
				writeSPI1(6);
			}
		}

		// STORE SELECTED FILES

		// Find the filename to copy
		else if (((character>='0' && character<='9') || character == '-') && renaming == 0) {

			UART_RxString(character);
			if (RXdone) {			
				index = atoi(rxstring);
				memset(rxstring, 0, sizeof(rxstring));
				strncpy(selectfile[k], directory[currentDrv].dirfiles[index-1], strlen(directory[currentDrv].dirfiles[index-1])+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], selectfile[k], strlen(selectfile[k])+3);
				strncpy(selection[k].root, root, 3);
				strncpy(selection[k].selectfile, selectfile[k], strlen(selectfile[k])+1);
				strncpy(selection[k].selectpath, selectpath[m], strlen(selectpath[m])+1);
				k++;
				m++;
			RXdone = 0;
			}
		}

		// CHOOSE DESTINATION

		// MSD1 chosen
		else if (character == 'm' && renaming == 0) {
			strncpy(destdrv[d], "0:", 3);
			dest[d] = 0;
			d++;
		}

		// MSD2 chosen
		else if (character == 'n' && renaming == 0) {
			strncpy(destdrv[d], "1:", 3);
			dest[d] = 1;
			d++;
		}

		// MSD3 chosen
		else if (character == 'o' && renaming == 0) {
			strncpy(destdrv[d], "2:", 3);
			dest[d] = 2;
			d++;
		}

		// MSD4 chosen
		else if (character == 'p' && renaming == 0) {
			strncpy(destdrv[d], "3:", 3);
			dest[d] = 3;
			d++;
		}

		// PASTE FILES

		else if (character == 'i' && renaming == 0) {
		writeSPI1(10);
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
			for (n = 0; n < k; n++) {
			    mPORTBClearBits(BIT_0 | BIT_1); 
				for (b=0; b<d; b++) {
					strncpy(destpath[b], destdrv[b], 3);
					strncat(destpath[b], selection[n].selectfile, strlen(selection[n].selectfile)+3);
				}
			
		//	unsigned long endTime = FILETIME_MS((ReadCoreTimer()-StartCount));
		//	DBPRINTF("s %lu ", endTime);
				if (d==1) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], "", "");
				}
				else if (d==2) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], destpath[1], "");
				}
				else if (d==3) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], destpath[1], destpath[2]);
				}
				if (res == FR_OK) {	
				//	DBPRINTF("d");
					mPORTBSetBits(BIT_0);
				}
				else {
				//	DBPRINTF("f");
					mPORTBSetBits(BIT_1);
				}
			}
//		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		FileTime = FILETIME_MS((ReadCoreTimer()-StartCount));
		DBPRINTF("t %lu\n", FileTime);
		mPORTBClearBits(BIT_2);
		writeSPI1(12);
		ReloadDirectories();
		ClearSelection();
		}


		// DELETE FILES

		else if (character == 'q' && renaming == 0) { // Delete files
		writeSPI1(10);
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
			res = f_unlink(selection[n].selectpath);
			if (res == FR_OK) {
				mPORTBSetBits(BIT_0);
			//	DBPRINTF("d");
			}
			else {
				mPORTBSetBits(BIT_1);
			//	DBPRINTF("f");
			}
		}
//		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		FileTime = FILETIME_MS((ReadCoreTimer()-StartCount));
		DBPRINTF("t %lu\n", FileTime);
		mPORTBClearBits(BIT_2);
		writeSPI1(12);
		ReloadDirectories();
		ClearSelection();
		}

		// MOVE FILES

		else if (character == 'j' && renaming == 0) {
		writeSPI1(10);
		mPORTBSetBits(BIT_2);
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
				mPORTBClearBits(BIT_0 | BIT_1); 
				for (b=0; b<d; b++) {
					strncpy(destpath[b], destdrv[b], 3);
					strncat(destpath[b], selection[n].selectfile, strlen(selection[n].selectfile)+3);
				}
				if (d==1) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], "", "");
				}
				else if (d==2) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], destpath[1], "");
				}
				else if (d==3) {
				res = f_copy(selection[n].selectfile, selection[n].selectpath, destpath[0], destpath[1], destpath[2]);
				}
				if (res == FR_OK) {
				//	DBPRINTF("d");
					res = f_unlink(selection[n].selectpath);
					if (res == FR_OK) {
						mPORTBSetBits(BIT_0);
				//		DBPRINTF("d");
					}
					else {
						mPORTBSetBits(BIT_1);
				//		DBPRINTF("f");
					}
				}
				else {
					mPORTBSetBits(BIT_1);
					DBPRINTF("f");
				}
			}
//		FileTime = (ReadCoreTimer()-StartCount)/40000L;
		FileTime = FILETIME_MS((ReadCoreTimer()-StartCount));
		DBPRINTF("t %lu\n", FileTime);
		mPORTBClearBits(BIT_2);
		writeSPI1(12);
		ReloadDirectories();
		ClearSelection();
		}

		// RENAME

		// Choose old filename
		else if (character == 'r' && renaming == 0) {
		renaming = 1;
		strncpy(oldname, root, strlen(root)+1);
		strncat(oldname, selection[0].selectfile, strlen(selection[0].selectfile)+3);
		strcpy(ext, getextension(selection[0].selectfile));
		}

		// Read new filename
		else if (character != ' ' && renaming == 1) {
			writeSPI1(10);
			mPORTBSetBits(BIT_2);
			UART_RxString(character);
			if (RXdone) {
				strncpy(newname, rxstring, strlen(rxstring));
				strncat(newname, ext, strlen(ext)+1);
				memset(rxstring, 0, sizeof(rxstring));
				memset(ext, 0, sizeof(ext));
				StartCount = ReadCoreTimer();
				res = f_rename(oldname, newname);
				if (res == FR_OK) {
					mPORTBSetBits(BIT_0);
				//	DBPRINTF("d\n");
				}
				else {
					mPORTBSetBits(BIT_1);
				//	DBPRINTF("f\n");
					}
	//		FileTime = (ReadCoreTimer()-StartCount)/40000L;
			FileTime = FILETIME_MS((ReadCoreTimer()-StartCount));
			DBPRINTF("t %lu\n", FileTime);
			mPORTBClearBits(BIT_2);
			writeSPI1(12);
			RXdone = 0;
			renaming = 0;
			memset(oldname, 0, sizeof(oldname));
			memset(newname, 0, sizeof(newname));
			ReloadDirectories();
			ClearSelection();
			}
		}

		else if (character == 's' && renaming == 0) { // RENAME CANCELLED
		renaming = 0;
		}

		// CLEAR SELECTION

		else if (character == 'k' && renaming == 0) {

			ReloadDirectories();
			InitSelected();
			ClearSelection();

		}		

		else if (character == 'l' && renaming == 0) {

		writeSPI1(14);
		SendFiles(0);

		}

		DelayMs(10);
		mPORTBClearBits(BIT_0 | BIT_1); 
}
/*
void RenameFile(void) {

			renaming = 1;
			UART_RxString(character);
			if (RXdone) {
				strncpy(newname, rxstring, strlen(rxstring));
				strncat(newname, ext, strlen(ext)+1);
				memset(rxstring, 0, sizeof(rxstring));
				memset(ext, 0, sizeof(ext));
				StartCount = ReadCoreTimer();
				res = f_rename(oldname, newname);
				if (res == FR_OK) {
					mPORTBSetBits(BIT_0);
				//	DBPRINTF("d\n");
				}
				else {
					mPORTBSetBits(BIT_1);
				//	DBPRINTF("f\n");
					}
	//		FileTime = (ReadCoreTimer()-StartCount)/40000L;
			FileTime = FILETIME_MS((ReadCoreTimer()-StartCount));
			DBPRINTF("t %lu\n", FileTime);
			mPORTBClearBits(BIT_2);
			writeSPI1(12);
			RXdone = 0;
			renaming = 0;
			memset(oldname, 0, sizeof(oldname));
			memset(newname, 0, sizeof(newname));
			ReloadDirectories();
			ClearSelection();
			}
		}

}
*/
/*-----------------------------------------------------------------------*/
/* SPI AND UART FUNCTIONS							                     */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Write data to SPI1 channel						                     */
/*-----------------------------------------------------------------------*/

unsigned int writeSPI1(unsigned int a) {

         putcSPI1(a);                 //Sends hex data unsigned int data to slave
         int receive = SPI1BUF;            //Read SP1BUF (dummy read)
         SPI1BUF = 0x0;                  //Write SP1BUF- sets Tx flag, if not done read will not clock
         return getcSPI1();            //Generates clock and reads SDO
}


// *****************************************************************************
// Send string from UART2
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

/*-----------------------------------------------------------------------*/
/* Receive string from UART2						                     */
/*-----------------------------------------------------------------------*/
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

/*-----------------------------------------------------------------------*/
/* UART2 interrupt handler, set at priority level 3		                 */
/*-----------------------------------------------------------------------*/

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
