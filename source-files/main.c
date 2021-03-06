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
    #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
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
    #pragma config DEBUG    = ON            // Background Debugger Enable

#else


    #error Cannot define configuration bits.

#endif

#define UART_MODULE_ID_1 UART1  // PIM is connected to Explorer through UART1 module
#define UART_MODULE_ID_2 UART2 // PIM is connected to Explorer through UART2 module
#define DESIRED_BAUDRATE (9600) //The desired BaudRate
#define _MAX_FILENAME 13
#define _MAX_PATHNAME 15
    
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
const char destpath[_MAX_PATHNAME]; // destination pathname
const char destipath[10][_MAX_PATHNAME]; // destination pathname
int USB1Selected = 0;
int USB2Selected = 0;
int USB3Selected = 0;
int USB4Selected = 0;
char selectfile[10][_MAX_FILENAME]; // copy filename selection array
char selectpath[10][_MAX_PATHNAME]; // copy pathname selection array
int k = 0; // filename selection array index
int m = 0; // pathname selection array index
int copying = 0; // copy in progress flag
char rxstring[_MAX_FILENAME];
int s = 0;
char namebuff[_MAX_FILENAME] = "";
int RXdone = 0; // receive string done flag
int RXnewname = 0; // receive newname string done flag
int renaming = 0; // rename flag
char newname[_MAX_FILENAME]; // copy filename selection array
char oldname[_MAX_PATHNAME]; // copy pathname selection array
char ext[5]; // filename extension
unsigned long int StartCount, FileTime;

// FUNCTION PROTOTYPES
void PassDirectory(const char choice);
FRESULT read_contents (char *path);
void BTINIT(void);
void GetBTCommand(const char character);
void CheckStatus(const char choice);
void BTfunctions(const char select);
UINT32 GetMenuChoice(void);
void SendDataBuffer(const char *buffer, UINT32 size);
extern BYTE CurrentPort;
void TestPutFile(void);
void WriteString(const char *string);
char *findfilename (char* path, int index);
void WriteString(const char *string);
void PutCharacter(const char character);
void PutInteger(unsigned int integer);
FRESULT f_copy (const char sourcename[_MAX_PATHNAME], const char destname[_MAX_PATHNAME]);
void ClearSelection(void);
char UART_RxString(const char character);

// CONSTANTS
#define USB_MAX_DEVICES 5
#define MAX_ALLOWED_CURRENT	(500)



int main(void)
{
    int i;
    FATFS fatfs[_VOLUMES];
    FRESULT res;
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
							//	DBPRINTF("%x: USB1 Mounted\n", volume);
								PutCharacter('A');
							} // if res
						MSD1Mounted = 1;
					//	DBPRINTF("0:/%s\n", findfilename("0:", 1));
						}
						}

					 else if ((MSD2Mounted == 0) && (MSD2Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(1)) {
					//	DBPRINTF("MSD Device Attached in Port 2\n");
						volume = 1;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
							//	DBPRINTF("%x: USB2 Mounted\n", volume);
								PutCharacter('B');
							} // if res
						MSD2Mounted = 1;
						}
						}

					 else if ((MSD3Mounted == 0) && (MSD3Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(2)) {
					//	DBPRINTF("MSD Device Attached in Port 3\n");
						volume = 2;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
							//	DBPRINTF("%x: USB3 Mounted\n", volume);
								PutCharacter('C');
							} // if res
						MSD3Mounted = 1;
					//	DBPRINTF("2:/%s\n", findfilename("2:", 1));
						}
						}

					 else if ((MSD4Mounted == 0) && (MSD4Attached == 1)) {
						if (USBHostMSDSCSIMediaDetect(3)) {
					//	DBPRINTF("MSD Device Attached in Port 4\n");
						volume = 3;
						res = f_mount(volume, &fatfs[volume]);
							if (res == FR_OK) {
							//	DBPRINTF("%x: USB4 Mounted\n", volume);
								PutCharacter('D');
							} // if res
						MSD4Mounted = 1;
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
                DBPRINTF( "\n***** USB Error - device requires too much current *****\n" );
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
			PutCharacter('v');
            return TRUE;
            break;

		case EVENT_ATTACH:
			// USB device is detached
		//	DBPRINTF("MSD Device at %x Attached\n", CurrentPort);
			switch (CurrentPort){
				case 1:
					MSD1Attached = 1;
					PutCharacter('A');
					break;

				case 2:
					MSD2Attached = 1;
					PutCharacter('B');
					break;

				case 3:
					MSD3Attached = 1;
					PutCharacter('C');
					break;

				case 4:
					MSD4Attached = 1;
					PutCharacter('D');
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
					PutCharacter('w');
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
					//	DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD1Mounted = 0;
					}
					break;

				case 2:
					MSD2Attached = 0;
					PutCharacter('x');
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
					//	DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD2Mounted = 0;
					}
					break;

				case 3:
					MSD3Attached = 0;
					PutCharacter('y');
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
					//	DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD3Mounted = 0;
					}				
					break;

				case 4:
					MSD4Attached = 0;
					PutCharacter('z');
					res = f_mount(volume, NULL);
					if (res == FR_OK) {
					//	DBPRINTF("%x: MSD Device Unmounted\n", volume);
						MSD4Mounted = 0;
					}
					break;

				default:
					break;					

			}

				deviceAddress[CurrentPort] = 0;

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
	char dirfiles[20][_MAX_FILENAME];
	
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

char *findfilename (
    char* path,        /* Start node to be scanned (also used as work area) */
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
	char filenames[20][_MAX_FILENAME];   /* This function assumes non-Unicode configuration */	

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlenpgm(path);
        for (;;) {

			res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */

			fn = fno.fname;

		//	filenames[j] = (char *)malloc(strlenpgm(fn)+1);   
		//	strcpy(filenames[j], fn);
			strncpy(filenames[j], fn, strlen(fn)+1);

		//	DBPRINTF("%s/%s, %i, %i\n", path, filenames[j], j, index);
			j++;

		}
		//fi = filenames[index];
		strncpy(fi, filenames[index], strlen(filenames[index])+1);
	//	DBPRINTF("%i %s/%s\n", index, path, filenames[index]);
    }
	return fi;
}

/*-----------------------------------------------------------------------*/
/* Copy and paste files from one drive to another                        */
/*-----------------------------------------------------------------------*/

FRESULT f_copy (
	const char sourcename[_MAX_PATHNAME],
	const char destname[_MAX_PATHNAME]
) {
    
    BYTE buffer[8192];   /* File copy buffer */
    FRESULT fr;          /* FatFs function common result code */
	FRESULT fropensrc;
    UINT br, bw;         /* File read/write count */
    static FIL fsrc, fdst;

	copying = 1;
	DBPRINTF("s");

            fropensrc = f_open(&fsrc, sourcename, FA_READ | FA_OPEN_ALWAYS);
			if (fropensrc) {
				copying = 0; 
				return (int) fr;
			}	
            
            fr = f_open(&fdst, destname, FA_WRITE | FA_CREATE_ALWAYS);
            if (fr) {
				if (fropensrc) {
				f_close(&fsrc); 
				}
				copying = 0;
				return (int) fr;
			}

    /* Copy source to destination */
    for (;;) {
        fr = f_read(&fsrc, buffer, sizeof buffer, &br);  /* Read a chunk of source file */
        if (fr || br == 0) break; /* error or eof */
        fr = f_write(&fdst, buffer, br, &bw);            /* Write it to the destination file */
        if (fr || bw < br) break; /* error or disk full */
		f_sync(&fdst);
    }

    /* Close open files */
    f_close(&fsrc);
    f_close(&fdst);    

//	DBPRINTF("d");
	copying = 0;
    return fr;
    
}

char *getextension(char *fn) {

	char *ext;

		ext = strrchr(fn, '.');
		if (!ext) {
			return 0;
		    /* no extension */
		} else {
		    printf("extension is %s\n", ext + 1);
			return ext;
		}

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
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    //mJTAGPortEnable(DEBUG_JTAGPORT_OFF);

    mPORTDClearBits(BIT_0); 		// Turn off RA7 on startup.
    mPORTDSetPinsDigitalOut(BIT_0);	// Make RA7 as output.

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

	int j, l;
	for (j = 0; j < k; j++) {
		strcpy(selectfile[j], "");
	}
	for (l = 0; l < m; l++) {
		strcpy(selectpath[m], "");
	}
	k = 0;
	m = 0;
}

void GetBTCommand(const char character) {

		FRESULT res;
		int n;
		int index = 0;

		DBPRINTF("%c", character);

		// GET DIRECTORY

		if (character == 'E' && RXnewname == 0) { // USB 1 Selected
			InitSelected();
			ClearSelection();
			if (MSD1Attached) {
			USB1Selected = 1;
			strncpy(root, "0:", 3);
			read_contents(root);
			PutCharacter('A');
			}
			else {
			PutCharacter('E');
			}
		}

		else if (character == 'F' && RXnewname == 0) { // USB 2 Selected
			InitSelected();
			ClearSelection();
			if (MSD2Attached) {
			USB2Selected = 1;
			strncpy(root, "1:", 3);
			read_contents(root);
			PutCharacter('B');
			}
			else {
			PutCharacter('F');
			}
		}

		else if (character == 'G' && RXnewname == 0) { // USB 3 Selected
			InitSelected();
			ClearSelection();
			if (MSD3Attached) {
			USB3Selected = 1;
			strncpy(root, "2:", 3);
			read_contents(root);
			PutCharacter('C');
			}
			else {
			PutCharacter('G');
			}
		}	

		else if (character == 'H' && RXnewname == 0) { // USB 4 Selected
			InitSelected();
			ClearSelection();
			if (MSD4Attached) {
			USB4Selected = 1;
			strncpy(root, "3:", 3);
			read_contents(root);
//			PassDirectory(character);
			PutCharacter('D');
			}
			else {
			PutCharacter('H');
			}
		}

		// STORE SELECTED FILES

		else if (((character>='0' && character<='9') || character == '-') && renaming == 0) { // Find the filename to copy
		//	DBPRINTF("%c\n", character);
			UART_RxString(character);
			if (RXdone) {			
			//	index = rxstring - '0';
				index = atoi(rxstring);
				memset(rxstring, 0, sizeof(rxstring));
				strcpy(fn, findfilename(root, index-1));
				strncpy(selectfile[k], fn, strlen(fn)+1);
				strncpy(selectpath[m], root, strlen(root)+1);
				strncat(selectpath[m], fn, strlen(fn)+3);
				k++;
				m++;
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
		// PASTE FILES

		else if (character == 'm' && RXnewname == 0) { // Paste to USB1
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
		while (!copying) {
			strncpy(destpath, "0:", 3);
			strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
		//	DBPRINTF("%s\n", destpath);
			res = f_copy(selectpath[n], destpath);
			if (res == FR_OK) {
				DBPRINTF("d");
				PutCharacter('m');
			//	PutInteger('1');
			}
			else {
				DBPRINTF("f");
				PutCharacter('q');
			//	PutInteger('2');
			}
			copying = 0;
			break;
		}
		}
		FileTime = (ReadCoreTimer()-StartCount)/80000;
		DBPRINTF("t %lu\n", FileTime);
		PutCharacter('u');
		}

		else if (character == 'n' && RXnewname == 0) { // Paste to USB2
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
		while (!copying) {
			strncpy(destpath, "1:", 3);
			strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
		//	DBPRINTF("%s\n", destpath);
			res = f_copy(selectpath[n], destpath);
			if (res == FR_OK) {
				DBPRINTF("d");
				PutCharacter('m');
			//	PutInteger('1');
			}
			else {
				DBPRINTF("f");
				PutCharacter('q');
			//	PutInteger('2');
			}
			break;
		}
		}
		FileTime = (ReadCoreTimer()-StartCount)/80000;
		DBPRINTF("t %lu\n", FileTime);
		PutCharacter('u');
		}


		else if (character == 'o' && RXnewname == 0) { // Paste to USB3
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
		while (!copying) {
			strncpy(destpath, "2:", 3);
			strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
		//	DBPRINTF("%s\n", destpath);
			res = f_copy(selectpath[n], destpath);
			if (res == FR_OK) {
				DBPRINTF("d");
				PutCharacter('m');
			}
			else {
				DBPRINTF("f");
				PutCharacter('q');
			}
			break;
		}
		}
		FileTime = (ReadCoreTimer()-StartCount)/80000;
		DBPRINTF("t %lu\n", FileTime);
		PutCharacter('u');
		}

		else if (character == 'p' && RXnewname == 0) { // Paste to USB4
		StartCount = ReadCoreTimer();
		for (n = 0; n < k; n++) {
		while (!copying) {
			strncpy(destpath, "3:", 3);
			strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
		//	DBPRINTF("%s\n", destpath);
			res = f_copy(selectpath[n], destpath);
			if (res == FR_OK) {
				DBPRINTF("d");
				PutCharacter('m');
			}
			else {
				DBPRINTF("f");
				PutCharacter('q');
			}
			break;
		}
		}
		FileTime = (ReadCoreTimer()-StartCount)/80000;
		DBPRINTF("t %lu\n", FileTime);
		PutCharacter('u');
		}

		// DELETE FILES

		else if (character == 'q' && RXnewname == 0) { // delete files
		for (n = 0; n < k; n++) {
			res = f_unlink(selectpath[n]);
			if (res == FR_OK) {
				DBPRINTF("d");
				PutCharacter('n');
			}
			else {
				DBPRINTF("f");
				PutCharacter('r');
			}
		}
		DBPRINTF("t\n");
		PutCharacter('u');
		ClearSelection();
		}

		// MOVE FILES

		else if (character == 'i' && RXnewname == 0) { // Move to USB1
			for (n = 0; n < k; n++) {
				while (!copying) {
					strncpy(destpath, "0:", 3);
					strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
				//	DBPRINTF("%s\n", destpath);
					res = f_copy(selectpath[n], destpath);
					if (res == FR_OK) {
						DBPRINTF("d");
						res = f_unlink(selectpath[n]);
						if (res == FR_OK) {
							DBPRINTF("d");
						}
						else {
							DBPRINTF("f");
						}
						PutCharacter('p');
					}
					else {
						DBPRINTF("f");
						PutCharacter('t');
					}
					break;
				}
			}
			DBPRINTF("t\n");
			PutCharacter('u');
			ClearSelection();
		}

		else if (character == 'j' && RXnewname == 0) { // Move to USB2
			for (n = 0; n < k; n++) {
				while (!copying) {
					strncpy(destpath, "1:", 3);
					strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
				//	DBPRINTF("%s\n", destpath);
					res = f_copy(selectpath[n], destpath);
					if (res == FR_OK) {
						DBPRINTF("d");
						res = f_unlink(selectpath[n]);
						if (res == FR_OK) {
							DBPRINTF("d");
						}
						else {
							DBPRINTF("f");
						}
						PutCharacter('p');
					}
					else {
						DBPRINTF("f");
						PutCharacter('t');
					}
					break;
				}
			}
			DBPRINTF("t\n");
			PutCharacter('u');
			ClearSelection();
		}

		else if (character == 'k' && RXnewname == 0) { // Move to USB3
			for (n = 0; n < k; n++) {
				while (!copying) {
					strncpy(destpath, "2:", 3);
					strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
				//	DBPRINTF("%s\n", destpath);
					res = f_copy(selectpath[n], destpath);
					if (res == FR_OK) {
						DBPRINTF("d");
						res = f_unlink(selectpath[n]);
						if (res == FR_OK) {
							DBPRINTF("d");
						}
						else {
							DBPRINTF("f");
						}
						PutCharacter('p');
					}
					else {
						DBPRINTF("f");
						PutCharacter('t');
					}
					break;
				}
			}
			DBPRINTF("t\n");
			PutCharacter('u');
			ClearSelection();
		}

		else if (character == 'l' && RXnewname == 0) { // Move to USB4
			for (n = 0; n < k; n++) {
				while (!copying) {
					strncpy(destpath, "3:", 3);
					strncat(destpath, selectfile[n], strlen(selectfile[n])+3);
				//	DBPRINTF("%s\n", destpath);
					res = f_copy(selectpath[n], destpath);
					if (res == FR_OK) {
						DBPRINTF("d");
						res = f_unlink(selectpath[n]);
						if (res == FR_OK) {
							DBPRINTF("d");
						}
						else {
							DBPRINTF("f");
						}
						PutCharacter('p');
					}
					else {
						DBPRINTF("f");
						PutCharacter('t');
					}
					break;
				}
			}
			DBPRINTF("t\n");
			PutCharacter('u');
			ClearSelection();
		}

		// RENAME

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
				strcpy(fn, findfilename(root, index-1));
				strcpy(ext, getextension(fn));
				DBPRINTF("%s", ext);
				strncpy(oldname, root, 3);
				strncat(oldname, fn, strlen(fn)+3);
			//	DBPRINTF("%s, %s\n", fn, oldname);
			RXdone = 0;
			RXnewname = 1;
			}
		}

		else if (character != ' ' && renaming == 1 && RXnewname == 1) {
			UART_RxString(character);
			if (RXdone) {
				strncpy(newname, rxstring, strlen(rxstring));
				strncat(newname, ext, strlen(ext)+1);
				DBPRINTF("%s, %s", newname, ext);
				memset(rxstring, 0, sizeof(rxstring));
				memset(ext, 0, sizeof(ext));
				res = f_rename(oldname, newname);
				if (res == FR_OK) {
					DBPRINTF("d\n");
					PutCharacter('o');
				}
				else {
					DBPRINTF("f\n");
					PutCharacter('s');
				}
			PutCharacter('u');
			RXdone = 0;
			RXnewname = 0;
			renaming = 0;
			memset(oldname, 0, sizeof(oldname));
			memset(newname, 0, sizeof(newname));
			}
		}

		DelayMs(10);
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

void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART_MODULE_ID_1))
            ;

        UARTSendDataByte(UART_MODULE_ID_1, character);


        while(!UARTTransmissionHasCompleted(UART_MODULE_ID_1))
            ;
}

void PutInteger(unsigned int integer)
{
        while(!UARTTransmitterIsReady(UART_MODULE_ID_2))
            ;

        UARTSendDataByte(UART_MODULE_ID_2, integer);


        while(!UARTTransmissionHasCompleted(UART_MODULE_ID_2))
            ;
}

char UART_RxString(const char character){
	
	RXdone = 0;
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

// UART 1 interrupt handler, set at priority level 2

void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_1)))
	{
             // Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART_MODULE_ID_1));
	}

	// We don't care about TX interrupt
	if (INTGetFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_1)))
	{
            INTClearFlag(INT_SOURCE_UART_TX(UART_MODULE_ID_1));
	}
}

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
