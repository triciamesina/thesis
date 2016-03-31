/******************************************************************************
 *
 *              USB MEDIA INTERFACE
 *
 ******************************************************************************
 * FileName:        fatfs_usb.c
 * Dependencies:    See include below
 * Processor:       PIC32
 * Compiler:        C32 V1.03
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 * It is
 * your responsibility to ensure that your application meets with your
 * specifications.Further, the implementation and use of the FAT file system
 * accompanying this code, SD card specifications, MMC card specifications
 * and other third party tools may require a license from various entities,
 * including, but not limited to Microsoft® Corporation, SD Card Association
 * and MMCA.  It is your responsibility to obtain more information regarding
 * any applicable licensing obligations.
 *
 * CODE AND INFORMATION ACCOMPANYING THIS MESSAGE IS PROVIDED “AS IS.
 * MICROCHIP AND ITS LICENSORS MAKES NO REPRESENTATION OR WARRANTIES OF ANY
 * KIND WHETHER EXPRESS OR IMPLIED, WRITTEN OR ORAL, STATUTORY OR OTHERWISE,
 * RELATED TO THE INFORMATION PROVIDED TO YOU, INCLUDING BUT NOT LIMITED TO
 * ITS CONDITION, QUALITY, PERFORMANCE, MERCHANTABILITY, NON-INFRINGEMENT,
 * OR FITNESS FOR PURPOSE.  MICROCHIP AND ITS LICENSORS ARE NOT LIABLE, UNDER
 * ANY CIRCUMSTANCES, FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES,
 * FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************
 *
 * 11/17/08	V1.00 D.Wenn		Placeholder for USB drive translation
 *
 *****************************************************************************/

 /*****************************************************************************
 *
 * 05/05/2010 Debugged by Marc coussement Belgium on
 * marc.coussement@edasol.be
 *
 *****************************************************************************/

#include "USB\usb.h"
#include "USB\usb_host_msd.h"
#include "USB\usb_host_msd_scsi.h"
#include "fatfs\diskio.h"
#include "fatfs\ff.h"

static volatile DSTATUS Stat = STA_NOINIT;

/******************************************************************************
 * Function:        DSTATUS USB_disk_initialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Status after init
 *
 * Side Effects:    None
 *
 * Overview:        Function to initialize the Media called during mount
 *
 * Note:            None
 *****************************************************************************/
DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
    if(!USBHostMSDSCSIMediaInitialize(drv))
    {
        return STA_NODISK;
    }
    return 0;
}

/******************************************************************************
 * Function:        DSTATUS USB_disk_status(void)
 *
 * PreCondition:    None
 *
 * Input:           drv - Physical drive number
 *
 * Output:          Status of drive
 *
 * Side Effects:    None
 *
 * Overview:        Function to return status of drive
 *
 * Note:            None
 *****************************************************************************/
DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
    if (!USBHostMSDSCSIMediaDetect(drv))
    {
        return STA_NODISK;
    }
    return 0;
}

/******************************************************************************
 * Function:        DRESULT USB_disk_read(BYTE *buff,
 *									  DWORD sector,
 *									  BYTE count)
 *
 * PreCondition:    None
 *
 * Input:           buff - pointer to the data buffer to store read data
 *					sector - start sector number (LBA)
 *					count - sector count (1..255)
 *
 * Output:          Status of read
 *
 * Side Effects:    None
 *
 * Overview:        Function to read a specific sector on the media.
 *
 * Note:            None
 *****************************************************************************/
DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{   
	if (count == 1) {
        if (!USBHostMSDSCSISectorRead(drv, sector, buff))
        {
            return RES_NOTRDY;
        }
	}


	if (count != 1) {
			DWORD sectorCount = 512*count;
	        if (!USBHostMSDSCSIMultiSectorRead(drv, sector, buff, count, sectorCount))
	        {
	            return RES_NOTRDY;
	        }
	    }
	
	
/*
	if (count != 1) {
	
		if (count & 1) {	
	
			nCluster = (count-1)/2;
			nSector = count - (nCluster*2);
		}
		else {
			nCluster = count/2;
			nSector = 0;
		}
		int i, j;
	    for(i = 0; i < nCluster; i++)
	    {
	        if (!USBHostMSDSCSIMultiSectorRead(drv, sector, buff))
	        {
	            return RES_NOTRDY;
	        }
	        sector+=2;
	        buff += 1024;
	    }
		if (i == nCluster && nSector != 0) {
			if (!USBHostMSDSCSISectorRead(drv, sector, buff))
	        {
	            return RES_NOTRDY;
	        }
			buff += 512;
		}
	
	}
*/
    return RES_OK;
 }

/******************************************************************************
 * Function:        DRESULT USB_disk_write(const BYTE *buff,
 *									   DWORD sector,
 *									   BYTE count)
 *
 * PreCondition:    None
 *
 * Input:           buff - Pointer to the data to be written
 *					sector - Start sector number (LBA)
 *					count - Sector count (1..255)
 *
 * Output:          Status of write
 *
 * Side Effects:    None
 *
 * Overview:        Function to write a specific sector on the media.
 *
 * Note:            None
 *****************************************************************************/
#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
    
	DWORD sectorCount = count*512;
	if (count == 1) {
 		if (!USBHostMSDSCSISectorWrite(drv, sector, (BYTE *)buff,0))
        {
            return RES_NOTRDY;
        }
	}
	
	if (count != 1) {
	
 		if (!USBHostMSDSCSIMultiSectorWrite(drv, sector, (BYTE *)buff,0, count, sectorCount))
        {
            return RES_NOTRDY;
        }
	}
    return RES_OK;
}
#endif

/******************************************************************************
 * Function:        DRESULT USB_disk_ioctl(BYTE ctrl, void* buff)
 *
 * PreCondition:    None
 *
 * Input:           ctrl - control code
 *					buff - buffer to send/receive data block
 *
 * Output:          Success/Failure
 *
 * Side Effects:    None
 *
 * Overview:        Perform miscellaneous control functions
 *
 * Note:            None
 *****************************************************************************/
DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    switch(ctrl)
    {
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;

#if _READONLY == 0
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT:
        *(DWORD *)buff = 0; // Number of sectors on the volume
        return RES_OK;

    case GET_BLOCK_SIZE:
        return RES_OK;
#endif
    }
    return RES_PARERR;
}
