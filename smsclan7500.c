 /***************************************************************************
 *
 * Copyright (C) 2008-2009  SMSC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 ***************************************************************************
 * File: smsc7500.c
 ***************************************************************************
 * History:
 * 4/30/2009, sean chen,
 *****************************************************************************/
#ifndef __KERNEL__
#	define __KERNEL__
#endif

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13))
#include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/if_vlan.h>
#include <asm/io.h>

#include "smsc7500version.h"
#include "smsc7500usbnet.h"
#include "ioctl_7500.h"
#include "smsclan7500.h"


#define CHECK_RETURN_STATUS(A) { if((A) < 0){ goto DONE;} }
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define SMSC7500_VLAN_ACCEL_SUPPORT
#endif

unsigned int debug_mode = DBG_WARNING | DBG_INIT | DBG_LINK_CHANGE;

/******************************************************************************************/
/***********************	Driver Parameters *********************************************/
/******************************************************************************************/

module_param(debug_mode, uint, 0);
MODULE_PARM_DESC(debug_mode,"bit 0 enables trace points, bit 1 enables warning points, bit 2 enables eth gpios, bit 3 enables gen gpios");

u32 link_mode = 0x1FFUL;
module_param(link_mode, uint, 0);
MODULE_PARM_DESC(link_mode,"Set Link speed and Duplex, default=0xFF");
/*Bit	8		7		6		5			4			3			2			1		0		*/
/*		AUTO	ASYMP	SYMP	1000SFD		1000MFD		100FD		100HD		10FD	10HD	*/

u32 auto_mdix=AMDIX_ENABLE;
module_param(auto_mdix, uint, 0);
MODULE_PARM_DESC(auto_mdix,"Set Auto-MDIX state, 0=Disabled straight,1=Disabled crossover,2=enabled");

u32 mac_addr_hi16 = DUMMY_VALUE;
module_param(mac_addr_hi16, uint, 0);
MODULE_PARM_DESC(mac_addr_hi16,"Specifies the high 16 bits of the mac address");

u32 mac_addr_lo32 = DUMMY_VALUE;
module_param(mac_addr_lo32, uint, 0);
MODULE_PARM_DESC(mac_addr_lo32,"Specifies the low 32 bits of the mac address");

bool scatter_gather = FALSE;
module_param(scatter_gather,bool, 0);
MODULE_PARM_DESC(scatter_gather,"Enable Scatter Gather");

bool tx_Csum = FALSE;
module_param(tx_Csum,bool, 0);
MODULE_PARM_DESC(tx_Csum,"Enable Tx Hardware Checksum Offload");

bool rx_Csum = FALSE;
module_param(rx_Csum,bool, 0);
MODULE_PARM_DESC(tx_Csum,"Enable Rx Hardware Checksum Offload");

bool TurboMode = TRUE;
module_param(TurboMode,bool, 0);
MODULE_PARM_DESC(TurboMode,"Enable Turbo Mode");

int LinkActLedCfg = DUMMY_VALUE;
module_param(LinkActLedCfg, uint, 0);
MODULE_PARM_DESC(LinkActLedCfg,"Enable separate Link and Activity LEDs");

/*
linkdownsuspend = 0----> Disabled
linkdownsuspend = 1----> Enabled, wake up on auto-negotiation complete, device is in suspend0.
linkdownsuspend = 2----> Enabled, wake up on energy detection, device is in suspend1.
*/
static uint linkdownsuspend=0;
module_param(linkdownsuspend, uint, 0);
MODULE_PARM_DESC(linkdownsuspend,"Suspend device when link is down");

static uint dynamicsuspend=0;
module_param(dynamicsuspend,uint, 0);
MODULE_PARM_DESC(dynamicsuspend,"Enable dynamic autosuspend mode");

static uint netdetach=0;
module_param(netdetach,uint, 0);
MODULE_PARM_DESC(netdetach,"Enable net detach mode");

bool vlan_accel = TRUE;
module_param(vlan_accel, bool, 0);
MODULE_PARM_DESC(vlan_accel,"Enable vlan acceleration");

bool tso = FALSE;
module_param(tso, bool, 0);
MODULE_PARM_DESC(tso,"Enable TCP segmentation offload");

uint bulkin_delay=DEFAULT_BULK_IN_DELAY;
module_param(bulkin_delay,uint, 0);
MODULE_PARM_DESC(bulkin_delay,"16 bit value in units of 16ns to delay UTX sending data");

/******************************************************************************************/
/************************Static Functions and Variables ***********************************/
/******************************************************************************************/
static int smsc7500_reset(struct usbnet *dev);
static int smsc7500_get_stats(struct usbnet *dev, u32 *data);
static int smsc7500_private_ioctl(PADAPTER_DATA  privateData, struct usbnet *dev, PSMSC7500_IOCTL_DATA ioctlData);
static int smsc7500_device_recovery(struct usbnet *dev);
static int smsc7500_system_suspend (struct usb_interface *intf, pm_message_t state);
static int smsc7500_system_resume(struct usb_interface *intf);
static u16 calculate_crc16(const BYTE * bpData,const u32 dwLen, const BOOLEAN fBitReverse);
static int set_linkdown_wakeup_events(struct usbnet *dev, int wakeUpMode);
static int reset_linkdown_wakeup_events(struct usbnet *dev);
static int smsc7500_autosuspend(struct usb_interface *intf, pm_message_t state);
static int smsc7500_autoresume(struct usb_interface *intf);
static int enable_PHY_wakeupInterrupt(struct usbnet *dev, u32 interrupt);
static int Disable_PHY_wakeupInterrupt(struct usbnet *dev, u32 interrupt);
static int smsc7500_bind(struct usbnet *dev, struct usb_interface *intf);
static void smsc7500_unbind(struct usbnet *dev, struct usb_interface *intf);
static void smsc7500_status(struct usbnet *dev, struct urb *urb);
static int smsc7500_set_frame_size(struct usbnet *dev, int size);
static int smsc7500_link_reset(struct usbnet *dev);
static int smsc7500_rx_fixup(struct usbnet *dev, struct sk_buff *skb);
static struct sk_buff *smsc7500_tx_fixup(struct usbnet *dev, struct sk_buff *skb, int flags);
static int smsc7500_rx_setmulticastlist(struct usbnet *dev);
static int smsc7500_start_tx_path(struct usbnet * dev);
static int smsc7500_stop_tx_path(struct usbnet * dev);
static int smsc7500_start_rx_path(struct usbnet * dev);
static int smsc7500_stop_rx_path(struct usbnet * dev);
static int set_flow_control(struct usbnet *dev, BOOLEAN txEn, BOOLEAN rxEn);
static int smsc7500_eth_mac_addr(struct net_device *netdev, void *p);
static int Phy_reset(struct usbnet *dev);
static BOOLEAN Phy_Initialize(struct usbnet *dev);
static int phy_set_link(struct usbnet *dev, u32 dwLinkRequest);
static int Phy_gig_workaround(struct usbnet *dev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11))
static int smsc7500_suspend (struct usb_interface *intf,  u32 state);
#else
static int smsc7500_suspend (struct usb_interface *intf, pm_message_t state);
#endif
static int smsc7500_resume(struct usb_interface *intf);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0))
#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
static void smsc7500_vlan_rx_register(struct net_device *ndev, struct vlan_group *grp);
#endif
#endif
static int set_reg_bits(struct usbnet *dev, u32 regAddr, u32 data);
static int reset_reg_bits(struct usbnet *dev, u32 regAddr, u32 data);
static int set_phy_bits(struct usbnet *dev, u32 phyAddr, u32 data);
static int reset_phy_bits(struct usbnet *dev, u32 phyAddr, u32 data);
#ifdef USE_DEBUG
void static LanDumpRegs(struct usbnet *dev);
#endif //USE_DEBUG
#define SET_REG_ARRAY(NAME)			lanRegMap[SYS_##NAME]	= NAME
#define SET_PHY_ARRAY(NAME)			phyRegMap[REG_##NAME]	= NAME;
static u32 lanRegMap[NO_OF_SYS_REGISTER];
static u32 phyRegMap[NO_PHY_REGS];
#define ETHTOOL_REG_SIZE        ((NO_OF_SYS_REGISTER + NO_PHY_REGS) * 4)
/******************************************************************/
/**************  Static structure declaration  ********************/
/*****************************************************************/
static const struct driver_info smsc7500_info = {
	.description = "smsc7500 USB 2.0 Ethernet",
	.bind = 		smsc7500_bind,
	.unbind =		smsc7500_unbind,
	.status = 		smsc7500_status,
	.set_max_frame_size = smsc7500_set_frame_size,
	.link_reset = 	smsc7500_link_reset,
	.reset = 		smsc7500_reset,
	.flags = 		FLAG_ETHER|FLAG_SEND_ZLP,
	.rx_fixup = 	smsc7500_rx_fixup,
	.tx_fixup = 	smsc7500_tx_fixup,
	.rx_setmulticastlist = smsc7500_rx_setmulticastlist,
};

static const struct usb_device_id	products [] = {
	{
		// SMSC7500 USB Ethernet Device
		USB_DEVICE (0x0424, 0x7500),
		.driver_info = (unsigned long) &smsc7500_info,
	},
	{
		// SMSC7500 USB Ethernet Device
		USB_DEVICE (0x0424, 0x7505),
		.driver_info = (unsigned long) &smsc7500_info,
	},
	{ },		// END
};

MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver smsc7500_driver = {
	.name =		"smsc7500",
	.id_table =	products,
	.probe =	smscusbnet_probe,
	.suspend =	smsc7500_suspend,
	.resume =	smsc7500_resume,
	.disconnect =	smscusbnet_disconnect,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
    .supports_autosuspend = 1,
#endif
};
/******************************************************************/
/**************  End of Static structure declaration  ***********/
/*****************************************************************/


/************************************************************************
Routine Description:
    This routine reads data from device register.
Arguments:
	dev 	- Point to device structure usbnet
	regAddr	- Register address
	data	- Buf to store data
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
*************************************************************************/
static int smsc7500_read_reg(struct usbnet *dev, u32 regAddr, u32 *data)
{
	int retVal = 0;
    u32 *buf = NULL;
    u16 retry_count = 0;

    BUG_ON(!dev);

//The heap buffer should be used for usb_control_msg, because the stack might not be DMA-mappable
//Control message is very slow so it really isn't big deal to dynamically allocate the data
	buf = kmalloc (sizeof(u32), GFP_KERNEL);
    if(buf == NULL){
        return SMSC7500_FAIL;
    }

    do{
    	retVal=usb_control_msg(
    		dev->udev,
    		usb_rcvctrlpipe(dev->udev, 0),
    		USB_VENDOR_REQUEST_READ_REGISTER,
    		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
    		00,
    		regAddr,
    		(void*)buf,
    		sizeof(u32),
    		USB_CTRL_GET_TIMEOUT);
    }while((retVal < 0) && (retry_count++ < 3));

	if (retVal<0){
		SMSC_WARNING("Failed to read register regAddr 0x%08x", regAddr);
    }else{
        le32_to_cpus(buf);
	   *data = *buf;
    }

	kfree(buf);

	return retVal;
}

/****************************************************************************
Routine Description:
    This routine writes data to device register.
Arguments:
	dev 	- Point to device structure usbnet
	regAddr	- Register address
	data	- data to be written
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
***************************************************************************/
static int smsc7500_write_reg(struct usbnet *dev,  u32 regAddr, u32 data)
{
	int retVal = SMSC7500_SUCCESS;
    u32* buf = NULL;
    u16 retry_count = 0;

    BUG_ON(!dev);

//The heap buffer should be used for usb_control_msg, because the stack might not be DMA-mappable
//Control message is very slow so it really isn't big deal to dynamically allocate the data
    buf = kmalloc (sizeof(u32), GFP_KERNEL);
    if(buf == NULL){
        return SMSC7500_FAIL;
    }
    *buf = data;

	cpu_to_le32s(buf);

    do{
    	retVal=usb_control_msg(
    		dev->udev,
    		usb_sndctrlpipe(dev->udev, 0),
    		USB_VENDOR_REQUEST_WRITE_REGISTER,
    		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
    		00,
    		regAddr,
    		buf,
    		sizeof(u32),
    		USB_CTRL_SET_TIMEOUT);
    }while((retVal < 0) && (retry_count++ < 3));

	if (retVal<0){
		SMSC_WARNING("Failed to write register regAddr 0x%08x", regAddr);
    }

    kfree(buf);

	return retVal;
}

/*********************************************************************
Routine Description:
    This routine sets usb feature.
Arguments:
	dev 	- Point to device structure usbnet
	feature	- usb features
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
********************************************************************/
static int smsc7500_set_feature(struct usbnet *dev,  u32 feature)
{
    BUG_ON(!dev);

	cpu_to_le32s((u32*)&feature);

	return usb_control_msg(
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		USB_REQ_SET_FEATURE,
		USB_RECIP_DEVICE,
		feature,
		0,
		NULL,
		0,
		USB_CTRL_SET_TIMEOUT);

}

/***********************************************************************
Routine Description:
    This routine clears usb feature.
Arguments:
	dev 	- Point to device structure usbnet
	feature	- usb features
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
**********************************************************************/
static int smsc7500_clear_feature(struct usbnet *dev,  u32 feature)
{
    BUG_ON(!dev);

	cpu_to_le32s((u32*)&feature);

	return usb_control_msg(
			dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			USB_REQ_CLEAR_FEATURE,
			USB_RECIP_DEVICE,
			feature,
			0,
			NULL,
			0,
			USB_CTRL_SET_TIMEOUT);
}

/********************************************************************************
Routine Description:
    This routine reads PHY registers through MII interface. The phy_mutex will be
    acquired during reading and released after reading.
Arguments:
	dev 		- Point to device structure usbnet
	phyRegIndex	- PHY register index
	pValue32	- Buffer to store register data
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
**********************************************************************************/
static int smsc7500_read_phy(struct usbnet *dev,  u32 phyRegIndex, u32 *pValue32)

{
	int retVal = SMSC7500_FAIL;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue,dwAddr;
	int Count;

    BUG_ON(!dev);

     if(down_interruptible(&adapterData->phy_mutex)){
        return -EINTR;
     }
    // confirm MII not busy
    CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MII_ACCESS, &dwValue));

    if ((dwValue & MII_ACCESS_BUSY) != 0UL)
    {
		SMSC_WARNING("MII is busy in smsc7500_read_phy\n");
       	goto DONE;
    }

    // set the address, index & direction (read from PHY)
    dwAddr = ((adapterData->dwPhyAddress << MII_ACCESS_PHY_ADDR_SHIFT) & MII_ACCESS_PHY_ADDR) | ((phyRegIndex << MII_ACCESS_MIIRINAD_SHIFT) & MII_ACCESS_MIIRINAD);
    dwAddr &= ~MII_ACCESS_RW;
    dwAddr |= MII_ACCESS_BUSY;

    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, MII_ACCESS, dwAddr));

	// Loop until the read is completed w/timeout
	for(Count=1;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MII_ACCESS, &dwValue));

		if(!(dwValue & MII_ACCESS_BUSY))
        		break;
		udelay(1);
	}

	if (Count < 100)
	{
		retVal = smsc7500_read_reg(dev, MII_DATA, pValue32);
		*pValue32 &= MII_DATA_MASK;
	}
	else
	{
		SMSC_WARNING ("Timed out reading MII register %08X\n",phyRegIndex);

	}
DONE:
    up(&adapterData->phy_mutex);
	return retVal;

} /* smsc7500_read_phy */

/**************************************************************************************
Routine Description:
    This routine writes PHY registers through MII interface. The phy_mutex will be
    acquired during writing and released after writing.
Arguments:
	dev 		- Point to device structure usbnet
	phyRegIndex	- PHY register index
	pValue32	- Data to be written into register
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Fail
***********************************************************************************/
static int smsc7500_write_phy(struct usbnet *dev,  u32 phyRegIndex, u32 pValue32)
{

	int retVal = SMSC7500_FAIL;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue,dwAddr;
	int Count;

    BUG_ON(!dev);

    if(down_interruptible(&adapterData->phy_mutex)){
        return -EINTR;
     }

    // confirm MII not busy
    CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MII_ACCESS, &dwValue));

    if ((dwValue & MII_ACCESS_BUSY) != 0UL)
    {
		SMSC_WARNING ("MII is busy in smsc7500_read_phy\n");
        	goto DONE;
    }

 	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, MII_DATA, pValue32 & MII_DATA_MASK));

    // set the address, index & direction (read from PHY)
	dwAddr = ((adapterData->dwPhyAddress << MII_ACCESS_PHY_ADDR_SHIFT) & MII_ACCESS_PHY_ADDR) | ((phyRegIndex << MII_ACCESS_MIIRINAD_SHIFT) & MII_ACCESS_MIIRINAD);
	dwAddr |= MII_ACCESS_RW | MII_ACCESS_BUSY;

    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, MII_ACCESS, dwAddr));

	// Loop until the read is completed w/timeout
	for(Count=1;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MII_ACCESS, &dwValue));
		if(!(dwValue & MII_ACCESS_BUSY))break;
		udelay(1);

	}

	if (Count < 100)
	{
		retVal = SMSC7500_SUCCESS;
	}
	else
	{
		SMSC_WARNING("Timed out writing MII register %08X\n",phyRegIndex);

	}
DONE:
    up(&adapterData->phy_mutex);
	return retVal;

} /* smsc7500_write_phy */

/**************************************************************************************
Routine Description:
    This routine waits for eeprom read/write completion.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	SMSC7500_SUCCESS 	- Operation completed
	SMSC7500_TIMEOUT	- Operation time out
***********************************************************************************/
static int smsc7500_eeprom_op_complete(struct usbnet *dev)
{
	int retVal = SMSC7500_SUCCESS;
	u32 dwValue;
	int Count;

    BUG_ON(!dev);

	for(Count=0;Count<1000;Count++) //40ms
	{
		if(smsc7500_read_reg(dev, E2P_CMD, &dwValue) < 0){
            return SMSC7500_FAIL;
        }
	    if (!(dwValue & E2P_CMD_BUSY) || (dwValue & E2P_CMD_TIMEOUT))
	    {
	    	break;
	    }
		udelay(40);
	}
   	if ((dwValue & E2P_CMD_TIMEOUT) || (dwValue & E2P_CMD_BUSY)){
    		SMSC_WARNING("EEPROM operation timeout");
    		retVal = SMSC7500_TIMEOUT;
   	}

    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine reads data from EEPROM.
Arguments:
	dev 		- Point to device structure usbnet
	dwOffset	- The base EEPROM address to read form
	dwLength	- Total bytes to be read
	pbValue		- Buffer to store data
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Operation fails
***********************************************************************************/
static int smsc7500_read_eeprom(struct usbnet *dev,  u32 dwOffset, u32 dwLength,  BYTE* pbValue)
{

	int retVal = SMSC7500_FAIL;
	u32 dwValue,dwAddr;
	int Count, i;
	PADAPTER_DATA adapterData;

    BUG_ON(!dev);
    BUG_ON(!pbValue);

    adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(dwOffset + dwLength > adapterData->eepromSize){
		SMSC_WARNING("EEPROM: out of eeprom space range, offset = %d, dwLength = %d", dwOffset, dwLength);
	}

    // confirm eeprom not busy
	for(Count=0;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, E2P_CMD, &dwValue));
	    if (!(dwValue & E2P_CMD_BUSY) || !(dwValue & E2P_CMD_LOADED))
	    {
	    	break;
	    }
		udelay(40);
	}
    if (!(dwValue & E2P_CMD_LOADED))
    {
		SMSC_WARNING("No EEPROM present");
        goto DONE;
    }
    if ((dwValue & E2P_CMD_BUSY) != 0UL)
    {
		SMSC_WARNING("EEPROM is busy ");
        goto DONE;
    }
    dwAddr = dwOffset;
    for(i=0; i<dwLength; i++){
    	//Isuue command
    	dwValue = E2P_CMD_BUSY | E2P_CMD_READ | (dwAddr & E2P_CMD_ADDR);
    	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, E2P_CMD, dwValue));
    	if(smsc7500_eeprom_op_complete(dev) == SMSC7500_TIMEOUT)goto DONE;


    	//Read data when ready
    	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, E2P_DATA, &dwValue));

    	pbValue[i] = dwValue & 0xFF;
    	dwAddr++;
    }
	retVal = SMSC7500_SUCCESS;
DONE:
	return retVal;

} /* smsc7500_read_eeprom */

/**************************************************************************************
Routine Description:
    This routine write data to EEPROM.
Arguments:
	dev 		- Point to device structure usbnet
	dwOffset	- The base EEPROM address to read form
	dwLength	- Total bytes to be read
	pbValue		- Buffer to store data
Return Value:
	SMSC7500_SUCCESS 	- Success
	SMSC7500_FAIL		- Operation fails
***********************************************************************************/
static int smsc7500_write_eeprom(struct usbnet *dev,  u32 dwOffset, u32 dwLength,  BYTE* pbValue)
{
	int retVal = SMSC7500_FAIL;
	u32 dwValue,dwAddr;
	int Count, i;
	PADAPTER_DATA adapterData;

    BUG_ON(!dev);
    BUG_ON(!pbValue);

    adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(dwOffset + dwLength > adapterData->eepromSize){
		SMSC_WARNING("EEPROM: out of eeprom space range");
	}

    // confirm eeprom not busy
	for(Count=0;Count<100;Count++)
	{
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, E2P_CMD, &dwValue));

	    if (!(dwValue & E2P_CMD_BUSY))
	    {
	    	break;
	    }
		udelay(40);
	}

    if ((dwValue & E2P_CMD_BUSY) != 0UL)
    {
		SMSC_WARNING("EEPROM is busy ");
        goto DONE;
    }

    //Issue write/erase enable command
    dwValue = E2P_CMD_BUSY | E2P_CMD_EWEN;
    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, E2P_CMD, dwValue));
    if(smsc7500_eeprom_op_complete(dev) == SMSC7500_TIMEOUT)goto DONE;

    dwAddr = dwOffset;
    for(i=0; i<dwLength; i++){

    	//Fill data register
    	dwValue = pbValue[i];
    	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, E2P_DATA, dwValue));

		//Send "write"  command
    	dwValue = E2P_CMD_BUSY | E2P_CMD_WRITE | (dwAddr & E2P_CMD_ADDR);
    	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, E2P_CMD, dwValue));
    	if(smsc7500_eeprom_op_complete(dev) == SMSC7500_TIMEOUT)goto DONE;

    	dwAddr++;
    }

	retVal = SMSC7500_SUCCESS;
DONE:
	return retVal;

} /* smsc7500_write_eeprom */

/**************************************************************************************
Routine Description:
    This routine waits for data port operation completion.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	TRUE 	- Data port ready.
	FALSE	- Data port busy.
***********************************************************************************/
static int IsDataPortReady(struct usbnet *dev){
	int retVal = FALSE;
	int count = 0;
	u32 dwValue;

// confirm data port is not busy
	for(count=0; count<100; count++)
	{
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, DP_SEL, &dwValue));
		if (dwValue & DP_SEL_DPRDY){
			retVal = TRUE;
			break;
		}
		udelay(40);
	}

	if (retVal == FALSE)
	{
		SMSC_WARNING("Data port is busy ");
	}
DONE:
	return retVal;
}

/**************************************************************************************
Routine Description:
    This routine read data from internal RAM.
Arguments:
	dev 		- Point to device structure usbnet
	ramSel		- Choose which internal RAM to access.
	startAddr	- The first offset to access.
	length		- Data length in DWORD.
	val			- buffer pointer.

Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int read_data_port(struct usbnet *dev, int ramSel, u32 startAddr, u32 length, u32 *val)
{
	u32 dwValue;
	int retVal = SMSC7500_FAIL;
	int i;
	PADAPTER_DATA adapterData;

	BUG_ON(!dev);
	adapterData = (PADAPTER_DATA)(dev->data[0]);
	BUG_ON(!adapterData);

    if(down_interruptible(&adapterData->internal_ram_mutex)){
        return -EINTR;
     }

  // confirm data port not busy
	if(!IsDataPortReady(dev))goto DONE;

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, DP_SEL, &dwValue));
	dwValue &= ~DP_SEL_RSEL;
	switch(ramSel){
		case RAMSEL_URX:			dwValue |= DP_SEL_URX; 			break;
		case RAMSEL_VHF:			dwValue |= DP_SEL_VHF; 			break;
		case RAMSEL_LSO_HEAD:		dwValue |= DP_SEL_LSO_HEAD; 	break;
		case RAMSEL_FCT_TX:			dwValue |= DP_SEL_FCT_TX; 		break;
		case RAMSEL_FCT_RX:			dwValue |= DP_SEL_FCT_RX; 		break;
		case RAMSEL_DESCRIPTOR:		dwValue |= DP_SEL_DESCRIPTOR; 	break;
		case RAMSEL_WOL:			dwValue |= DP_SEL_WOL; 			break;
	}

	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_SEL, dwValue));

	for(i=0; i<length; i++){
		//Set device ram address
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_ADDR, startAddr + i));
		//Enable reading
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_CMD, DP_CMD_READ));

		if(!IsDataPortReady(dev))goto DONE;

		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, DP_DATA, &val[i]));
	}

	retVal = SMSC7500_SUCCESS;
DONE:

    up(&adapterData->internal_ram_mutex);

	return retVal;
}

/**************************************************************************************
Routine Description:
    This routine write data to internal RAM.
Arguments:
	dev 		- Point to device structure usbnet
	ramSel		- Choose which internal RAM to access.
	startAddr	- The first offset to access.
	length		- Data length in DWORD.
	val			- buffer pointer.

Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int write_data_port(struct usbnet *dev, int ramSel, u32 startAddr, u32 length, u32 *val)
{
	u32 dwValue;
	int retVal = SMSC7500_FAIL;
	int i;
	PADAPTER_DATA adapterData;

	BUG_ON(!dev);
	adapterData = (PADAPTER_DATA)(dev->data[0]);
	BUG_ON(!adapterData);

    if(down_interruptible(&adapterData->internal_ram_mutex)){
        return -EINTR;
     }

  // confirm data port not busy
	if(!IsDataPortReady(dev))goto DONE;

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, DP_SEL, &dwValue));
	dwValue &= ~DP_SEL_RSEL;
	switch(ramSel){
		case RAMSEL_URX:			dwValue |= DP_SEL_URX; 			break;
		case RAMSEL_VHF:			dwValue |= DP_SEL_VHF; 			break;
		case RAMSEL_LSO_HEAD:		dwValue |= DP_SEL_LSO_HEAD; 	break;
		case RAMSEL_FCT_TX:			dwValue |= DP_SEL_FCT_TX; 		break;
		case RAMSEL_FCT_RX:			dwValue |= DP_SEL_FCT_RX; 		break;
		case RAMSEL_DESCRIPTOR:		dwValue |= DP_SEL_DESCRIPTOR; 	break;
		case RAMSEL_WOL:			dwValue |= DP_SEL_WOL; 			break;
	}

	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_SEL, dwValue));

	for(i=0; i<length; i++){
		//Set device ram address
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_ADDR, startAddr + i));
		//Set data
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_DATA, val[i]));
		//Enable writing
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, DP_CMD, DP_CMD_WRITE));

		if(!IsDataPortReady(dev))goto DONE;
	}

	retVal = SMSC7500_SUCCESS;
DONE:

    up(&adapterData->internal_ram_mutex);

	return retVal;
}

/**************************************************************************************
Routine Description:
    This routine checks rx FIFO through interrupt endpoint.
Arguments:
	dev 		- Point to device structure usbnet
	urb			- Interrupt urb
Return Value:

***********************************************************************************/
static void smsc7500_status(struct usbnet *dev, struct urb *urb)
{
	struct smsc7500_int_data *event;
	int hasFrame;

    BUG_ON(!dev);
    BUG_ON(!urb);

	SMSC_DEBUG(DBG_INTR,"---->in smsc7500_status\n");

	if (urb->actual_length < 4) {
		SMSC_WARNING("urb->actual_length= %d",urb->actual_length);
		return;
	}

	event = urb->transfer_buffer;

	le32_to_cpus((u32*)&event->IntEndPoint);

	SMSC_DEBUG(DBG_INTR, "event->IntEndPoint= 0x%08x\n", event->IntEndPoint);
	hasFrame = event->IntEndPoint & INT_END_RXFIFO_HAS_FRAME;

	if (hasFrame) {
		dev->StopSummitUrb=0;
		tasklet_schedule (&dev->bh);
	}

	SMSC_DEBUG(DBG_INTR,"<----out of smsc7500_status\n");
}

static int smsc7500_set_frame_size(struct usbnet *dev, int size)
{
	int retVal = 0;
	u32 dwData, rxen = FALSE;

	if ((retVal = smsc7500_read_reg(dev, MAC_RX, &dwData)< 0)) {
		SMSC_WARNING("Failed to read BURST_CAP: %d", retVal);
		return retVal;
	}

	if(dwData & MAC_RX_RXEN){
		rxen = TRUE;
		dwData &= ~MAC_RX_RXEN;
		if ((retVal = smsc7500_write_reg(dev, MAC_RX, dwData)< 0)) {
			return retVal;
		}
	}
	dwData &= ~MAC_RX_MAX_SIZE;
	dwData |= (size + 4) << MAC_RX_MAX_SIZE_SHIFT; //Add FCS size
	if ((retVal = smsc7500_write_reg(dev, MAC_RX, dwData)< 0)) {
		return retVal;
	}
	if(rxen){
		dwData |= MAC_RX_RXEN;
		if ((retVal = smsc7500_write_reg(dev, MAC_RX, dwData)< 0)) {
			return retVal;
		}
	}

	return retVal;
}
/**************************************************************************************
Routine Description:
    This routine gets statistics counter through vendor command.
Arguments:
	dev 		- Point to device structure usbnet
	data		- Data buffer
Return Value:
	0			- Success
	<0			- Fail
***********************************************************************************/
static int smsc7500_get_stats(struct usbnet *dev, u32 *data)
{
	int retVal = 0;
    u16 retry_count = 0;
    u32 *statistics;
    int cntSize = NO_OF_STAS_CNT * sizeof(u32);

    BUG_ON(!dev);
    BUG_ON(!data);

	SMSC_DEBUG(DBG_RX, "in smsc7500_get_stats\n");

	statistics = kmalloc (cntSize, GFP_KERNEL);
    if(statistics == NULL){
        return SMSC7500_FAIL;
    }

    do{
	   retVal=usb_control_msg(
    		dev->udev,
    		usb_rcvctrlpipe(dev->udev, 0),
    		USB_VENDOR_REQUEST_GET_STATS,
    		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
    		00,
    		0,
    		(void*)statistics,
    		cntSize,
    		USB_CTRL_SET_TIMEOUT);
    }while((retVal < 0) && (retry_count++ < 3));

    if (retVal < 0){
        SMSC_WARNING("Failed to get status");
    }else{
        memcpy(data,  statistics, cntSize);
    }

    kfree(statistics);

	return retVal;
}

/**************************************************************************************
Routine Description:
    This is urb callback function.
Arguments:
	urb			-
Return Value:
***********************************************************************************/
/*
static void smsc7500_async_cmd_callback(struct urb *urb, struct pt_regs *regs)
{
	struct USB_CONTEXT * usb_context = (struct USB_CONTEXT *)urb->context;

	if (urb->status < 0)
		SMSC_WARNING("smsc7500_async_cmd_callback() failed with %d\n",urb->status);

	complete((struct completion *)&usb_context->notify);

	kfree(&usb_context->req);
	usb_free_urb(urb);
}
*/
/**************************************************************************************
Routine Description:
    This routine reads device registers, synchronized by wait_for_completion_timeout().
    The callback will call complete() to notify completion.
Arguments:
	dev 		- Point to device structure usbnet
	index		- register address
	data		- Data buffer
	wait		- if ture, wait for completion.
Return Value:
	ASYNC_RW_SUCCESS			- Success
	ASYNC_RW_TIMEOUT			- timwout to wait for completion.
***********************************************************************************/
/*
static int smsc7500_read_reg_async(struct usbnet *dev,   u32 index, void *data, int wait)
{
	int retVal = ASYNC_RW_SUCCESS, expire;
	struct USB_CONTEXT * usb_context;
	int status;
	struct urb *urb;
	u32 size=4;

    BUG_ON(!dev);
    BUG_ON(!data);

	if ((urb = usb_alloc_urb(0, GFP_ATOMIC)) == NULL) {
		SMSC_WARNING("Error allocating URB in write_cmd_async!");
		return ASYNC_RW_FAIL;
	}

	if ((usb_context = kmalloc(sizeof(struct USB_CONTEXT), GFP_ATOMIC)) == NULL) {
		SMSC_WARNING( "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return ASYNC_RW_FAIL;
	}

	usb_context->req.bRequestType = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	usb_context->req.bRequest = USB_VENDOR_REQUEST_READ_REGISTER;
	usb_context->req.wValue = 00;
	usb_context->req.wIndex = cpu_to_le32(index);
	usb_context->req.wLength = cpu_to_le32(size);
	init_completion(&usb_context->notify);

	usb_fill_control_urb(urb, dev->udev,
			     usb_rcvctrlpipe(dev->udev, 0),
			     (void *)&usb_context->req, data, size,
			     (usb_complete_t)smsc7500_async_cmd_callback, (void*)usb_context);

	if((status = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		SMSC_WARNING( "Error submitting the control message: status=%d", status);
		kfree(usb_context);
		usb_free_urb(urb);
	}

	if(wait){
//wait_for_completion_timeout only implemented in 2.6.11 and higher kernel version
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
	    expire = msecs_to_jiffies(USB_CTRL_SET_TIMEOUT);
	    if (!wait_for_completion_timeout(&usb_context->notify, expire)) {

	  		retVal = ASYNC_RW_TIMEOUT;
	  		SMSC_DEBUG(DBG_WARNING,"urb timeout \n");
	  		kfree(usb_context);
	        usb_free_urb(urb);
	    }
#endif
	}

	return retVal;

}
*/


/**************************************************************************************
Routine Description:
    This routine calculates tx checksum offset.
Arguments:
	skb						- skb buffer to calculate with
	csum_start_offset(OUT)	- result
Return Value:
***********************************************************************************/
#if 0
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
extern void *page_address(struct page *);
static void calculate_tx_checksum_offset(
	struct sk_buff *skb,
	int *csum_start_offset
	)
{
	unsigned int skbFragCnt;
	int i;
	u32	offset;

	skbFragCnt = skb_shinfo(skb)->nr_frags + 1;

	// Initialize csum offset locations as if it was single frag.
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
		SMSC_ASSERT(skb->h.raw);
	#else
		SMSC_ASSERT(skb->transport_header);	// Should never happen for a CHECKSUM_HW packet.
	#endif

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
		*csum_start_offset = skb->h.raw - skb->data;
	#else
		*csum_start_offset = (unsigned long)skb->transport_header - (unsigned long)skb->data;
	#endif

	offset = (skbFragCnt == 1) ? skb->len : (skb->len - skb->data_len);

	// Process all fragments
	for(i=0;i<(skbFragCnt-1);i++)
	{
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		unsigned char *frag_addr = (unsigned char *) page_address(frag->page) + frag->page_offset;

		// Find if transport header start belongs to this fragment and if so calculate offset from start of packet.
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
			if((frag_addr <= skb->h.raw) && ((frag_addr + frag->size) >=skb->h.raw))
			{
				*csum_start_offset = offset + ((u32)skb->h.raw) - ((u32)frag_addr);
			}
		#else
			if((frag_addr <= (unsigned char *)((unsigned long)skb->transport_header)) &&
				((frag_addr + frag->size) >= (unsigned char *)((unsigned long)skb->transport_header)))
			{
				*csum_start_offset = offset + ((unsigned long)skb->transport_header) - ((unsigned long)frag_addr);
			}
		#endif

		SMSC_ASSERT((offset + frag->size) <= skb->len);

		offset += frag->size;
	}

}
#endif
#endif

/**************************************************************************************
Routine Description:
    This routine will notify upper layer to stop tx queue.
Arguments:
	dev 		- Point to device structure usbnet
	dwSource	- Source to call this routine
Return Value:
***********************************************************************************/
static void tx_stop_queue(struct usbnet *dev,u32 dwSource)
{
	unsigned long intFlags=0;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	spin_lock_irqsave(&(adapterData->TxQueueLock),intFlags);
	if(adapterData->dwTxQueueDisableMask==0) {
		netif_stop_queue(dev->net);
	}
	adapterData->dwTxQueueDisableMask|=dwSource;
	spin_unlock_irqrestore(&(adapterData->TxQueueLock),intFlags);
}

/**************************************************************************************
Routine Description:
    This routine will notify upper layer to start tx queue.
Arguments:
	dev 		- Point to device structure usbnet
	dwSource	- Source to call this routine
Return Value:
***********************************************************************************/
static void tx_wake_queue(
	struct usbnet *dev,u32 dwSource)
{
	unsigned long intFlags=0;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	spin_lock_irqsave(&(adapterData->TxQueueLock),intFlags);
	adapterData->dwTxQueueDisableMask&=(~dwSource);
	if(adapterData->dwTxQueueDisableMask==0) {
		netif_wake_queue(dev->net);
	}
	spin_unlock_irqrestore(&(adapterData->TxQueueLock),intFlags);
}

/**************************************************************************************
Routine Description:
    This routine returns hash bit number for given MAC address.
Arguments:
	addr		- MAC address
Return Value:
	hash bit number for given MAC address
For example:
   01 00 5E 00 00 01 -> returns
***********************************************************************************/
static u32 rx_hash(BYTE addr[])
{
	int i;
	u32 crc=0xFFFFFFFFUL;
	u32 poly=0xEDB88320UL;
	u32 result=0;

	for(i=0; i<ETH_ADDR_LEN; i++)
	{
		int bit;
		u32 data=((u32)addr[i]);

		for(bit=0;bit<8;bit++)
		{
			u32 p = (crc^((u32)data))&1UL;
			crc >>= 1;
			if(p!=0) crc ^= poly;
			data >>=1;
		}
	}

  // The calculation above produced a bit-reversed result, so un-reverse it
   result=((crc&0x001UL)<<8)|
	           ((crc&0x002UL)<<6)|
	           ((crc&0x004UL)<<4)|
	           ((crc&0x008UL)<<2)|
	           ((crc&0x010UL))|
	           ((crc&0x020UL)>>2)|
	           ((crc&0x040UL)>>4)|
	           ((crc&0x080UL)>>6)|
	           ((crc&0x100UL)>>8);

   result &= (VHF_HASH_LEN * 32 - 1); // hash result has to be a number between 0 and 511

   return result;
}

/**************************************************************************************
Routine Description:
    This routine prepares and fills Hash table register.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int smsc7500_rx_setmulticastlist(struct usbnet *dev)
{

	u32 dwRef, i;
	u32 retVal = SMSC7500_FAIL;

	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	if (dev->suspendFlag) {
		return 0;
	}

	if(down_interruptible(&adapterData->RxFilterLock)){
       return -EINTR;
    }

	SMSC_DEBUG(DBG_MCAST, "---------->in smsc7500_set_multicast\n");

	for(i=0; i<VHF_HASH_LEN; i++){
		adapterData->MulticastHashTable[i] = 0;
	}

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, REF_CTL, &dwRef));

    if(dev->net->flags & IFF_PROMISC) {
		SMSC_DEBUG(DBG_MCAST,"Promiscuous Mode Enabled");
		dwRef |= REF_CTL_AB | REF_CTL_AM | REF_CTL_AU;
		dwRef &= (~(REF_CTL_DPF | REF_CTL_MHF));
	}else if(dev->net->flags & IFF_ALLMULTI) {
		SMSC_DEBUG(DBG_MCAST, "Receive all Multicast Enabled");
		dwRef |= REF_CTL_AM;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	}else if(dev->net->mc_count>0) {
		u32 dwCount=0;
		struct dev_mc_list *mc_list=dev->net->mc_list;

		dwRef &= ~REF_CTL_AM;
		dwRef |= REF_CTL_MHF;

		while(mc_list!=NULL) {
			dwCount++;
			if((mc_list->dmi_addrlen) == ETH_ADDR_LEN) {
				u32 dwBitNum = rx_hash(mc_list->dmi_addr);
				adapterData->MulticastHashTable[dwBitNum/32] |= (1UL << (dwBitNum % 32));
			} else {
				SMSC_WARNING("dmi_addrlen!=6");
			}
			mc_list=mc_list->next;
		}
		if(dwCount != ((u32)(dev->net->mc_count))) {
			SMSC_WARNING("dwCount!=dev->net->mc_count");
		}
#else
	}else if( netdev_mc_count(dev->net)>0) {
		struct netdev_hw_addr *ha;
		dwRef &= ~REF_CTL_AM;
		dwRef |= REF_CTL_MHF;
		netdev_for_each_mc_addr(ha, dev->net) {
			u32 dwBitNum = rx_hash(ha->addr);
			adapterData->MulticastHashTable[dwBitNum/32] |= (1UL << (dwBitNum % 32));
		}
#endif
	}else{

		dwRef &= ~(REF_CTL_AM | REF_CTL_MHF);
		SMSC_DEBUG(DBG_MCAST, "Receive own packets only.");
	}

	up(&adapterData->RxFilterLock);

	retVal = write_data_port(dev, RAMSEL_VHF, VHF_VLAN_LEN, VHF_HASH_LEN, adapterData->MulticastHashTable);

	if(retVal != SMSC7500_SUCCESS)goto DONE;

	CHECK_RETURN_STATUS(smsc7500_write_reg(dev ,REF_CTL, dwRef));

	retVal = 0;
DONE:

	SMSC_DEBUG(DBG_MCAST, "<---------out of smsc7500_set_multicast");
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine send EVENT_SET_MULTICAST to myevent().
Arguments:
	netdev 		- Point to net device structure
Return Value:
***********************************************************************************/
static void smsc7500_set_multicast(struct net_device *netdev)
{
	struct usbnet *dev=netdev_priv(netdev);
	smscusbnet_defer_myevent(dev, EVENT_SET_MULTICAST);

}
static const u32 pause_mode_table[] =
{
                0,      0,      0,      0,    
                0,      3,      0,      3,
                0,      0,      0,      1,
                0,      3,      2,      3
};

/**************************************************************************************
Routine Description:
    This routine gets current link mode from device register.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int phy_get_link_mode(struct usbnet *dev)
{
	u32 mssr, mscr;
	u32 regLPA, regADV, wRegBSR, CommonDenom, flow_index, flow_mode;
    int retVal = SMSC7500_FAIL;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_DEBUG(DBG_LINK, "---------->in phy_get_link_mode");

//	adapterData->actualLinkMode = LINK_OFF;
	CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &wRegBSR));
	CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &wRegBSR));
	if(!(wRegBSR & PHY_BSR_LINK_STATUS)) {//Link down
		adapterData->actualLinkMode = LINK_OFF;
		retVal = SMSC7500_SUCCESS;
		goto DONE;
	}

	if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE){
		/* Get link speed */
		// First check for 1000Mbps
		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_MSCR, &mscr));
		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_MSSR, &mssr));
		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_AN_ADV, &regADV));
		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_AN_LPA, &regLPA));
		// Get the least common denominator

		CommonDenom = regADV & regLPA;

		if ((mscr & PHY_MSCR_1000FDX)  && // LAN7500 can only do 1000T-FD
			(mssr & PHY_MSSR_LP_1000FDX))
		{
			// Master/Slave state of the PHY
			if (mssr & PHY_MSSR_CONFIG_RESO) // Master
			{
				adapterData->actualLinkMode = LINK_SPEED_MASTER_1000FD;
			}
			else //Slave
			{
				adapterData->actualLinkMode = LINK_SPEED_SLAVE_1000FD;
			}
		}
		else
		{
			if(CommonDenom & PHY_AN_ADV_100FDX){
				adapterData->actualLinkMode = LINK_SPEED_100FD;
			}else if(CommonDenom & PHY_AN_ADV_100HDX){
				adapterData->actualLinkMode = LINK_SPEED_100HD;
			}else if(CommonDenom & PHY_AN_ADV_10FDX){
				adapterData->actualLinkMode = LINK_SPEED_10FD;
			}else if(CommonDenom & PHY_AN_ADV_10HDX){
				adapterData->actualLinkMode = LINK_SPEED_10HD;
			}
		}

		//Get flow control

                flow_index = (
                                //bit0 <== LSP
                                ((!!(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE)) << 0) |
                                //bit1 <== LASP
                                ((!!(adapterData->LinkAdvertisedCapabilites & LINK_ASYMMETRIC_PAUSE))<< 1) |
                                //bit2 <== RSP
                                ((!!(regLPA & PHY_AN_ADV_PAUSE)) << 2) |
                                //bit3 <== RASP
                                ((!!(regLPA & PHY_AN_ADV_ASYMP)) << 3)
                             );
                flow_mode = pause_mode_table[flow_index];
                adapterData->bGeneratePause = !!(flow_mode & (1 << 1)); //bit1 is enable pause Tx 
                adapterData->bRespondToPause = !!(flow_mode & (1 << 0)); //bit0 is enable pause Rx 
#if 0
		tadapterData->bGeneratePause = FALSE;
		adapterData->bRespondToPause = FALSE;

		if(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE){
			if(regLPA & PHY_AN_ADV_PAUSE){
				adapterData->bGeneratePause = TRUE;
				adapterData->bRespondToPause = TRUE;
			}else if(regLPA & LINK_ASYMMETRIC_PAUSE){
				adapterData->bGeneratePause = FALSE;
				adapterData->bRespondToPause = TRUE;
			}
		}else if(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE){
			if(regLPA & PHY_AN_ADV_PAUSE){
				adapterData->bGeneratePause = TRUE;
				adapterData->bRespondToPause = FALSE;
			}
		}
		if(adapterData->actualLinkMode == LINK_SPEED_100HD || adapterData->actualLinkMode == LINK_SPEED_10HD){
			adapterData->bGeneratePause = FALSE;
			adapterData->bRespondToPause = FALSE;
		}
#endif
	}else{
		adapterData->actualLinkMode = adapterData->LinkAdvertisedCapabilites & LINK_SPEED_ALL;
	}

	SMSC_DEBUG(DBG_LINK,"<----------out of phy_get_link_mode");

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine updates link mode.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int phy_update_link_mode(struct usbnet *dev)
{
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	int retVal = SMSC7500_FAIL;
	u32 regLPA, mssr;
	u32 oldLinkMode = adapterData->actualLinkMode;

	SMSC_DEBUG(DBG_LINK,"---------->in phy_update_link_mode");

	retVal = phy_get_link_mode(dev);
	if(retVal != SMSC7500_SUCCESS)goto DONE;

	if(oldLinkMode != adapterData->actualLinkMode){

		if(adapterData->actualLinkMode != LINK_OFF){

			/* PHY Rx clock workaround */
			if (adapterData->actualLinkMode == LINK_SPEED_MASTER_1000FD ||
				adapterData->actualLinkMode == LINK_SPEED_SLAVE_1000FD) {
				adapterData->applyPhyRxClkWorkaround = TRUE;
			} else if ((adapterData->applyPhyRxClkWorkaround == TRUE) &&
				(adapterData->actualLinkMode == LINK_SPEED_10HD ||
				adapterData->actualLinkMode == LINK_SPEED_10FD)) {

				Phy_reset(dev);
				if (!Phy_Initialize(dev))
					return SMSC7500_FAIL;
				phy_set_link(dev, adapterData->linkRequest);
				adapterData->applyPhyRxClkWorkaround = FALSE;
				return 0;
			}

			switch(adapterData->actualLinkMode){
			case LINK_SPEED_10HD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 10Mbps HD");
				break;
			case LINK_SPEED_10FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 10Mbps FD");
				break;
			case LINK_SPEED_100HD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 100Mbps HD");

				break;
			case LINK_SPEED_100FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at 100Mbps FD");
				break;
			case LINK_SPEED_MASTER_1000FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at MASTER 1000Mbps FD");
				break;
			case LINK_SPEED_SLAVE_1000FD:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at SLAVE 1000Mbps FD");
				break;
			default:
				SMSC_TRACE(DBG_LINK_CHANGE,"Link is now UP at Unknown Link Speed, actualLinkMode=0x%08X",
					adapterData->actualLinkMode);
				break;
			}
			
			set_flow_control(dev, adapterData->bGeneratePause, adapterData->bRespondToPause);

			if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE){
				CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_AN_LPA, &regLPA));
				CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_MSSR, &mssr));

				SMSC_TRACE(DBG_LINK_CHANGE,"SMSC7500: %s,%s,%s,%s,%s,%s,%s",
					(adapterData->LinkAdvertisedCapabilites & LINK_ASYMMETRIC_PAUSE)?"ASYMP":"     ",
					(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE)?"SYMP ":"     ",
					(adapterData->LinkAdvertisedCapabilites & (LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD) )?"1000FD":"     ",
					(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100FD)?"100FD":"     ",
					(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100HD)?"100HD":"     ",
					(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10FD)?"10FD ":"     ",
					(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10HD)?"10HD ":"     ");

				SMSC_TRACE(DBG_LINK_CHANGE,"Partner: %s,%s,%s,%s,%s,%s,%s",
					(regLPA & PHY_AN_LPA_ASYMP)?"ASYMP":"     ",
					(regLPA & PHY_AN_LPA_PAUSE)?"SYMP ":"     ",
					(mssr & PHY_MSSR_LP_1000FDX)?"1000FD":"     ",
					(regLPA & PHY_AN_LPA_100FDX)?"100FD":"     ",
					(regLPA & PHY_AN_LPA_100HDX)?"100HD":"     ",
					(regLPA & PHY_AN_LPA_10FDX)?"10FD ":"     ",
					(regLPA & PHY_AN_LPA_10HDX)?"10HD ":"     ");
			}

			netif_carrier_on(dev->net);
			tx_wake_queue(dev,TX_QUEUE_SRC_LINK);

		} else {
			SMSC_TRACE(DBG_LINK_CHANGE,"Link is now DOWN");
			tx_stop_queue(dev,TX_QUEUE_SRC_LINK);
			netif_carrier_off(dev->net);
		}
	}
	SMSC_DEBUG(DBG_LINK,"<----------out of phy_update_link_mode");

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine checks link status. Also updates statistics counters.
Arguments:
	ptr 		- Point to device structure usbnet
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int phy_check_link(void * ptr)
{
	struct usbnet		*dev = ptr;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 stats[NO_OF_STAS_CNT];
	u32 dwValue, frames = 0;
    int retVal = SMSC7500_FAIL;
    int i;

    BUG_ON(!dev);
	SMSC_DEBUG(DBG_LINK,"-------->in phy_check_link");

	if(phy_update_link_mode(dev) < 0)return retVal;

	if(dev->suspendFlag & AUTOSUSPEND_DETACH){
		if(!netif_carrier_ok(dev->net)){//Link is down, detach device
			//Set wakeup event
			set_linkdown_wakeup_events(dev, WAKEPHY_ENERGY);

			//Enable net detach
			CHECK_RETURN_STATUS(smsc7500_read_reg(dev,	HW_CFG,&dwValue));
			dwValue |= HW_CFG_SMDET_EN;
			CHECK_RETURN_STATUS(smsc7500_write_reg(dev,	HW_CFG, dwValue));
			dev->suspendFlag &= ~AUTOSUSPEND_DETACH;
			retVal = SMSC7500_SUCCESS;
			goto DONE;
		}
	}

	//Get statistics counters
	if(smsc7500_get_stats(dev, stats) > 0){
		for(i=0; i<NO_OF_STAS_CNT; i++){
			le32_to_cpus(&stats[i]);

			if(adapterData->frameCnt[i].len == LEN_20){
				if(stats[i] >= adapterData->preFrameCnt[i]){
					frames = stats[i] - adapterData->preFrameCnt[i];
				}else{//Rollover
					frames = 0x100000 - adapterData->preFrameCnt[i] + stats[i];
				}
				
				if((i == rxFcsErrors)||
				   (i == rxAlignmentErrors)||
				   (i == rxFragmentErrors) ||
				   (i == rxJabberErrors)||
				   (i == rxRuntFrameErrors)||
				   (i == rxFrameTooLongError)) {			   
				   		dev->stats.rx_errors += frames;
					}
				if(i ==rxAlignmentErrors) dev->stats.rx_frame_errors += frames;
				if(i ==rxFcsErrors) dev->stats.rx_crc_errors += frames;
				
				adapterData->frameCnt[i].cnt += frames;
			}else{
				adapterData->frameCnt[i].cnt = stats[i];
			}
			adapterData->preFrameCnt[i] = stats[i];

		}
	}

	if( (!(dev->StopLinkPolling)) && (!timer_pending(&dev->LinkPollingTimer))) {
		dev->LinkPollingTimer.expires=jiffies+HZ;
		add_timer(&(dev->LinkPollingTimer));
	}
	SMSC_DEBUG(DBG_LINK,"<---------out of phy_check_link");
	retVal = SMSC7500_SUCCESS;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine sets link mode.
Arguments:
	dev 			- Point to device structure usbnet
	dwLinkRequest	- Link mode will be set
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int phy_set_link(struct usbnet *dev, u32 dwLinkRequest)
{
	u32 dwValue = 0;
	u16 bcr = 0;
	int retVal = SMSC7500_FAIL;
	PADAPTER_DATA adapterData = (PADAPTER_DATA)(dev->data[0]);

	SMSC_DEBUG(DBG_LINK,"--------->in phy_set_link");

	adapterData->LinkAdvertisedCapabilites = dwLinkRequest;

	if(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE){
		adapterData->bGeneratePause = TRUE;
		adapterData->bRespondToPause = TRUE;
	}
	else if(adapterData->LinkAdvertisedCapabilites & LINK_ASYMMETRIC_PAUSE){
		adapterData->bGeneratePause = TRUE;
		adapterData->bRespondToPause = FALSE;
	}else{
		adapterData->bGeneratePause = FALSE;
		adapterData->bRespondToPause = FALSE;
	}

	if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) {

		if((dev->udev->speed != USB_SPEED_HIGH)) {//If it is not usb high speed, disable high speed advertisement
			adapterData->LinkAdvertisedCapabilites &= ~(LINK_SPEED_100HD | LINK_SPEED_100FD | LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD);

			if(!(adapterData->LinkAdvertisedCapabilites & (LINK_SPEED_10FD | LINK_SPEED_10HD))){//If no speed setting, forced to 10fd
				adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_10FD;
			}
		}

		//If no speed setting, forced to 100fd
		if(!(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_ALL)){
			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_100FD;
		}

		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_AN_ADV,&dwValue));
		dwValue &= ~PHY_AN_ADV_ALL_CAPS;
//                printk("%s():%d:Virgin value is %x\n", __FUNCTION__, __LINE__, dwValue);

		if(adapterData->LinkAdvertisedCapabilites & LINK_ASYMMETRIC_PAUSE) {
			dwValue |= PHY_AN_ADV_ASYMP;
		}
		if(adapterData->LinkAdvertisedCapabilites & LINK_SYMMETRIC_PAUSE) {
			dwValue |= PHY_AN_ADV_PAUSE;
		}

		if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10HD) {
			dwValue |= PHY_AN_ADV_10HDX;
		}
		if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10FD) {
			dwValue |= PHY_AN_ADV_10FDX;
		}

		if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100HD) {
			dwValue |= PHY_AN_ADV_100HDX;
		}
		if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100FD) {
			dwValue |= PHY_AN_ADV_100FDX;
		}

                //printk("%s():%d:PHY_AN_ADV value is %x\n", __FUNCTION__, __LINE__, dwValue);

		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_AN_ADV, dwValue));

		// 1000Base-T Capability
		// Setup Advertisement register
		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_MSCR,&dwValue));

		if (adapterData->LinkAdvertisedCapabilites & (LINK_SPEED_MASTER_1000FD | LINK_SPEED_MASTER_1000FD))
		{
			// Only 1000BASE-T FD, Bothwell can not operate in 1000BASE-T HD
			dwValue &= ~PHY_MSCR_1000HDX;
			dwValue |= PHY_MSCR_1000FDX;
		}else{
			dwValue &= ~(PHY_MSCR_1000HDX | PHY_MSCR_1000FDX);
		}
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_MSCR, dwValue));

		// begin to establish link
		dwValue = PHY_BCR_AUTO_NEG_ENABLE | PHY_BCR_RESTART_AUTO_NEG;
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_BCR, dwValue));

	} else {//Forced link mode

		//Only force to one link mode
		dwValue = adapterData->LinkAdvertisedCapabilites;
		adapterData->LinkAdvertisedCapabilites &= ~(LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD | LINK_SPEED_100FD | LINK_SPEED_100HD | LINK_SPEED_10FD | LINK_SPEED_10HD);
		bcr = 0;
		if(dwValue & LINK_SPEED_MASTER_1000FD){
			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_MASTER_1000FD;

			bcr |= (PHY_BCR_SPEED_SEL | PHY_BCR_DUPLEX_MODE);

			CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_MSCR, &dwValue));

			/* Force Master Manual Config */
			dwValue &= ~PHY_MSCR_1000HDX;
			dwValue |= PHY_MSCR_MANUAL_EN;
			dwValue |= PHY_MSCR_MANUAL_VALUE;

			CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_MSCR, dwValue));

		} else if(dwValue & LINK_SPEED_SLAVE_1000FD){
			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_SLAVE_1000FD;

			bcr |= (PHY_BCR_SPEED_SEL | PHY_BCR_DUPLEX_MODE);

			CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_MSCR, &dwValue));

			/* Force Slave Manual Config */
			dwValue &= ~PHY_MSCR_1000HDX;
			dwValue |= PHY_MSCR_MANUAL_EN;
			dwValue &= ~PHY_MSCR_MANUAL_VALUE;

			CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_MSCR, dwValue));

		}else if(dwValue & LINK_SPEED_100FD) {

			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_100FD;
			bcr |= (PHY_BCR_SPEED_SELECT | PHY_BCR_DUPLEX_MODE);


		} else if(dwValue & LINK_SPEED_100HD) {

			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_100HD;
			bcr = PHY_BCR_SPEED_SELECT;

		} else if(dwValue & LINK_SPEED_10FD) {

			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_10FD;
			bcr |= PHY_BCR_DUPLEX_MODE;

		} else if(dwValue & LINK_SPEED_10HD) {
			adapterData->LinkAdvertisedCapabilites |= LINK_SPEED_10HD;
		}

        // Force the link partner to restart autonegotiation by dropping
        // the link (put the PHY into loopback mode)
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_BCR, bcr | PHY_BCR_LOOPBACK));
		msleep(1);         /* wait for 1ms */
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_BCR, bcr));
	}

	adapterData->actualLinkMode = adapterData->LinkAdvertisedCapabilites;

	retVal = set_flow_control(dev, adapterData->bGeneratePause, adapterData->bRespondToPause);

	SMSC_DEBUG(DBG_LINK,"<---------out of phy_set_link");

DONE:
    return retVal;
}

static int set_flow_control(struct usbnet *dev, BOOLEAN txEn, BOOLEAN rxEn)
{
	int retVal = SMSC7500_FAIL;
	u32 dwValue = 0;
	PADAPTER_DATA adapterData = (PADAPTER_DATA)(dev->data[0]);

// Setup Flow Control options
    if (txEn)
    {
        if (adapterData->fctFlowReg == 0)
        {
            adapterData->fctFlowReg = (((adapterData->rxFifoSize * 2)/(10 * 512)) & 0x7FUL);  // Set Hi Water Mark to 20% of the Rx Fifo Size (Used Space)
            adapterData->fctFlowReg <<= 8UL;
            adapterData->fctFlowReg |= (((adapterData->rxFifoSize * 8)/(10 * 512)) & 0x7FUL); // Set Lo Water Mark to 80% of the Rx Fifo Size (Used Space)
        }

        dwValue |= (FLOW_TX_FCEN | 0xFFFF);
    }else{
    	adapterData->fctFlowReg = 0;
    }

    if (rxEn){
    	dwValue |= FLOW_RX_FCEN;
    }

    /* Set FLOW and FCT_FLOW registers  */
    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, FLOW, dwValue));
    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, FCT_FLOW, adapterData->fctFlowReg));

    retVal = SMSC7500_SUCCESS;
 DONE:
	 return retVal;
}

/**************************************************************************************
Routine Description:
    This routine sets auto mdix.
Arguments:
	dev 			- Point to device structure usbnet
	wAutoMdix		- desired setting
Return Value:
	SMSC7500_SUCCESS 	- Success.
	SMSC7500_FAIL		- Operation fails.
***********************************************************************************/
static int phy_set_automdix(struct usbnet *dev, u16 wAutoMdix)
{
	int retVal = SMSC7500_FAIL;
	u32 dwTemp;

	CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_MODE_CSR, &dwTemp));
	if (wAutoMdix == AMDIX_DISABLE_CROSSOVER) {
		dwTemp |= PHY_MODE_CSR_AUTO_MDIX_DIS;	
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_MODE_CSR, dwTemp));		

		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_EXT_CSR, &dwTemp));
		dwTemp |= (PHY_EXT_CSR_MDIX_AB | PHY_EXT_CSR_MDIX_CD);
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_EXT_CSR, dwTemp));		

		SMSC_TRACE(DBG_INIT,"AutoMdix is disabled, use crossover cable\n");
	} else if (wAutoMdix == AMDIX_DISABLE_STRAIGHT) {
		dwTemp |= PHY_MODE_CSR_AUTO_MDIX_DIS;	
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_MODE_CSR, dwTemp));		

		CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_EXT_CSR, &dwTemp));
		dwTemp &= (~(PHY_EXT_CSR_MDIX_AB | PHY_EXT_CSR_MDIX_CD));
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_EXT_CSR, dwTemp));		

		SMSC_TRACE(DBG_INIT,"AutoMdix is disabled, use straight cable\n");
	} else {
		dwTemp &= (~PHY_MODE_CSR_AUTO_MDIX_DIS);	
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_MODE_CSR, dwTemp));		
	}
	retVal = SMSC7500_SUCCESS;
DONE:
	return retVal;
}

static int shape_phy(struct usbnet *dev)
{
        int delay = 500;
        int retVal = SMSC7500_FAIL;
        //set bank3
        CHECK_RETURN_STATUS(smsc7500_write_phy(dev, 23, (0xFF)));
        CHECK_RETURN_STATUS(smsc7500_write_phy(dev, 20, (7 | (1 << 14))));
        do
        {
                u32 reg;
                udelay(100);
                CHECK_RETURN_STATUS(smsc7500_read_phy(dev, 20, &reg));
                if(!(reg & (1 << 14)))
                {
                        break;
                }
        }while(--delay);
        if(!delay)
        {
                SMSC_WARNING("PHY shape timeout\n");
        }
    retVal = SMSC7500_SUCCESS;
DONE:
        return retVal;

}
/**************************************************************************************
Routine Description:
    This routine initializes PHY.
Arguments:
	dev 			- Point to device structure usbnet
Return Value:
	TRUE		 	- Success.
	FALSE			- Operation fails.
***********************************************************************************/
static BOOLEAN Phy_Initialize(struct usbnet *dev)
{
	BOOLEAN result=FALSE, bConfigureAutoMdix = FALSE;
	u32 dwTemp=0,dwValue;
	//u32 dwPhyBcr;

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	SMSC_DEBUG(DBG_INIT,"-->Phy_Initialize");

    CHECK_RETURN_STATUS(smsc7500_read_reg(dev, HW_CFG, &dwValue));

    //No external PHY support
	SMSC_DEBUG(DBG_INIT,"using internal PHY ");
	adapterData->dwPhyAddress=1;
    bConfigureAutoMdix = TRUE;

	//Power up PHY
	if(set_reg_bits(dev, PMT_CTL, PMT_CTL_PHY_PWRUP) < 0){
		SMSC_WARNING("Failed to access PMT_CTL");
		goto DONE;
	}

	CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_ID2,&dwTemp));

	adapterData->bPhyRev 	= dwTemp & PHY_ID2_REV;
	adapterData->bPhyModel 	= (dwTemp & PHY_ID2_MODEL) >> PHY_ID2_MODEL_SHIFT;
	adapterData->dwPhyId 	= (dwTemp & PHY_ID2_ID) >> PHY_ID2_ID_SHIFT;

	CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_ID1,&dwTemp));
	adapterData->dwPhyId	|= ((dwTemp&(0x0000FFFFUL))<<2);


	SMSC_DEBUG(DBG_INIT,"dwPhyId==0x%08X,bPhyModel==0x%02X,bPhyRev==0x%02X",
		adapterData->dwPhyId,
		adapterData->bPhyModel,
		adapterData->bPhyRev);

	adapterData->actualLinkMode = LINK_INIT;

	CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_EXT_CSR, PHY_EXT_CSR_LED_MODE));
	CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_EXT_CSR, PHY_EXT_CSR_LED_MODE_4));
	if (bConfigureAutoMdix)
	{
		phy_set_automdix(dev, (u16)auto_mdix);
	}

	shape_phy(dev);

	result=TRUE;
DONE:
	SMSC_DEBUG(DBG_INIT,"<--Phy_Initialize, result=%s\n",result?"TRUE":"FALSE");
	return result;

}
static void smsc7500_get_regs(struct net_device *net, struct ethtool_regs * regs, 
                void *buf)
{

#if 0
	struct usbnet *dev = netdev_priv(net);
        PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
        unsigned int i, j = 0;

        u32 *data = buf;
        //__TODO__
#endif

}
static int smsc7500_get_regs_len(struct net_device * net)
{
        /* all 7500 registers + phy registers */
        return ETHTOOL_REG_SIZE;

}
/**************************************************************************************
Routine Description:
    This routine is used for ethtool to get link settings.
Arguments:
	net 			- Point to net device structure
	cmd				- ethtool command
Return Value:
***********************************************************************************/
static int smsc7500_get_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
			cmd->supported=
				SUPPORTED_10baseT_Half |
				SUPPORTED_10baseT_Full |
				SUPPORTED_100baseT_Half |
				SUPPORTED_100baseT_Full |
				SUPPORTED_1000baseT_Full |
				SUPPORTED_Autoneg |
				SUPPORTED_MII;
			cmd->advertising=ADVERTISED_MII;

			if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10HD)
				cmd->advertising |= ADVERTISED_10baseT_Half;
			if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_10FD)
				cmd->advertising |= ADVERTISED_10baseT_Full;
			if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100HD)
				cmd->advertising |= ADVERTISED_100baseT_Half;
			if(adapterData->LinkAdvertisedCapabilites & LINK_SPEED_100FD)
				cmd->advertising |= ADVERTISED_100baseT_Full;
			if(adapterData->LinkAdvertisedCapabilites & (LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD))
				cmd->advertising |= ADVERTISED_1000baseT_Full;
			if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) {
				cmd->advertising |= ADVERTISED_Autoneg;
				cmd->autoneg = AUTONEG_ENABLE;
			} else cmd->autoneg = AUTONEG_DISABLE;

			if(adapterData->actualLinkMode & (LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD))
				cmd->speed = SPEED_1000;
			else if(adapterData->actualLinkMode & (LINK_SPEED_100HD|LINK_SPEED_100FD))
				cmd->speed = SPEED_100;
			else cmd->speed = SPEED_10;

			if(adapterData->actualLinkMode & (LINK_SPEED_10FD | LINK_SPEED_100FD | LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD))
				cmd->duplex=DUPLEX_FULL;
			else cmd->duplex=DUPLEX_HALF;

			cmd->port=PORT_MII;
			cmd->phy_address=(u8)adapterData->dwPhyAddress;
			cmd->transceiver=XCVR_INTERNAL;
			cmd->maxtxpkt=0;
			cmd->maxrxpkt=0;


			return 0;

}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to set link mode.
Arguments:
	net 			- Point to net device structure
	cmd				- ethtool command
Return Value:
***********************************************************************************/
static int smsc7500_set_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	int result=-EFAULT;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 linkRequest = 0;

	if (cmd->autoneg == AUTONEG_ENABLE) {
		u32 mask = (ADVERTISED_1000baseT_Half |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full);

		if (cmd->advertising & ~mask)
			return -EINVAL;

		if (cmd->advertising == ADVERTISED_1000baseT_Half)
			return -EOPNOTSUPP;

		if (cmd->advertising & ADVERTISED_1000baseT_Full)
			linkRequest |= (LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD);
		if (cmd->advertising & ADVERTISED_100baseT_Full)
			linkRequest |= LINK_SPEED_100FD;
		if (cmd->advertising & ADVERTISED_100baseT_Half)
			linkRequest |= LINK_SPEED_100HD;
		if (cmd->advertising & ADVERTISED_10baseT_Full)
			linkRequest |= LINK_SPEED_10FD;
		if (cmd->advertising & ADVERTISED_10baseT_Half)
			linkRequest |= LINK_SPEED_10HD;
		linkRequest |= LINK_AUTO_NEGOTIATE;
	} else {

		if (!((cmd->speed == SPEED_10) || (cmd->speed == SPEED_100) || (cmd->speed == SPEED_1000)))
			return -EOPNOTSUPP;

		if(cmd->speed == SPEED_1000 && cmd->duplex == DUPLEX_HALF)
			return -EOPNOTSUPP;

		if(cmd->speed == SPEED_1000){
			linkRequest |= LINK_SPEED_MASTER_1000FD;
		}else if(cmd->speed == SPEED_100){
			if(cmd->duplex == DUPLEX_FULL){
				linkRequest |= LINK_SPEED_100FD;
			}else{
				linkRequest |= LINK_SPEED_100HD;
			}
		}else{
			if(cmd->duplex == DUPLEX_FULL){
				linkRequest |= LINK_SPEED_10FD;
			}else{
				linkRequest |= LINK_SPEED_10HD;
			}
		}
	}

	linkRequest |= adapterData->linkRequest & (LINK_SYMMETRIC_PAUSE | LINK_ASYMMETRIC_PAUSE); //Keep flow control setting
	adapterData->linkRequest = linkRequest;

	if(phy_set_link(dev, adapterData->linkRequest) == SMSC7500_SUCCESS){
		result=0;
	}

	return result;
}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to get driver information.
Arguments:
	net 			- Point to net device structure
	info			- Point to ethtool drvinfo
Return Value:
***********************************************************************************/
void smsc7500_get_drvinfo (struct net_device *net, struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

			strcpy(info->driver,"Smsc7500");
			memset(&info->version,0,sizeof(info->version));
			sprintf(info->version,"%lX.%02lX.%02lX",
				(DRIVER_VERSION>>16),(DRIVER_VERSION>>8)&0xFF,(DRIVER_VERSION&0xFFUL));
			memset(&info->fw_version,0,sizeof(info->fw_version));
			sprintf(info->fw_version,"%lu",(adapterData->dwIdRev)&0xFFFFUL);
			memset(&info->bus_info,0,sizeof(info->bus_info));
			memset(&info->reserved1,0,sizeof(info->reserved1));
			memset(&info->reserved2,0,sizeof(info->reserved2));
			info->n_stats=0;
			info->testinfo_len=0;
			info->eedump_len=0;
			info->regdump_len=ETHTOOL_REG_SIZE;

}

static int smsc7500_nway_reset(struct net_device *net)
{
        //__TODO__

	struct usbnet *dev = netdev_priv(net);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

		if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) {
			//Restart autonegotiate with previous advertisement setting
			smsc7500_write_phy(dev,PHY_BCR, PHY_BCR_AUTO_NEG_ENABLE |
                                                        PHY_BCR_RESTART_AUTO_NEG);
			return  0;
		}
                return -ENODEV;

}
/**************************************************************************************
Routine Description:
    This routine is used for ethtool to get link status.
Arguments:
	net 		- Point to net device structure
Return Value:
	1			- Link up
	0			- Link off
***********************************************************************************/
static u32 smsc7500_get_link (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	u32 wRegBSR;


	CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &wRegBSR));
	CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &wRegBSR));
	if(!(wRegBSR & PHY_BSR_LINK_STATUS)) {//Link down
		return 0;
	} else {	
		return 1;
	}
DONE:
	return 0;
}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to get message level.
Arguments:
	net 		- Point to net device structure
Return Value:
	1			- message enabled
	0			- message disabled
***********************************************************************************/
static u32 smsc7500_get_msglevel (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return dev->msg_enable;
}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to set message level.
Arguments:
	net 		- Point to net device structure
	level		- Desired message level
Return Value:
***********************************************************************************/
static void smsc7500_set_msglevel (struct net_device *net, u32 level)
{
	struct usbnet *dev = netdev_priv(net);

	dev->msg_enable = level;
}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to get wol setting.
Arguments:
	net 		- Point to net device structure
	wolinfo		- structure to hold wol info
Return Value:
***********************************************************************************/
static void smsc7500_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	wolinfo->supported=(WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC);
	wolinfo->wolopts= adapterData->WolWakeupOpts;
}

/**************************************************************************************
Routine Description:
    This routine is used for ethtool to set wol setting.
Arguments:
	net 		- Point to net device structure
	wolinfo		- structure to hold wol info
Return Value:
***********************************************************************************/
static int smsc7500_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	int result=-EFAULT;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	adapterData->WolWakeupOpts = wolinfo->wolopts;
	result=0;

	return result;
}

/**************************************************************************************
Routine Description:
    This routine returns EEPROM size.
Arguments:
	net 		- Point to net device structure
Return Value:
	EEPROM size
***********************************************************************************/
static int smsc7500_get_eeprom_len(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	return adapterData->eepromSize;
}

/**************************************************************************************
Routine Description:
    This routine reads EEPROM and fill buffer "data".
Arguments:
	netdev 		- Point to net device structure
	ee			- ethtool_eeprom structure
	data		- buffer
Return Value:
	0			- Success
	<0			- Fail
***********************************************************************************/
static int smsc7500_get_eeprom(struct net_device *netdev, struct ethtool_eeprom *ee, u8 *data)
{
	struct usbnet *dev=netdev_priv(netdev);
	int offset = ee->offset;
	int len = ee->len;
	int result = 0;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	ee->magic = LAN7500_EEPROM_MAGIC;

	if(len == 0){
		return 0;
	}

	if(offset + len > adapterData->eepromSize){
		SMSC_WARNING("EEPROM address is out of range");
		result = -EINVAL;
	}else{
		if(smsc7500_read_eeprom(dev, offset, len,  data) < 0){
			result = -EFAULT;
		}
	}

	return result;
}

/**************************************************************************************
Routine Description:
    This routine writess EEPROM.
Arguments:
	netdev 		- Point to net device structure
	ee			- ethtool_eeprom structure
	data		- buffer
Return Value:
	0			- Success
	<0			- Fail
***********************************************************************************/
static int smsc7500_set_eeprom(struct net_device *netdev, struct ethtool_eeprom *ee, u8 *data)
{
	struct usbnet *dev=netdev_priv(netdev);
	int offset = ee->offset;
	int len = ee->len;
	int result = 0;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	if(len == 0){
		return 0;
	}

	if(offset + len > adapterData->eepromSize){
		SMSC_WARNING("EEPROM address is out of range");
		result = -EINVAL;
		return result;
	}

	if(ee->magic != LAN7500_EEPROM_MAGIC){
		SMSC_WARNING("EEPROM: magic value mismatch, writing fail, magic = 0x%x", ee->magic);
		result = -EFAULT;
		return result;
	}

	if(smsc7500_write_eeprom(dev, offset, len,  data) < 0){
		result=-EFAULT;
	}

	return result;
}

/**************************************************************************************
Routine Description:
    This routine calculates physical EEPROM size.
Arguments:
	dev 		- Point to device structure usbnet
Return Value:
	0			- EEPROM doesn't exist
	>0			- EEPROM size
***********************************************************************************/
static int smsc7500_eeprom_size(struct usbnet *dev)
{
#define CHECK_SIZE	4
	u32 dwValue;
	int size = 0;
	int i;
	char save[CHECK_SIZE+1];
	char saveEach[CHECK_SIZE+1];

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, E2P_CMD, &dwValue));
    if (!(dwValue & E2P_CMD_LOADED))
    {
		size = 0;
		goto DONE;
    }

	if(smsc7500_read_eeprom(dev, 0, CHECK_SIZE,  save) < 0){//Save first 4 bytes
		goto DONE;
	}

	for(i=128; i<=MAX_EEPROM_SIZE; i+=128){
		if(smsc7500_read_eeprom(dev, i, CHECK_SIZE,  saveEach) < 0){
			goto DONE;
		}
		if(!strncmp(save, saveEach, CHECK_SIZE)){
			size = i;
			break;
		}
	}

DONE:
	return size;

}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
static u32 smsc7500_get_rx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	return adapterData->UseRxCsum;
}

static int smsc7500_set_rx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 data; 
	int retval = 0;

	if(down_interruptible(&adapterData->RxFilterLock)) {
		return -EINTR;
	}

	adapterData->UseRxCsum = !!val;

	CHECK_RETURN_STATUS(retval = smsc7500_read_reg(dev, REF_CTL, &data));
	if (adapterData->UseRxCsum) {
		data |= REF_CTL_TCPUDP_CKM | REF_CTL_IP_CKM; //Enable Rx checksum offload
	} else {
		data &= ~(REF_CTL_TCPUDP_CKM | REF_CTL_IP_CKM);
	}

	CHECK_RETURN_STATUS(retval = smsc7500_write_reg(dev, REF_CTL, data));
DONE:
	up(&adapterData->RxFilterLock);
	if (retval >= 0)
		return 0;
	else
		return -1;
}

static int smsc7500_set_tx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	adapterData->UseTxCsum = !!val;

	if(adapterData->UseTxCsum){
		dev->net->features |= NETIF_F_HW_CSUM;
	} else {
		dev->net->features &= ~NETIF_F_HW_CSUM;
	}
	return 0;
}

static int smsc7500_set_tso(struct net_device *netdev, u32 data)
{
	if (data)
		netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;
	else
		netdev->features &= ~(NETIF_F_TSO | NETIF_F_TSO6);

	return 0;
}

static int smsc7500_set_sg(struct net_device *netdev, u32 data)
{
	if(data) {
		netdev->features |= NETIF_F_SG | NETIF_F_FRAGLIST;
	} else {
		netdev->features &= ~(NETIF_F_SG | NETIF_F_FRAGLIST);
	}
	return 0;
}
#endif 

static void smsc7500_get_ethtool_stats(struct net_device *netdev,struct ethtool_stats *stats, u64 *data)
{
	
	struct usbnet *dev = netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 frames=0;
	u32 statsCounter[NO_OF_STAS_CNT];
	int i=0;
	
	//Get statistics counters
		if(smsc7500_get_stats(dev, statsCounter) > 0){
			for(i=0; i<NO_OF_STAS_CNT; i++){
				le32_to_cpus(&statsCounter[i]);
	
				if(adapterData->frameCnt[i].len == LEN_20){
					if(statsCounter[i] >= adapterData->preFrameCnt[i]){
						frames = statsCounter[i] - adapterData->preFrameCnt[i];
					}else{//Rollover
						frames = 0x100000 - adapterData->preFrameCnt[i] + statsCounter[i];
					}
					
					if((i == rxFcsErrors)||
					   (i == rxAlignmentErrors)||
					   (i == rxFragmentErrors) ||
					   (i == rxJabberErrors)||
					   (i == rxRuntFrameErrors)||
					   (i == rxFrameTooLongError)) {			   
							dev->stats.rx_errors += frames;
						}
					if(i ==rxAlignmentErrors) dev->stats.rx_frame_errors += frames;
					if(i ==rxFcsErrors) dev->stats.rx_crc_errors += frames;
					
					adapterData->frameCnt[i].cnt += frames;
				}else{
					adapterData->frameCnt[i].cnt = statsCounter[i];
				}
				adapterData->preFrameCnt[i] = statsCounter[i];
	
			}
		}
		
	data[0]=dev->stats.rx_packets;
	data[1]=dev->stats.tx_packets;
	data[2]=dev->stats.rx_bytes;
	data[3]=dev->stats.tx_bytes;
	data[4]=dev->stats.rx_errors;
	data[5]=dev->stats.tx_errors;
	data[6]=dev->stats.tx_dropped;
	data[7]=dev->stats.rx_dropped;
	data[8]=dev->stats.rx_length_errors;
	data[9]=dev->stats.rx_over_errors;
	data[10]=dev->stats.rx_crc_errors;
	data[11]=dev->stats.rx_frame_errors;
	data[12]=adapterData->frameCnt[rxFragmentErrors].cnt;
	data[13]=adapterData->frameCnt[rxJabberErrors].cnt;
	data[14]=adapterData->frameCnt[rxRuntFrameErrors].cnt;
	data[15]=adapterData->frameCnt[rxFrameTooLongError].cnt;
	data[16]=adapterData->frameCnt[rxDroppedFrames].cnt;
	data[17]=adapterData->frameCnt[rxUnicastFrames].cnt;
	data[18]=adapterData->frameCnt[rxBroadcastFrames].cnt;
	data[19]=adapterData->frameCnt[rxMulticastFrames].cnt;
	data[20]=adapterData->frameCnt[rxPauseFrames].cnt;
	data[21]=adapterData->frameCnt[txFcsErrors].cnt;
	data[22]=adapterData->frameCnt[txExcessDeferralErrors].cnt;
	data[23]=adapterData->frameCnt[txCarrierErrors].cnt;
	data[24]=adapterData->frameCnt[txSingleCollisions].cnt;
	data[25]=adapterData->frameCnt[txMultipleCollisions].cnt;
	data[26]=adapterData->frameCnt[txExcessCollision].cnt;
	data[27]=adapterData->frameCnt[txLateCollision].cnt;
	data[28]=adapterData->frameCnt[txUnicastFrames].cnt;
	data[29]=adapterData->frameCnt[txBroadcastFrames].cnt;
	data[30]=adapterData->frameCnt[txMulticastFrames].cnt;
	data[31]=adapterData->frameCnt[txPauseFrames].cnt;	
}

static void smsc7500_get_strings(struct net_device *netdev, u32 stringset,u8 *data)
{
	memcpy(data, ethtool_stats_keys, sizeof(ethtool_stats_keys));
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
static int smsc7500_get_sset_count(struct net_device *netdev) 
{	
	return ARRAY_SIZE(ethtool_stats_keys);
}
#else
static int smsc7500_get_sset_count(struct net_device *netdev, int sset)

{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(ethtool_stats_keys);
	default:
		return -EOPNOTSUPP;
	}
}
#endif


/* We need to override some ethtool_ops so we require our
   own structure so we don't interfere with other usbnet
   devices that may be connected at the same time. */
static struct ethtool_ops smsc7500_ethtool_ops = {
	.get_settings		= smsc7500_get_settings,
	.set_settings		= smsc7500_set_settings,
	.get_drvinfo		= smsc7500_get_drvinfo,
        .get_regs_len           = smsc7500_get_regs_len,
        .get_regs               = smsc7500_get_regs,
	.get_wol		= smsc7500_get_wol,
	.set_wol		= smsc7500_set_wol,
	.get_msglevel		= smsc7500_get_msglevel,
	.set_msglevel		= smsc7500_set_msglevel,
	.nway_reset             = smsc7500_nway_reset,
	.get_link		= smsc7500_get_link,
	.get_eeprom_len		= smsc7500_get_eeprom_len,
	.get_eeprom		= smsc7500_get_eeprom,
	.set_eeprom		= smsc7500_set_eeprom,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
	.get_tx_csum		= ethtool_op_get_tx_csum,
	.set_tx_csum		= smsc7500_set_tx_csum,
	.get_rx_csum		= smsc7500_get_rx_csum,
	.set_rx_csum		= smsc7500_set_rx_csum,
	.set_tso		= smsc7500_set_tso,
	.get_tso		= ethtool_op_get_tso,
	.get_sg                 = ethtool_op_get_sg,
	.set_sg                 = smsc7500_set_sg,
#endif
	.get_strings            = smsc7500_get_strings,
	.get_ethtool_stats      = smsc7500_get_ethtool_stats,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
	.get_stats_count	= smsc7500_get_sset_count,
#else
	.get_sset_count		= smsc7500_get_sset_count,
#endif
	
};

/**************************************************************************************
Routine Description:
    This routine is ioctl implementation.
Arguments:
	netdev 		- Point to net device structure
	ifr			- The ifreq structure
	cmd			- ioctl command
Return Value:
	0				- Success
	-EFAULT			- Fail
	-EOPNOTSUPP		- Unknown command
***********************************************************************************/
static int smsc7500_do_ioctl(
	struct net_device *netdev,
	struct ifreq *ifr,
	int cmd)
{
	int result=0;
	struct usbnet *dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	void __user *userAddr=NULL;

	SMSC_DEBUG(DBG_IOCTL,"---->smsc7500_do_ioctl");
	if(netdev==NULL) {
		SMSC_WARNING("netdev==NULL");
		result=-EFAULT;
		goto DONE;
	}
#if 0
	if(netdev->priv==NULL) {
		SMSC_WARNING("netdev->priv==NULL");
		result=-EFAULT;
		goto DONE;
	}
#endif

	if(ifr==NULL) {
		SMSC_WARNING("ifr==NULL");
		result=-EFAULT;
		goto DONE;
	}
	userAddr=ifr->ifr_data;

	switch(cmd) {

	case SIOCGMIIPHY:
	case SIOCDEVPRIVATE:
		SMSC_DEBUG(DBG_IOCTL,"SIOCGMIIPHY");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			miiData->phy_id=1;
		};
		break;

	case SIOCGMIIREG:
	case SIOCDEVPRIVATE+1:
		SMSC_DEBUG(DBG_IOCTL,"SIOCGMIIREG");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			{
				u32 dwValue;
				if(smsc7500_read_phy(dev,miiData->reg_num,&dwValue) < 0){
                    result = -EFAULT;
                }
				miiData->val_out=(u16)dwValue;
			}
		};break;

	case SIOCSMIIREG:
	case SIOCDEVPRIVATE+2:
		SMSC_DEBUG(DBG_IOCTL,"SIOCSMIIREG");
		if(adapterData->LanInitialized) {
			struct mii_ioctl_data *miiData=
				(struct mii_ioctl_data *)&(ifr->ifr_data);
			{
				u32 dwValue;
				dwValue=miiData->val_in;
				if(smsc7500_write_phy(dev,miiData->reg_num, dwValue) < 0){
                    result = -EFAULT;
                }
			}
		};break;

	case SMSC7500_IOCTL:
		result = smsc7500_private_ioctl(adapterData, dev, (PSMSC7500_IOCTL_DATA)userAddr);
		break;

	default:
		result=-EOPNOTSUPP;
		break;
	}

DONE:

	SMSC_DEBUG(DBG_IOCTL,"<--smsc7500_do_ioctl");
	return result;
}

/**************************************************************************************
Routine Description:
    Private ioctl commands.
Arguments:
	dev 		- Point to usb device structure
	privateData	- private device structure
	ioctlData	- Pass in/out command and parameters
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Fail
***********************************************************************************/
static int smsc7500_private_ioctl(PADAPTER_DATA  privateData, struct usbnet *dev, PSMSC7500_IOCTL_DATA ioctlData)
{
	BOOLEAN success=FALSE;
    int i;
	u32 dwBuf;
    u32 offset;
	if(ioctlData->dwSignature!=SMSC7500_APP_SIGNATURE) {
		goto DONE;
	}

	switch(ioctlData->dwCommand) {
	case COMMAND_GET_SIGNATURE:
		success=TRUE;
		break;
	case COMMAND_GET_CONFIGURATION:	
		ioctlData->Data[0] = DRIVER_VERSION;
		ioctlData->Data[1] = privateData->linkRequest;
		ioctlData->Data[2] = privateData->macAddrHi16;
		ioctlData->Data[3] = privateData->MmacAddrLo32;
		ioctlData->Data[4] = debug_mode;
		ioctlData->Data[5] = privateData->dwIdRev;
		ioctlData->Data[6] = privateData->dwFpgaRev;
		ioctlData->Data[7] = 1;
		ioctlData->Data[8] = privateData->dwPhyId;
		ioctlData->Data[9] = privateData->bPhyModel;
		ioctlData->Data[10]= privateData->bPhyRev;
		ioctlData->Data[11]= privateData->actualLinkMode;
		ioctlData->Data[12]= privateData->eepromSize / 128;
		sprintf(ioctlData->Strng1,"%s, %s",__DATE__,__TIME__);

		success=TRUE;
		break;
	case COMMAND_LAN_GET_REG:
        offset = ioctlData->Data[0];

		if((ioctlData->Data[0] <= MAX_ADDR_OF_REGISTER) && ((ioctlData->Data[0]&0x3UL)==0))
		{
			if(smsc7500_read_reg(dev, offset, &dwBuf) >= 0){
                ioctlData->Data[1] = dwBuf;
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Reading LAN7500 Mem Map Failed");
			goto MEM_MAP_ACCESS_FAILED;
		}
		break;
	case COMMAND_LAN_SET_REG:
		if((ioctlData->Data[0] <= MAX_ADDR_OF_REGISTER) && ((ioctlData->Data[0]&0x3UL)==0))
		{
            offset = ioctlData->Data[0];
            dwBuf = ioctlData->Data[1];
			if(smsc7500_write_reg(dev, offset,  dwBuf) >= 0){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Writing LAN7500 Mem Map Failed");
MEM_MAP_ACCESS_FAILED:
			SMSC_WARNING("  Invalid offset == 0x%08lX",ioctlData->Data[0]);
			if(ioctlData->Data[0] > MAX_ADDR_OF_REGISTER) {
				SMSC_WARNING("    Out of range");
			}
			if(ioctlData->Data[0]&0x3UL) {
				SMSC_WARNING("    Not u32 aligned");
			}
		}
		break;
	case COMMAND_PHY_GET_REG:
		if((ioctlData->Data[0]<32)&&(privateData->LanInitialized)) {
            offset = ioctlData->Data[0];
			if(smsc7500_read_phy(dev,offset, &dwBuf) >= 0){
				success=TRUE;
                ioctlData->Data[1] = dwBuf;
			}
		} else {
			SMSC_WARNING("Reading Phy Register Failed");
			goto PHY_ACCESS_FAILURE;
		}
		break;
	case COMMAND_PHY_SET_REG:
		if((ioctlData->Data[0]<32)&&(privateData->LanInitialized)) {
            offset = ioctlData->Data[0];
            dwBuf = ioctlData->Data[1];
			if(smsc7500_write_phy(dev,offset, dwBuf) >= 0){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Writing Phy Register Failed");
PHY_ACCESS_FAILURE:
			if(!(privateData->LanInitialized)) {
				SMSC_WARNING("  Lan Not Initialized,");
				SMSC_WARNING("    Use ifconfig to bring interface UP");
			}
			if(!(ioctlData->Data[0]<32)) {
				SMSC_WARNING("  Invalid index == 0x%ld",ioctlData->Data[0]);
			}
		}
		break;
    case COMMAND_GET_EEPROM:
    {
        BYTE cBuf;
        offset = ioctlData->Data[0];
        if(offset < privateData->eepromSize) {
            if(smsc7500_read_eeprom(dev,offset, 1, &cBuf) >= 0){
                success=TRUE;
                ioctlData->Data[1] = cBuf;
            }
        } else {
            SMSC_WARNING("Reading EEPROM Failed");
            goto PHY_ACCESS_FAILURE;
        }
    }
        break;
    case COMMAND_SET_EEPROM:
    {
        BYTE cBuf;
        offset = ioctlData->Data[0];
        cBuf = (BYTE)ioctlData->Data[1];
        if(offset < privateData->eepromSize) {
            if(smsc7500_write_eeprom(dev, offset, 1, &cBuf) >= 0){
                success=TRUE;
            }
        } else {
            SMSC_WARNING("Writing EEPROM Failed");
            if(!(offset < privateData->eepromSize)) {
                SMSC_WARNING("  Invalid eeprom offset == 0x%d",offset);
            }
        }
    }
        break;

	case COMMAND_DUMP_LAN_REGS:

        success=TRUE;
        for(i=0; i<NO_OF_SYS_REGISTER; i++){
	        if(smsc7500_read_reg(dev, lanRegMap[i], &dwBuf) < 0){
	            SMSC_WARNING("Failed to read LAN reg 0x%x", (unsigned int)lanRegMap[i]);
                success = FALSE;
	        }else{
	            ioctlData->Data[i] = dwBuf;
	        }
        }
		break;
	case COMMAND_DUMP_PHY_REGS:
		if(privateData->LanInitialized) {
            success=TRUE;
            for(i=0; i<NO_PHY_REGS; i++){
                if(smsc7500_read_phy(dev, phyRegMap[i], &dwBuf) < 0){
                    SMSC_WARNING("Failed to read PHY reg 0x%x", (unsigned int)phyRegMap[i]);
                    success = FALSE;
                }else{
                    ioctlData->Data[i] = dwBuf;
                }
            }
		} else {
			SMSC_WARNING("Phy Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		}
		break;
	case COMMAND_DUMP_EEPROM:
		{
			success=TRUE;

			if(smsc7500_read_eeprom(dev, 0, privateData->eepromSize,  (BYTE*)ioctlData->Data) < 0){
				success=FALSE;
			}
		};break;
	case COMMAND_GET_MAC_ADDRESS:

		if(privateData->LanInitialized) {
			CHECK_RETURN_STATUS(smsc7500_read_reg(dev, RX_ADDRH, &dwBuf));
            ioctlData->Data[0] = dwBuf;
			CHECK_RETURN_STATUS(smsc7500_read_reg(dev, RX_ADDRL, &dwBuf));
            ioctlData->Data[1] = dwBuf;
			success=TRUE;
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		}
		break;

	case COMMAND_SET_MAC_ADDRESS:
		if(privateData->LanInitialized)
		{
			u32 dwLow32=ioctlData->Data[1];
			u32 dwHigh16=ioctlData->Data[0];

			CHECK_RETURN_STATUS(smsc7500_write_reg(dev, RX_ADDRH, dwHigh16));
			CHECK_RETURN_STATUS(smsc7500_write_reg(dev, RX_ADDRL, dwLow32));

		    dev->net->dev_addr[0]=LOBYTE(LOWORD(dwLow32));
		    dev->net->dev_addr[1]=HIBYTE(LOWORD(dwLow32));
		    dev->net->dev_addr[2]=LOBYTE(HIWORD(dwLow32));
		    dev->net->dev_addr[3]=HIBYTE(HIWORD(dwLow32));
		    dev->net->dev_addr[4]=LOBYTE(LOWORD(dwHigh16));
		    dev->net->dev_addr[5]=HIBYTE(LOWORD(dwHigh16));

			success=TRUE;
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;

	case COMMAND_LOAD_MAC_ADDRESS:
		if(privateData->LanInitialized) {
            if(smsc7500_read_eeprom(dev, EEPROM_MAC_OFFSET, 6,  dev->net->dev_addr) == 0){
                dwBuf = dev->net->dev_addr[0] | dev->net->dev_addr[1] << 8 | dev->net->dev_addr[2] << 16 | dev->net->dev_addr[3] << 24;
                ioctlData->Data[1] = dwBuf;
                CHECK_RETURN_STATUS(smsc7500_write_reg(dev, RX_ADDRL, ioctlData->Data[1]));
                dwBuf = dev->net->dev_addr[4] | dev->net->dev_addr[5] << 8;
                ioctlData->Data[0] = dwBuf;
    		    CHECK_RETURN_STATUS(smsc7500_write_reg(dev, RX_ADDRH, ioctlData->Data[0]));

			    success=TRUE;
			} else {
				SMSC_WARNING("Failed to Load Mac Address");
			}
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;
	case COMMAND_SAVE_MAC_ADDRESS:
		if(privateData->LanInitialized) {
            u32 dwLow32 = ioctlData->Data[1];
            u32 dwHigh16 = ioctlData->Data[0];

            cpu_to_le32s((u32*)&dwLow32);
            cpu_to_le32s((u32*)&dwHigh16);

			if((smsc7500_write_eeprom(dev, EEPROM_MAC_OFFSET, 4, (BYTE*)&dwLow32) == 0) &&
                (smsc7500_write_eeprom(dev, EEPROM_MAC_OFFSET+4, 2, (BYTE*)&dwHigh16) == 0)){
				success=TRUE;
			}
		} else {
			SMSC_WARNING("Lan Not Initialized,");
			SMSC_WARNING("  Use ifconfig to bring interface UP");
		};break;

	case COMMAND_SET_DEBUG_MODE:
		debug_mode=ioctlData->Data[0];
		if(debug_mode&0x04UL) {
			CHECK_RETURN_STATUS(smsc7500_write_reg(dev, GPIO_CFG, 0x00670700UL));
			success=TRUE;
		} else {
			CHECK_RETURN_STATUS(smsc7500_write_reg(dev, GPIO_CFG, 0x70070000));
			success=TRUE;
		}
		success=TRUE;
		break;
	case COMMAND_SET_LINK_MODE:
		privateData->linkRequest = ioctlData->Data[0];
		if(privateData->LanInitialized) {
			phy_set_link(dev,privateData->linkRequest);
		}
		success=TRUE;
		break;
	case COMMAND_GET_LINK_MODE:
		ioctlData->Data[0] = privateData->linkRequest;
		success=TRUE;
		break;
	case COMMAND_CHECK_LINK:
		phy_update_link_mode(dev);
		success=TRUE;
		break;

    case COMMAND_GET_ERRORS:

        ioctlData->Data[0] = dev->extra_error_cnts.tx_epipe;
        ioctlData->Data[1] = dev->extra_error_cnts.tx_eproto;
        ioctlData->Data[2] = dev->extra_error_cnts.tx_etimeout;
        ioctlData->Data[3] = dev->extra_error_cnts.tx_eilseq;

        ioctlData->Data[4] = dev->extra_error_cnts.rx_epipe;
        ioctlData->Data[5] = dev->extra_error_cnts.rx_eproto;
        ioctlData->Data[6] = dev->extra_error_cnts.rx_etimeout;
        ioctlData->Data[7] = dev->extra_error_cnts.rx_eilseq;
        ioctlData->Data[8] = dev->extra_error_cnts.rx_eoverflow;

        success = TRUE;
        break;
	case COMMAND_READ_BYTE:
		ioctlData->Data[1]=(*((volatile BYTE *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_READ_WORD:
		ioctlData->Data[1]=(*((volatile u16 *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_READ_DWORD:
		ioctlData->Data[1]=(*((volatile u32 *)(ioctlData->Data[0])));
		success=TRUE;
		break;
	case COMMAND_WRITE_BYTE:
		(*((volatile BYTE *)(ioctlData->Data[0])))=
			((BYTE)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_WRITE_WORD:
		(*((volatile u16 *)(ioctlData->Data[0])))=
			((u16)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_WRITE_DWORD:
		(*((volatile u32 *)(ioctlData->Data[0])))=
			((u32)(ioctlData->Data[1]));
		success=TRUE;
		break;
	case COMMAND_SET_AMDIX_STS:
		auto_mdix=(ioctlData->Data[0]);
		if(privateData->LanInitialized) {
			phy_set_automdix(dev, (u16)auto_mdix);
		}
		success=TRUE;
		break;
	case COMMAND_GET_AMDIX_STS:
		ioctlData->Data[0]=auto_mdix;
		success=TRUE;
		break;
	case COMMAND_DUMP_STATISTICS:
	    success=TRUE;
	    for(i=0; i<NO_OF_STAS_CNT; i++){
	        ioctlData->Data[i] = privateData->frameCnt[i].cnt;
	    }
		break;
	default:break;//make lint happy
	}

DONE:
	if((success)&&(ioctlData!=NULL)) {
		ioctlData->dwSignature=SMSC7500_DRIVER_SIGNATURE;
		return SMSC7500_SUCCESS;
	}
	return SMSC7500_FAIL;

}

static int smsc7500_eth_mac_addr(struct net_device *netdev, void *p)
{
	struct usbnet *dev = netdev_priv(netdev);
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);
	struct sockaddr *addr = p;

        if (netif_running(netdev))
                return -EBUSY;
        if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;
        memcpy(dev->net->dev_addr, addr->sa_data, ETH_ALEN);

	adapterData->MmacAddrLo32 = dev->net->dev_addr[0] | dev->net->dev_addr[1] << 8 |
		dev->net->dev_addr[2] << 16 | dev->net->dev_addr[3] << 24;
	adapterData->macAddrHi16 = dev->net->dev_addr[4] | dev->net->dev_addr[5] << 8;

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
static const struct net_device_ops smsc95xx_netdev_ops =
{
        .ndo_open               = smscusbnet_open,
        .ndo_stop               = smscusbnet_stop,
        .ndo_start_xmit         = smscusbnet_start_xmit,
        .ndo_tx_timeout         = smscusbnet_tx_timeout,
        .ndo_change_mtu         = smscusbnet_change_mtu,
        .ndo_set_mac_address    = smsc7500_eth_mac_addr,
        .ndo_validate_addr      = eth_validate_addr,
        .ndo_do_ioctl           = smsc7500_do_ioctl,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0))
        .ndo_set_multicast_list = smsc7500_set_multicast,
#else
	.ndo_set_rx_mode	= smsc7500_set_multicast,
#endif
        .ndo_get_stats          = smscusbnet_get_stats,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0))
#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
	.ndo_vlan_rx_register   = smsc7500_vlan_rx_register,
#endif
#endif
};
#endif //linux 2.6.29

/**************************************************************************************
Routine Description:
    Driver binding routine.
Arguments:
	netdev 		- Point to usb device structure
	intf		- usb interface
Return Value:
	0		- Success
	<0		- Fail
***********************************************************************************/
static int smsc7500_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int retVal=0, i;
	PADAPTER_DATA adapterData=NULL;
	u32 dwBuf;
	char version[15];

	SMSC_DEBUG(DBG_INIT,"---------->in smsc7500_bind\n");

    //Init system control and status regsiter map
	SET_REG_ARRAY(ID_REV);
	SET_REG_ARRAY(FPGA_REV);
	SET_REG_ARRAY(BOND_CTL);
	SET_REG_ARRAY(INT_STS);
	SET_REG_ARRAY(HW_CFG);
	SET_REG_ARRAY(PMT_CTL);
	SET_REG_ARRAY(LED_GPIO_CFG);
	SET_REG_ARRAY(GPIO_CFG);
	SET_REG_ARRAY(GPIO_WAKE);
	SET_REG_ARRAY(DP_SEL);
	SET_REG_ARRAY(DP_CMD);
	SET_REG_ARRAY(DP_ADDR);
	SET_REG_ARRAY(DP_DATA);
	SET_REG_ARRAY(BURST_CAP);
	SET_REG_ARRAY(INT_EP_CTL);
	SET_REG_ARRAY(BULK_IN_DLY);
	SET_REG_ARRAY(E2P_CMD);
	SET_REG_ARRAY(E2P_DATA);
	SET_REG_ARRAY(DBG_STRAP);
	SET_REG_ARRAY(HS_ATTR);
	SET_REG_ARRAY(FS_ATTR);
	SET_REG_ARRAY(STRING_ATTR0);
	SET_REG_ARRAY(STRING_ATTR1);
	SET_REG_ARRAY(FLAG_ATTR);
	SET_REG_ARRAY(REF_CTL);
	SET_REG_ARRAY(VLAN_TYPE);
	SET_REG_ARRAY(FCT_RX_CTL);
	SET_REG_ARRAY(FCT_TX_CTL);
	SET_REG_ARRAY(FCT_RX_FIFO_END);
	SET_REG_ARRAY(FCT_TX_FIFO_END);
	SET_REG_ARRAY(FCT_FLOW);
	SET_REG_ARRAY(MAC_CR);
	SET_REG_ARRAY(MAC_RX);
	SET_REG_ARRAY(MAC_TX);
	SET_REG_ARRAY(FLOW);
	SET_REG_ARRAY(RAND_SEED);
	SET_REG_ARRAY(ERR_STS);
	SET_REG_ARRAY(RX_ADDRH);
	SET_REG_ARRAY(RX_ADDRL);
	SET_REG_ARRAY(MII_ACCESS);
	SET_REG_ARRAY(MII_DATA);
	SET_REG_ARRAY(WUCSR);
	SET_REG_ARRAY(WUF_CFGX);
	SET_REG_ARRAY(WUF_MASKX);
	SET_REG_ARRAY(ADDR_FILTX);
	SET_REG_ARRAY(WUCSR2);
	SET_REG_ARRAY(IPV4_ADDRX);
	SET_REG_ARRAY(IPV6_ADDRX);

    //Init PHY map
    SET_PHY_ARRAY(PHY_BCR);
    SET_PHY_ARRAY(PHY_BSR);
    SET_PHY_ARRAY(PHY_ID1);
    SET_PHY_ARRAY(PHY_ID2);
    SET_PHY_ARRAY(PHY_AN_ADV);
    SET_PHY_ARRAY(PHY_AN_LPA);
    SET_PHY_ARRAY(PHY_AN_EXP);
    SET_PHY_ARRAY(PHY_AN_NP_TX);
    SET_PHY_ARRAY(PHY_AN_NP_RX);
    SET_PHY_ARRAY(PHY_MSCR);
    SET_PHY_ARRAY(PHY_MSSR);
    SET_PHY_ARRAY(PHY_EXT_ST);
    SET_PHY_ARRAY(PHY_LINK_CONTROL);
    SET_PHY_ARRAY(PHY_MODE_CSR);
    SET_PHY_ARRAY(PHY_SPECIAL_MODE);
    SET_PHY_ARRAY(PHY_EXT_CSR);
    SET_PHY_ARRAY(PHY_AR_ADDR);
    SET_PHY_ARRAY(PHY_AR_DATA);
    SET_PHY_ARRAY(PHY_CSI);
    SET_PHY_ARRAY(PHY_CHANNEL_QUALITY);
    SET_PHY_ARRAY(PHY_INT_SRC);
    SET_PHY_ARRAY(PHY_INT_MASK);
    SET_PHY_ARRAY(PHY_SCSR);

	sprintf(version,"%lX.%02lX.%02lX",
		(DRIVER_VERSION>>16),(DRIVER_VERSION>>8)&0xFF,(DRIVER_VERSION&0xFFUL));
	SMSC_DEBUG(DBG_INIT,"Driver smsc7500.ko verison %s, built on %s, %s",version, __TIME__, __DATE__);

	retVal=smscusbnet_get_endpoints(dev,intf);
	if (retVal<0)
		goto out1;

	dev->data[0]=(unsigned long) kmalloc(sizeof(ADAPTER_DATA),GFP_KERNEL);

	if((PADAPTER_DATA)dev->data[0]==NULL) {
		SMSC_WARNING("Unable to allocate ADAPTER_DATA");
		retVal=-ENOMEM;
		goto out1;
	}
	memset((PADAPTER_DATA)dev->data[0],0,sizeof(ADAPTER_DATA));
	adapterData=(PADAPTER_DATA)(dev->data[0]);

	sema_init(&adapterData->phy_mutex, 1);
	sema_init(&adapterData->internal_ram_mutex, 1);
	sema_init(&adapterData->RxFilterLock, 1);

	if ((retVal = smsc7500_read_reg(dev,HW_CFG,&dwBuf)< 0)) {
		SMSC_WARNING("Failed to read HW_CFG: %d", retVal);
		return retVal;
	}
	if(dwBuf & HW_CFG_SMDET_STS){
		SMSC_DEBUG(DBG_INIT,"Come back from net detach");
	}

	adapterData->linkRequest = link_mode;
	adapterData->macAddrHi16 = mac_addr_hi16;
	adapterData->MmacAddrLo32 = mac_addr_lo32;

	adapterData->rxFifoSize = MAX_RX_FIFO_SIZE;
	adapterData->txFifoSize = MAX_TX_FIFO_SIZE;

	adapterData->eepromSize = MAX_EEPROM_SIZE + 128; //Set a initial value
	adapterData->eepromSize = smsc7500_eeprom_size(dev);
	//SMSC_TRACE(DBG_INIT,"EEPROM size: %d bytes", adapterData->eepromSize);

	//Init all registers
	retVal = smsc7500_reset(dev);
    if(retVal < 0)goto out1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
	dev->net->do_ioctl = smsc7500_do_ioctl;
	dev->net->set_multicast_list = smsc7500_set_multicast;
	dev->net->set_mac_address = smsc7500_eth_mac_addr;
#else
	dev->net->netdev_ops = &smsc95xx_netdev_ops;
#endif //2.6.29
	dev->net->ethtool_ops = &smsc7500_ethtool_ops;
	dev->net->flags |= IFF_MULTICAST;

	dev->linkDownSuspend = linkdownsuspend;
	dev->dynamicSuspend = dynamicsuspend;

	if(dev->udev->config->desc.bmAttributes & USB_CONFIG_ATT_WAKEUP) {
		SMSC_DEBUG(DBG_INIT,"The device is configed to support remote wakes\n");
	} else {
		SMSC_WARNING("The device is not configed to support remote wakes , Over write to disable linkdownsuspend and dynamicsuspend \n");
		if(dev->dynamicSuspend || dev->linkDownSuspend){
		dev->dynamicSuspend = dev->linkDownSuspend = 0;
		}
	} 
	
#ifndef CONFIG_PM
	if(dev->dynamicSuspend || dev->linkDownSuspend){
		SMSC_WARNING("Power management has to be enabled in the kernel configuration to support dynamicsuspend and linkdownsuspend");
		dev->dynamicSuspend = dev->linkDownSuspend = 0;
	}
#endif //CONFIG_PM
#ifndef CONFIG_USB_SUSPEND
	if(dev->dynamicSuspend || dev->linkDownSuspend){
		SMSC_WARNING("Usb suspend has to be enabled in the kernel configuration to support dynamicsuspend and linkdownsuspend");
		dev->dynamicSuspend = dev->linkDownSuspend = 0;
	}
#endif //CONFIG_USB_SUSPEND

	dev->netDetach = netdetach;
	//If net detach is enabled, link down suspend should be disabled
	if(dev->netDetach)dev->linkDownSuspend = 0;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_PM
	if(dev->dynamicSuspend || dev->linkDownSuspend){
#if ((LINUX_VERSION_CODE > KERNEL_VERSION(2,6,21)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))) 
		if(dev->udev->autosuspend_disabled){
#endif
			SMSC_WARNING("Autosuspend should be enabled by shell cmd \"echo auto > /sys/bus/usb/devices/X-XX/power/level\"");
#if ((LINUX_VERSION_CODE > KERNEL_VERSION(2,6,21)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)))
		}
#endif
	}
#endif //CONFIG_PM
#endif

	adapterData->UseTxCsum = tx_Csum;
	adapterData->UseRxCsum = rx_Csum;
	if (tx_Csum)
		SMSC_DEBUG(DBG_INIT,"Tx HW Checksum");
	if (rx_Csum)
		SMSC_DEBUG(DBG_INIT,"Rx HW Checksum");

	dev->net->features = 0;

	if(tso){
		dev->net->features |= NETIF_F_TSO | NETIF_F_TSO6;
	}
	if(adapterData->UseTxCsum){
		dev->net->features |= NETIF_F_HW_CSUM;
	}
	if(scatter_gather){
		dev->net->features |= NETIF_F_SG | NETIF_F_FRAGLIST;
	}
#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
	adapterData->vlanAcceleration = vlan_accel;
#endif
	if(adapterData->vlanAcceleration){
#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
		dev->net->vlan_rx_register = smsc7500_vlan_rx_register;
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
		dev->net->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#else
		dev->net->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
#endif
#endif
	}

	adapterData->dwTxQueueDisableMask=0;
	spin_lock_init(&(adapterData->TxQueueLock));
	adapterData->TxInitialized=TRUE;

	adapterData->WolWakeupOpts= WAKE_PHY;
	adapterData->LinkActLedCfg = LinkActLedCfg;

	for(i=0; i<NO_OF_STAS_CNT; i++){
		adapterData->frameCnt[i].len = LEN_20;
    }
	adapterData->frameCnt[txBadBytes].len = LEN_32;
	adapterData->frameCnt[txUnicastByteCount].len = LEN_32;
	adapterData->frameCnt[txBroadcastByteCount].len = LEN_32;
	adapterData->frameCnt[txMulticastByteCount].len = LEN_32;

	adapterData->LanInitialized=TRUE;

	SMSC_DEBUG(DBG_INIT,"<--------out of bind, return 0\n");
	return 0;

	if (adapterData != NULL){
		kfree(adapterData);
		adapterData=NULL;
	}
out1:
	SMSC_DEBUG(DBG_INIT,"<--------bind out1, return %d\n",retVal);
	return retVal;
}

/**************************************************************************************
Routine Description:
    Driver unbinding routine.
Arguments:
	netdev 		- Point to usb device structure
	intf		- usb interface
Return Value:
	0		- Success
	<0		- Fail
***********************************************************************************/
static void smsc7500_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	SMSC_DEBUG(DBG_CLOSE,"------->in smsc7500_unbind\n");

	if (adapterData != NULL){
		SMSC_DEBUG(DBG_CLOSE,"free adapterData\n");
		kfree(adapterData);
		adapterData=NULL;
	}

	SMSC_DEBUG(DBG_CLOSE,"<-------out of smsc7500_unbind\n");
}

/**************************************************************************************
Routine Description:
    This routine parsers Rx packets based on device control word.
Arguments:
	dev 		- Point to usb device structure
	skb			- skb buffer to be processed
Return Value:
	RX_FIXUP_VALID_SKB		- Valid skb
	RX_FIXUP_INVALID_SKB	- invalid skb
	RX_FIXUP_ERROR			- Error
***********************************************************************************/
static int smsc7500_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u32 size;
	u32  cmdA, cmdB, AlignCount=0;
	struct sk_buff *clone;
	int retVal = RX_FIXUP_VALID_SKB;
	u16 *vlan_tag;

	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_DEBUG(DBG_RX,"------->in smsc7500_rx_fixup\n");

	memcpy(&cmdA, (void*)skb->data, sizeof(cmdA));
	le32_to_cpus(&cmdA);
	skb_pull(skb, 4);
	memcpy(&cmdB, (void*)skb->data, sizeof(cmdB));
	le32_to_cpus(&cmdB);
	skb_pull(skb, 4);

	while (skb->len > 0) {
		/* get the packet length */
		size = cmdA & RX_CMD_A_LEN;
		size -= RXW_PADDING; //Remove padding bytes

		skb_pull(skb, RXW_PADDING);

		AlignCount = (STATUS_WORD_LEN - ((size + RXW_PADDING) % STATUS_WORD_LEN)) % STATUS_WORD_LEN;

		if(cmdA & RX_CMD_A_RED){

			if(size == skb->len){//last packet
				return RX_FIXUP_INVALID_SKB;
			}else{

				skb_pull(skb, size+AlignCount);

				if (skb->len == 0) {
					retVal = RX_FIXUP_INVALID_SKB;

					return retVal;
				}
				goto NEXT_PACKET;
			}

		}

		if (size == skb->len){

			if (adapterData->UseRxCsum && !(cmdA & RX_CMD_A_LCSM)) {
				skb->csum = (u16)(cmdB >> RX_CMD_B_CSUM_SHIFT);
				skb->csum = ntohs(skb->csum); //It is big-endian.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
				skb->ip_summed = CHECKSUM_HW;
#else
				skb->ip_summed = CHECKSUM_COMPLETE;
#endif
			}else{
				skb->ip_summed = CHECKSUM_NONE;
			}
			skb_trim(skb,size-4);
			//Get vlan tag

			vlan_tag = (u16*)&skb->cb[0];
			if((cmdA & RX_CMD_A_FVTG) && adapterData->vlanAcceleration){
				*vlan_tag = (u16)(cmdB & RX_CMD_B_VTAG); //Passes valan tag to routine smscusbnet_skb_return()
			}else{
				*vlan_tag = VLAN_DUMMY; //Reserved value
			}
//Kernel calculate received size based on skb->truesize, which holds total buffer size.
//If we allocate a big skb buffer, but only part of buffer hold valid data like turbo mode did,
//Kernel accumulate received data with skb->truesize, so total received data might be over limit. But
//actual data size isn't, then kernel may drop the subsequent packets.
//We are not supposed to change truesize, but this is the easy way to cheat kernel without memory copy
			skb->truesize = skb->len + sizeof(struct sk_buff);

			return retVal;
		}

        clone = skb_clone(skb, GFP_ATOMIC);

        if (clone) {

            clone->len = size;
            clone->data = skb->data;
			skb_set_tail_pointer(clone, size);
			if (adapterData->UseRxCsum && !(cmdA & RX_CMD_A_LCSM)){
				clone->csum = (u16)(cmdB >> RX_CMD_B_CSUM_SHIFT);
				clone->csum = ntohs(clone->csum); //It is big-endian.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
				clone->ip_summed = CHECKSUM_HW;
#else
				clone->ip_summed = CHECKSUM_COMPLETE;
#endif
			}else{
				clone->ip_summed = CHECKSUM_NONE;
			}

			skb_trim(clone,size-4);

			//Get vlan tag
			vlan_tag = (u16*)&clone->cb[0];
			if((cmdA & RX_CMD_A_FVTG) && adapterData->vlanAcceleration){
				*vlan_tag = (u16)(cmdB & RX_CMD_B_VTAG); //Passes valan tag to routine smscusbnet_skb_return()
			}else{
				*vlan_tag = 0xFFFF; //Reserved value
			}

//We are not supposed to change truesize, but this is the easy way to cheat kernel without memory copy
			clone->truesize = clone->len + sizeof(struct sk_buff);

			smscusbnet_skb_return(dev, clone);
		} else {
			SMSC_DEBUG(DBG_RX,"Cannot clone\n");
			return RX_FIXUP_ERROR;
		}

		skb_pull(skb, size+AlignCount);

		if (skb->len == 0) {
			SMSC_DEBUG(DBG_RX,"skb->len==0 left\n");
			break;
		}
NEXT_PACKET:

		memcpy(&cmdA, (void*)skb->data, sizeof(cmdA));
		le32_to_cpus(&cmdA);
		skb_pull(skb, sizeof(cmdA));
		memcpy(&cmdB, (void*)skb->data, sizeof(cmdB));
		le32_to_cpus(&cmdB);
		skb_pull(skb, sizeof(cmdB));
	}

	if (skb->len < 0) {
		SMSC_WARNING("invalid rx length<0 %d", skb->len);
		return RX_FIXUP_ERROR;
	}

	SMSC_DEBUG(DBG_RX,"<-------out of smsc7500_rx_fixup\n");
	return retVal;
}

/**************************************************************************************
Routine Description:
    This routine processes Tx packets and adds device control word.
Arguments:
	dev 		- Point to usb device structure
	skb			- skb buffer to be processed
	flag		- memory allocation flag
Return Value:
	!NULL		- skb pointer
	NULL		- Fail
***********************************************************************************/
static struct sk_buff *smsc7500_tx_fixup(struct usbnet *dev, struct sk_buff *skb, int flags)
{

	int headroom = skb_headroom(skb);
	int tailroom = skb_tailroom(skb);
	int skbSize;
	int skbExpend = FALSE;
	struct sk_buff *skbOut = skb;
	struct sk_buff *skb_tmp;
	u32 TxCommandA = 0, TxCommandB = 0;
	u32 mss = 0;
	u32 AlignmentSize;

	SMSC_DEBUG(DBG_TX, "------->in smsc7500_tx_fixup\n");

	AlignmentSize = skb->len % STATUS_WORD_LEN;
	if(AlignmentSize) AlignmentSize = STATUS_WORD_LEN - AlignmentSize;

	if(skb_is_nonlinear(skb)){
		//Merge into a single skb buffer
		SMSC_DEBUG(DBG_TX, "Multifrags cnt = %d\n", skb_shinfo(skb)->nr_frags + 1);

		skbSize = skb->len + TX_TOTAL_CONTROL_WORD_LEN+AlignmentSize;
		skb_tmp = dev_alloc_skb(skbSize);
		if (!skb_tmp){
			SMSC_WARNING("cannot allocat skb buffer\n");
			return NULL;
		}
		skb_put(skb_tmp, skbSize - TX_TOTAL_CONTROL_WORD_LEN-AlignmentSize);
		if(skb_copy_bits(skb, 0, skb_tmp->data, skb->len) != 0){
			SMSC_WARNING("Failed to copy skb buffer\n");
			return NULL;
		}
		memmove(skb_shinfo(skb_tmp), skb_shinfo(skb), sizeof(struct skb_shared_info));
		skb_shinfo(skb_tmp)->nr_frags = 0;
		skbOut = skb_tmp;
		skbOut->ip_summed = skb->ip_summed;
		dev_kfree_skb_any(skb);
	}

	if ((headroom < (2*STATUS_WORD_LEN)) || (tailroom < AlignmentSize)){

		if (!skb_cloned(skbOut)){
			 if((headroom + tailroom) >= (TX_TOTAL_CONTROL_WORD_LEN+ AlignmentSize)){
				skbOut->data = memmove(skbOut->head + TX_TOTAL_CONTROL_WORD_LEN, skbOut->data, skbOut->len);
				skbSize = skbOut->len;
				skb_trim(skbOut, 0);
				skb_put(skbOut, skbSize);
			 }else{
				 skbExpend = TRUE;
			 }
		}else{
			skbExpend = TRUE;
		}
		if(skbExpend){

			skb_tmp = skb_copy_expand(skbOut, TX_TOTAL_CONTROL_WORD_LEN, AlignmentSize, flags);
			if (!skb_tmp){
				return NULL;
			}
			skb_tmp->ip_summed = skbOut->ip_summed; //ip_summed was cleared
			dev_kfree_skb_any(skbOut);
			skbOut = skb_tmp;
		}
	}
	skb_push(skbOut, STATUS_WORD_LEN);
	if (skb_is_gso(skbOut)) {
		mss = skb_shinfo(skbOut)->gso_size;
		if(mss < TX_MSS_MIN)mss = TX_MSS_MIN;
		TxCommandB = (mss << TX_CMD_B_MSS_SHIFT) & TX_CMD_B_MSS;
	}

	if(vlan_tx_tag_present(skbOut)){
		TxCommandB |= vlan_tx_tag_get(skbOut) & TX_CMD_B_VTAG;
	}

	cpu_to_le32s((u32*)&TxCommandB);
	memcpy(skbOut->data, &TxCommandB, STATUS_WORD_LEN);

	skb_push(skbOut, STATUS_WORD_LEN);

	TxCommandA = 0;
	if (skb_is_gso(skbOut)) {//TSO packet
		SMSC_DEBUG(DBG_TX,"Gso packet size = %d, skb->len = %d\n", mss, skbOut->len);
		TxCommandA |= TX_CMD_A_LSO;
	}
	TxCommandA |= ((skbOut->len - 2*STATUS_WORD_LEN) & TX_CMD_A_LEN);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
	if(skbOut->ip_summed == CHECKSUM_HW)
#else
	if(skbOut->ip_summed == CHECKSUM_PARTIAL)
#endif
	{
		TxCommandA |= TX_CMD_A_IPE | TX_CMD_A_TPE;
	}

	if(vlan_tx_tag_present(skbOut)){//Insert vlan tag
		TxCommandA |= TX_CMD_A_IVTG;
	}
	TxCommandA |= TX_CMD_A_FCS;

	cpu_to_le32s((u32*)&TxCommandA);
	memcpy(skbOut->data, &TxCommandA, STATUS_WORD_LEN);

	skb_put(skbOut, AlignmentSize);
	SMSC_DEBUG(DBG_TX, "<-------out of smsc7500_tx_fixup\n");

	return skbOut;
}

static int set_phy_bits(struct usbnet *dev, u32 phyAddr, u32 data)
{
	u32 dwData = 0;
	int retVal = 0;

	if ((retVal = smsc7500_read_phy(dev, phyAddr, &dwData)< 0)) {
		return retVal;
	}
	dwData |= data;
	if ((retVal = smsc7500_write_phy(dev, phyAddr, dwData)< 0)){
		return retVal;
	}
	return 0;
}

static int reset_phy_bits(struct usbnet *dev, u32 phyAddr, u32 data)
{
	u32 dwData = 0;
	int retVal = 0;

	if ((retVal = smsc7500_read_phy(dev, phyAddr, &dwData)< 0)) {
		return retVal;
	}
	dwData &= ~data;
	if ((retVal = smsc7500_write_phy(dev, phyAddr, dwData)< 0)){
		return retVal;
	}
	return 0;
}

static int set_reg_bits(struct usbnet *dev, u32 regAddr, u32 data)
{
	u32 dwData = 0;
	int retVal = 0;

	if ((retVal = smsc7500_read_reg(dev, regAddr, &dwData)< 0)) {
		return retVal;
	}
	dwData |= data;
	if ((retVal = smsc7500_write_reg(dev, regAddr, dwData)< 0)){
		return retVal;
	}
	return 0;
}

static int reset_reg_bits(struct usbnet *dev, u32 regAddr, u32 data)
{
	u32 dwData = 0;
	int retVal = 0;

	if ((retVal = smsc7500_read_reg(dev, regAddr, &dwData)< 0)) {
		return retVal;
	}
	dwData &= ~data;
	if ((retVal = smsc7500_write_reg(dev, regAddr, dwData)< 0)){
		return retVal;
	}
	return 0;
}

/**************************************************************************************
Routine Description:
    This routine resets lan7500 and sets all registers.
Arguments:
	dev 		- Point to usb device structure
Return Value:
	0			- Success
***********************************************************************************/
static int smsc7500_reset(struct usbnet *dev)
{
	int retVal=0,Timeout,i;
	u32 dwData, dwAddrH, dwAddrL, dwBurstCap;
	PADAPTER_DATA adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_DEBUG(DBG_INIT,"---------->smsc7500_reset\n");

	/*********Reset device *******************************/
	if((retVal = set_reg_bits(dev, HW_CFG, HW_CFG_LRST)) < 0){
		SMSC_WARNING("Failed to access HW_CFG: %d", retVal);
		return retVal;
	}

	Timeout = 0;
	do {
		if ((retVal = smsc7500_read_reg(dev,HW_CFG,&dwData)< 0)) {
			SMSC_WARNING("Failed to read HW_CFG: %d", retVal);
			return retVal;
		}
		msleep(100);         /* wait for 100 us before trying again */
	} while ( (dwData & HW_CFG_LRST) && (Timeout++ < 20));

	if(Timeout >= 20)
	{
		SMSC_WARNING("Timeout waiting for completion of Lite Reset\n");
		return SMSC7500_FAIL;
	}

	/****************get chip ID *******************************/
	if((retVal = smsc7500_read_reg(dev, ID_REV, &dev->chipID)) < 0){
		SMSC_WARNING("Failed to read GPIO_CFG: %d", retVal);
		return retVal;
	  }
	dev->chipID = dev->chipID >> 16;

	/****************Reset PHY *******************************/
	if((retVal = set_reg_bits(dev, PMT_CTL, PMT_CTL_PHY_RST)) < 0){
		SMSC_WARNING("Failed to access PMT_CTL: %d", retVal);
		return retVal;
	}
	Timeout = 0;
	do {
		if ((retVal = smsc7500_read_reg(dev,PMT_CTL,&dwData)< 0)) {
			SMSC_WARNING("Failed to read PMT_CTL: %d", retVal);
			return retVal;
		}
		msleep(100);
		Timeout++;
	} while ( (dwData & PMT_CTL_PHY_RST) && (Timeout < 100));

	if(Timeout >= 100)
	{
		SMSC_WARNING("Timeout waiting for PHY Reset\n");
		return SMSC7500_FAIL;
	}

/*********************Init MAC address ******************************************************/
	dwAddrH = 0x0000FFFFUL;
	dwAddrL = 0xFFFFFFFF;

	if(adapterData->macAddrHi16 != DUMMY_VALUE || adapterData->MmacAddrLo32 != DUMMY_VALUE){
		dwAddrH = adapterData->macAddrHi16 & 0xFFFF;
		dwAddrL = adapterData->MmacAddrLo32;
	}else{
		if(adapterData->eepromSize && smsc7500_read_eeprom(dev, EEPROM_MAC_OFFSET, 6,  dev->net->dev_addr) == 0){
			dwAddrL = dev->net->dev_addr[0] | dev->net->dev_addr[1] << 8 | dev->net->dev_addr[2] << 16 | dev->net->dev_addr[3] << 24;
			dwAddrH = dev->net->dev_addr[4] | dev->net->dev_addr[5] << 8;
		}else{//LAN7500's descriptor RAM may provide Mac address
			MAC_ADDR_IN_RAM macRam;
			if(read_data_port(dev, DP_SEL_DESCRIPTOR, 0, sizeof(MAC_ADDR_IN_RAM)/4, (u32*)&macRam) == SMSC7500_SUCCESS){
				cpu_to_le32s(&macRam.signature);
				cpu_to_le32s(&macRam.MacAddrL);
				cpu_to_le32s(&macRam.MacAddrH);
				cpu_to_le16s(&macRam.crc);
				cpu_to_le16s(&macRam.crcComplement);
				if(macRam.signature == 0x736D7363){//Signature "smsc"
					u16 crc = calculate_crc16((char*)&macRam, 12, FALSE);
					if((crc == macRam.crc) && (crc == (u16)~macRam.crcComplement)){
						dwAddrL = macRam.MacAddrL;
						dwAddrH = macRam.MacAddrH;
					}
				}
			}
		}
	}

	if((dwAddrH==0x0000FFFFUL)&&(dwAddrL==0xFFFFFFFF)){
		if ((retVal = smsc7500_read_reg(dev,RX_ADDRL, &dwAddrL)< 0)) {
			SMSC_WARNING("Failed to read RX_ADDRL: %d", retVal);
			return retVal;
		}
		if ((retVal = smsc7500_read_reg(dev,RX_ADDRH, &dwAddrH)< 0)) {
			SMSC_WARNING("Failed to read RX_ADDRH: %d", retVal);
			return retVal;
		}
	}

	if(((dwAddrH & 0xFFFF) == 0x0000FFFFUL) && (dwAddrL == 0xFFFFFFFF))
	{
		dwAddrH=0x00000070UL;
		dwAddrL=0x110F8000UL;

		SMSC_DEBUG(DBG_INIT,"Mac Address is set by default to 0x%04X%08X\n",
			dwAddrH,dwAddrL);
	}
	adapterData->macAddrHi16 = dwAddrH;
	adapterData->MmacAddrLo32 = dwAddrL;

	if ((retVal = smsc7500_write_reg(dev,RX_ADDRL, dwAddrL)< 0)) {
		SMSC_WARNING("Failed to write RX_ADDRL: %d", retVal);
		return retVal;
	}
	if ((retVal = smsc7500_write_reg(dev,RX_ADDRH, dwAddrH)< 0)) {
		SMSC_WARNING("Failed to write RX_ADDRH: %d", retVal);
		return retVal;
	}

	dwData = ADDR_FILTX_FB_VALID | adapterData->macAddrHi16;
	/*First slot is reserved for own address*/
	if ((retVal = smsc7500_write_reg(dev, ADDR_FILTX, dwData)< 0)) {
		SMSC_WARNING("Failed to write RX_ADDRH: %d", retVal);
		return retVal;
	}
	if ((retVal = smsc7500_write_reg(dev, ADDR_FILTX+4, adapterData->MmacAddrLo32)< 0)) {
		SMSC_WARNING("Failed to write RX_ADDRH: %d", retVal);
		return retVal;
	}

	dev->net->dev_addr[0]=LOBYTE(LOWORD(dwAddrL));
	dev->net->dev_addr[1]=HIBYTE(LOWORD(dwAddrL));
	dev->net->dev_addr[2]=LOBYTE(HIWORD(dwAddrL));
	dev->net->dev_addr[3]=HIBYTE(HIWORD(dwAddrL));
	dev->net->dev_addr[4]=LOBYTE(LOWORD(dwAddrH));
	dev->net->dev_addr[5]=HIBYTE(LOWORD(dwAddrH));

	SMSC_DEBUG(DBG_INIT,"dev->net->dev_addr %02x:%02x:%02x:%02x:%02x:%02x\n",
			dev->net->dev_addr [0], dev->net->dev_addr [1],
			dev->net->dev_addr [2], dev->net->dev_addr [3],
			dev->net->dev_addr [4], dev->net->dev_addr [5]);

/**********************************************************************************************/
	if (!(smscusbnet_IsOperationalMode(dev))) {
		/****************Reset PHY *******************************/
		if((retVal = set_reg_bits(dev, HW_CFG, HW_CFG_BIR)) < 0){
			SMSC_WARNING("Failed to access HW_CFG: %d", retVal);
			return retVal;
		}
		SMSC_DEBUG(DBG_INIT,"Read Value from HW_CFG : 0x%08x\n",dwData);
	}

/*********************Init Burst delay ******************************************************/
	if (TurboMode) {
		if(dev->udev->speed == USB_SPEED_HIGH){
			dev->rx_urb_size = DEFAULT_HS_BURST_CAP_SIZE;
			dwBurstCap = DEFAULT_HS_BURST_CAP_SIZE / HS_USB_PKT_SIZE;
		}else{
			dev->rx_urb_size = DEFAULT_FS_BURST_CAP_SIZE;
			dwBurstCap = DEFAULT_FS_BURST_CAP_SIZE / FS_USB_PKT_SIZE;
		}
	}else{
		dwBurstCap = 0;
		dev->rx_urb_size = MAX_SINGLE_PACKET_SIZE;
	}
	SMSC_DEBUG(DBG_INIT,"rx_urb_size= %d\n", (int)dev->rx_urb_size);

	if ((retVal = smsc7500_write_reg(dev, BURST_CAP, dwBurstCap))<0)
	{
		SMSC_WARNING("Failed to write BURST_CAP");
		return retVal;
	}

	if ((retVal = smsc7500_write_reg(dev, BULK_IN_DLY, bulkin_delay))<0)
	{
		SMSC_WARNING("Failed to write BULK_IN_DLY");
		return retVal;
	}

	if (TurboMode) {
		if((retVal = set_reg_bits(dev, HW_CFG, HW_CFG_MEF | HW_CFG_BCE)) < 0){
			SMSC_WARNING("Failed to access HW_CFG: %d", retVal);
			return retVal;
		}
		//smsc7500_write_reg(dev, BOND_CTL, 0x00446373); //Enalbe MEF
	}

/**********************************************************************************************/
	/*Set FIFO size*/
	if ((retVal = smsc7500_write_reg(dev,FCT_RX_FIFO_END, (adapterData->rxFifoSize - 512UL)/512UL)< 0)) {
		SMSC_WARNING("Failed to read FCT_RX_FIFO_END: %d", retVal);
		return retVal;
	}
	if ((retVal = smsc7500_write_reg(dev,FCT_TX_FIFO_END, (adapterData->txFifoSize - 512UL)/512UL)< 0)) {
		SMSC_WARNING("Failed to read FCT_TX_FIFO_END: %d", retVal);
		return retVal;
	}

	if ((retVal = smsc7500_write_reg(dev, INT_STS, 0xFFFFFFFFUL))<0){
		SMSC_WARNING("Failed to write INT_STS register, retVal = %d \n",retVal);
		return retVal;
	}

#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
	if((retVal = set_reg_bits(dev, MAC_RX, MAC_RX_FSE)) < 0){//Enable VALN frame enforcement
		SMSC_WARNING("Failed to access MAC_RX: %d", retVal);
		return retVal;
	}
#endif

	if ((retVal = smsc7500_read_reg(dev,ID_REV,&dwData)< 0)) {
		SMSC_WARNING("Failed to read ID_REV: %d", retVal);
		return retVal;
	}
	adapterData->dwIdRev = dwData;

	if ((retVal = smsc7500_read_reg(dev,FPGA_REV,&dwData)< 0)) {
		SMSC_WARNING("Failed to read FPGA_REV: %d", retVal);
		return retVal;
	}
	adapterData->dwFpgaRev = dwData;

	if (smscusbnet_IsOperationalMode(dev)) {
		if((retVal = set_reg_bits(dev, INT_EP_CTL, INT_EP_CTL_RX_FIFO_EN)) < 0){
			SMSC_WARNING("Failed to access INT_EP_CTL: %d", retVal);
			return retVal;
		}
	}

	dwData = REF_CTL_AB | REF_CTL_DPF; //Enable receiving broadcast
	if(adapterData->vlanAcceleration){
		dwData |= REF_CTL_VS;		//Strip vlan tag from Rx packets
	}
	if (adapterData->UseRxCsum) {
		dwData |= REF_CTL_TCPUDP_CKM | REF_CTL_IP_CKM; //Enable Rx checksum offload
	}

	if((retVal = set_reg_bits(dev, REF_CTL, dwData)) < 0){
		SMSC_WARNING("Failed to access REF_CTL: %d", retVal);
		return retVal;
	}

	if(adapterData->eepromSize == 0){//No eeprom
		if(adapterData->LinkActLedCfg == DUMMY_VALUE){
			adapterData->LinkActLedCfg = FALSE;
		}
	}

	if(adapterData->LinkActLedCfg != DUMMY_VALUE){
	   if ((retVal = smsc7500_read_reg(dev, LED_GPIO_CFG,&dwData)< 0)) {
			SMSC_WARNING("Failed to read LED_GPIO_CFG: %d", retVal);
			return retVal;
		}
		dwData &= ~(LED_GPIO_CFG_LED2_FUN_SEL | LED_GPIO_CFG_LED10_FUN_SEL);
		dwData |= LED_GPIO_CFG_LEDGPIO_EN;

		if(adapterData->LinkActLedCfg){//enables separate Link and Activity LEDs
			dwData |= LED_GPIO_CFG_LED2_FUN_SEL;
		}else{
			dwData |= LED_GPIO_CFG_LED10_FUN_SEL;
		}

		// Set LED GPIO Config
		if((retVal = smsc7500_write_reg(dev, LED_GPIO_CFG, dwData)) < 0){
			SMSC_WARNING("Failed to write LED_GPIO_CFG: %d", retVal);
			return retVal;
		}
   }

	//Statistics counters are rollover ones
	if(smsc7500_get_stats(dev, adapterData->preFrameCnt) > 0){//Save initial value
		for(i=0; i<NO_OF_STAS_CNT; i++){
			le32_to_cpus(&adapterData->preFrameCnt[i]);
		}
	}

	smsc7500_rx_setmulticastlist(dev);

	/* phy workaround for gig link */
	Phy_gig_workaround(dev);

	if (!Phy_Initialize(dev))
		return SMSC7500_FAIL;

	phy_set_link(dev, adapterData->linkRequest);

	/******MAC_CR has to be set after PHY init***********
	******MAC will auto detect PHY speed*****************/
	if((retVal = set_reg_bits(dev, MAC_CR, MAC_CR_ADD | MAC_CR_ASD)) < 0){
		SMSC_WARNING("Failed to access MAC_CR: %d", retVal);
		return retVal;
	}

	smsc7500_start_rx_path(dev);
	smsc7500_start_tx_path(dev);

#ifdef USE_DEBUG
	LanDumpRegs(dev);
#endif //USE_DEBUG
	SMSC_DEBUG(DBG_INIT,"<--------out of smsc7500_reset, return 0\n");

	return 0;
}

/**************************************************************************************
Routine Description:
    This routine checks link status.
Arguments:
	dev 		- Point to usb device structure
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_link_reset(struct usbnet *dev)
{
    int retVal = 0;
	SMSC_DEBUG(DBG_LINK,"---->in smsc7500_link_reset\n");
	retVal = phy_check_link(dev);

	if(dev->StopLinkPolling){
		clear_bit (EVENT_DEV_RECOVERY, &dev->flags);
	}

    if (test_bit (EVENT_DEV_RECOVERY, &dev->flags)) {
        retVal = smsc7500_device_recovery(dev);
        clear_bit (EVENT_DEV_RECOVERY, &dev->flags);
    }

	SMSC_DEBUG(DBG_LINK,"<----out of smsc7500_link_reset\n");
	return retVal;
}

/**************************************************************************************
Routine Description:
    This routine Starts the Tx path at the MAC and  and Tx FIFO
    bulk out requests
Arguments:
    dev 			- pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_start_tx_path(struct usbnet * dev )
{
	int retVal = SMSC7500_FAIL;

	// Enable Tx at MAC
	CHECK_RETURN_STATUS(set_reg_bits(dev, MAC_TX, MAC_TX_TXEN));
	CHECK_RETURN_STATUS(set_reg_bits(dev, FCT_TX_CTL, FCT_TX_CTL_EN));
	retVal = SMSC7500_SUCCESS;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine Stops the Tx Path at Mac and Tx FIFO
Arguments:
	dev 		- Point to usb device structure
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
*********************************************************************+**************/
static int smsc7500_stop_tx_path(struct usbnet * dev)
{
	u32 Value32;
    int retVal = SMSC7500_FAIL;
    int Count = 0;

    SMSC_DEBUG(DBG_TX,"--> smsc7500_stop_tx_path\n");

	CHECK_RETURN_STATUS(reset_reg_bits(dev, MAC_TX, MAC_TX_TXEN));
	do{
	   CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MAC_TX, &Value32));
	}
	while ( (++Count<1000) && ((Value32 & MAC_TX_TXD)==0) );

    //Disable Tx FIFO
    Count = 0;
    CHECK_RETURN_STATUS(reset_reg_bits(dev, FCT_TX_CTL, FCT_TX_CTL_EN));
 	do{
	   CHECK_RETURN_STATUS(smsc7500_read_reg(dev, FCT_TX_CTL, &Value32));
	}
	while ( (++Count<1000) && ((Value32 & FCT_TX_CTL_TX_DISABLED)==0) );

	//Reset Tx FIFO
	Value32 |= FCT_TX_CTL_RST;
	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, FCT_TX_CTL, Value32));

	SMSC_DEBUG(DBG_TX,"<-- smsc7500_stop_tx_path\n");
	retVal = SMSC7500_SUCCESS;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    Starts the Receive path.

    Note that if we are operating in USB bandwidth friendly mode we defer
    starting the bulk in requests for now (will start when operational mode pipe signals
    receive data available).
Arguments:
    dev 			- pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_start_rx_path(struct usbnet *dev )
{
    int retVal = SMSC7500_FAIL;
	// Enable Rx at MAC
    CHECK_RETURN_STATUS(set_reg_bits(dev, MAC_RX, MAC_RX_RXEN));
    CHECK_RETURN_STATUS(set_reg_bits(dev, FCT_RX_CTL, FCT_RX_CTL_EN));

	retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine Stops the Rx Path at both Mac and Rx FIFO
Arguments:
	dev 		- Point to usb device structure
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_stop_rx_path(struct usbnet * dev)
{
	u32 Value32;
    int retVal = SMSC7500_FAIL;
    int Count = 0;

  	SMSC_DEBUG(DBG_RX,"--> smsc7500_stop_rx_path\n");

	// Stop the Receive path
	CHECK_RETURN_STATUS(reset_reg_bits(dev, MAC_RX, MAC_RX_RXEN));
	do{
	   CHECK_RETURN_STATUS(smsc7500_read_reg(dev, MAC_RX, &Value32));
	}
	while ( (++Count<1000) && ((Value32 & MAC_RX_RXD)==0) );
	
		
    //Disable Rx FIFO
    Count = 0;
    CHECK_RETURN_STATUS(reset_reg_bits(dev, FCT_RX_CTL, FCT_RX_CTL_EN));
 	do{
	   CHECK_RETURN_STATUS(smsc7500_read_reg(dev, FCT_RX_CTL, &Value32));
	}
	while ( (++Count<1000) && ((Value32 & FCT_RX_CTL_RX_DISABLED)==0) );


	//Reset Rx FIFO
    CHECK_RETURN_STATUS(set_reg_bits(dev, FCT_RX_CTL, FCT_RX_CTL_RST));
	

    SMSC_DEBUG(DBG_RX,"<-- smsc7500_stop_rx_path\n");
    retVal = SMSC7500_SUCCESS;

DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine calculates CRC16 checksum.
Arguments:
	bpData 		-
Return Value:
	CRC16 value
***********************************************************************************/
static u16 calculate_crc16(const BYTE * bpData,const u32 dwLen, const BOOLEAN fBitReverse)
{
	const u16 wCrc16Poly = 0x8005U;	// s/b international standard for CRC-16
								// x^16 + x^15 + x^2 + 1
	//u16 wCrc16Poly = 0xA001;	// reverse
	u16 i, j, bit;
	u16 wCrc = 0xFFFFU;
	u16 wMsb;
	BYTE bCurrentByte;
	u16 wNumOfBits = 16U;
	u16 wCrcOut=0;

	for (i=0; i<(u16)dwLen; i++)
	{
		bCurrentByte = *bpData++;

		for (bit=(u16)0U; bit<(u16)8U; bit++)
		{
			wMsb = wCrc >> 15;
			wCrc <<= 1;

			if (wMsb ^ (u16)(bCurrentByte & 1))
			{
				wCrc ^= wCrc16Poly;
				wCrc |= (u16)0x0001U;
			}
			bCurrentByte >>= 1;
		}
	}
	//bit reverse if needed
	// so far we do not need this for 117
	// but the standard CRC-16 seems to require this.
	if (fBitReverse)
	{
		j = 1;
		for (i=(u16)(1<<(wNumOfBits-(u16)1U)); i; i = i>>1) {
			if (wCrc & i)
			{
				wCrcOut |= j;
			}
			j <<= 1;
		}
		wCrc = wCrcOut;
	}

	return wCrc;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0))
#ifdef SMSC7500_VLAN_ACCEL_SUPPORT
static void smsc7500_vlan_rx_register(struct net_device *ndev, struct vlan_group *grp)
{
	struct usbnet *dev = netdev_priv(ndev);

	SMSC_DEBUG(DBG_INIT, "Set smsc7500_vlan_rx_register, grp = 0x%x\n", (u32)grp);
	dev->vlgrp = grp;

}
#endif
#endif
/**************************************************************************************
Routine Description:
            This routine sets device at suspend3 state.
Arguments:
    netdev - pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int SetWakeupOnSuspend3(struct net_device *netdev)
{
	struct usbnet * dev=netdev_priv(netdev);
	int retVal = SMSC7500_FAIL;
	u32 Value32;
	BUG_ON(!dev);

	SMSC_DEBUG(DBG_PWR,"Setting Suspend3 mode\n");

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, FCT_RX_CTL, &Value32));

	if((Value32 & 0xFFFF) != 0){
		SMSC_DEBUG(DBG_PWR,"Rx FIFO is not empty, abort suspend\n");
		goto DONE;
	}else{
		SMSC_DEBUG(DBG_PWR,"Rx FIFO is empty, continue suspend\n");
	}

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &Value32));
	Value32 &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
	Value32 |= PMT_CTL_SUS_MODE_3 | PMT_CTL_RES_CLR_WKP_EN;
	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, Value32));
	Value32 &= ~PMT_CTL_WUPS;
	Value32 |= PMT_CTL_WUPS_WOL; //Clear wol status
	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, Value32));

	CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

	retVal = SMSC7500_SUCCESS;

DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine sets wakeup event.
Arguments:
    netdev - pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int SetWakeupEvents(struct net_device *netdev)
{
	u32 dwValue;
	int i=0, retVal = SMSC7500_FAIL;
	int filter = 0;
	u32 filterCfg = 0;

	struct usbnet * dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	u32 opts = adapterData->WolWakeupOpts;

	BYTE bMcast[MCAST_LEN] = {0x01, 0x00, 0x5E};
	BYTE bArp[ARP_LEN] = {0x08, 0x06};

	BUG_ON(!netdev);

	SMSC_DEBUG(DBG_PWR,"In SetWakeupEvents. \n");

	if(opts & (WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC))
	{
		if(opts & WAKE_PHY) {
		// Clear any pending Phy interrupt and enable the mask for it
			adapterData->systemSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN;
			if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE)
				adapterData->systemSuspendPHYEvent |= PHY_INT_MASK_ANEG_COMP;

			enable_PHY_wakeupInterrupt(dev, adapterData->systemSuspendPHYEvent);

			// If there's currently no link we can use Suspend1
			CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &dwValue));
			CHECK_RETURN_STATUS(smsc7500_read_phy(dev, PHY_BSR, &dwValue));
			if (!(dwValue & PHY_BSR_LINK_STATUS))
			{
				SMSC_DEBUG(DBG_PWR,"Setting PHY in PD/ED Mode.\n");

				/* If we are in 100 force mode, set the NWAY */
				if(((adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) == 0) &&
					((adapterData->actualLinkMode & LINK_SPEED_100HD) || 
					(adapterData->actualLinkMode & LINK_SPEED_100FD))) {
					CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_BCR, PHY_BCR_AUTO_NEG_ENABLE));
				}

				//Enable the energy detect power-down mode
				CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_MODE_CSR, PHY_MODE_CSR_EDPWRDOWN));
				CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_INT_SRC, 0xFFFF)); //Write to clear
				//Enable ENERGYON interrupt source
				CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ENERGY_ON));

				// Put Lan7500 in Suspend1
				SMSC_DEBUG(DBG_PWR,"Setting Suspend1 mode\n");
				CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
				dwValue &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
				dwValue |= PMT_CTL_SUS_MODE_1;
				CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
				dwValue &= ~PMT_CTL_WUPS;
				dwValue |= (PMT_CTL_WUPS_ED | PMT_CTL_ED_EN); //Clear wol status, enable energy detection
				CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));

				CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));
			}
			else
			{
				// Put Lan7500 in Suspend0
				SMSC_DEBUG(DBG_PWR,"Setting Suspend0 mode\n");
				CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
				dwValue &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
				dwValue |= PMT_CTL_SUS_MODE_0;
				CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
				dwValue |= PMT_CTL_WUPS;
				//dwValue |= PMT_CTL_WUPS_ED; //Clear wol status
				CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
				CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));
			}
		}
		else
		{
			// Clear any pending Phy interrupt and disable the mask for it
			dwValue = PHY_INT_MASK_LINK_DOWN;
			if(adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE)
				dwValue |= PHY_INT_MASK_ANEG_COMP;
			Disable_PHY_wakeupInterrupt(dev, dwValue);
		}


		if(opts & (WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_UCAST|WAKE_MAGIC))
		{
			if(opts & (WAKE_MCAST | WAKE_ARP)){

					//Disable all filters
					for(i=0; i<WUFF_NUM; i++){
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_CFGX + i * 4, 0));
					}
					if(opts & WAKE_MCAST) {
						SMSC_DEBUG(DBG_PWR,"Set multicast detection\n");
						filterCfg = WUF_CFGX_EN | WUF_CFGX_ATYPE_MULTICAST | calculate_crc16(bMcast, MCAST_LEN, FALSE);
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_CFGX + filter * 4, filterCfg));

						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16, 0x0007));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 4, 0x0000));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 8, 0x0000));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 12, 0x0000));

						filter++;
					}
					if(opts & WAKE_ARP) {
						SMSC_DEBUG(DBG_PWR,"Set ARP detection\n");
						filterCfg = WUF_CFGX_EN | WUF_CFGX_ATYPE_ALL | (0x0C << WUF_CFGX_PATTERN_OFFSET_SHIFT) | calculate_crc16(bArp, ARP_LEN, FALSE);
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_CFGX + filter * 4, filterCfg));

						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16, 0x0003));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 4, 0x0000));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 8, 0x0000));
						CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUF_MASKX + filter * 16 + 12, 0x0000));

						filter++;
					}
					// Clear any pending pattern match packet status
					// Enable pattern match packet wake
					CHECK_RETURN_STATUS(smsc7500_read_reg(dev, WUCSR, &dwValue));
					dwValue |= WUCSR_WUFR;
					CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUCSR, dwValue));

					CHECK_RETURN_STATUS(smsc7500_read_reg(dev, WUCSR, &dwValue));
					dwValue |= WUCSR_WUEN;
					CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUCSR, dwValue));
			}else{
					CHECK_RETURN_STATUS(smsc7500_read_reg(dev, WUCSR, &dwValue));
					dwValue &= (~WUCSR_WUEN);
					CHECK_RETURN_STATUS(smsc7500_write_reg(dev, WUCSR, dwValue));
			}

			CHECK_RETURN_STATUS(reset_reg_bits(dev, WUCSR, WUCSR_MPEN | WUCSR_BCST_EN | WUCSR_PFDA_EN));

			if (opts & WAKE_MAGIC){
				SMSC_DEBUG(DBG_PWR,"Setting magic packet detection\n");
				// Clear any pending magic packet status and enable MPEN
				CHECK_RETURN_STATUS(set_reg_bits(dev, WUCSR, WUCSR_MPR | WUCSR_MPEN));
			}
			if (opts & WAKE_BCAST){
				SMSC_DEBUG(DBG_PWR,"Setting broadicast packet detection\n");
				// Clear any pending magic packet status and enable BCST wake
				CHECK_RETURN_STATUS(set_reg_bits(dev, WUCSR, WUCSR_BCAST_FR | WUCSR_BCST_EN));
			}
			if (opts & WAKE_UCAST){
				SMSC_DEBUG(DBG_PWR,"Setting unicast packet detection\n");
				// Clear any pending magic packet status and enable BCST wake
				CHECK_RETURN_STATUS(set_reg_bits(dev, WUCSR, WUCSR_WUFR | WUCSR_PFDA_EN));
			}

			//Enable recevier
			CHECK_RETURN_STATUS(set_reg_bits(dev, MAC_RX, MAC_RX_RXEN));

			SMSC_DEBUG(DBG_PWR,"Setting Suspend0 mode\n");
			CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
			dwValue &= (~(PMT_CTL_SUS_MODE | PMT_CTL_PHY_RST));
			dwValue |= PMT_CTL_SUS_MODE_0 | PMT_CTL_WOL_EN | PMT_CTL_WUPS;
			CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL,  dwValue));

			CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));
		}

	}else {

		SMSC_DEBUG(DBG_PWR,"Disabling Wake events. ");

		// Disable Energy detect (Link up) & Wake up events to do USB wake
		CHECK_RETURN_STATUS(reset_reg_bits(dev, WUCSR, WUCSR_MPEN | WUCSR_WUEN));
		CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_ED_EN | PMT_CTL_WOL_EN));

		// Put Lan7500 in Suspend2
		SMSC_DEBUG(DBG_PWR,"Setting Suspend2 mode\n");
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
		dwValue &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
		dwValue |= PMT_CTL_SUS_MODE_2;
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
	}

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine disables wakeup event.
Arguments:
    netdev - pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int ResetWakeupEvents(struct net_device *netdev)
{
	int retVal = SMSC7500_FAIL;

	struct usbnet * dev=netdev_priv(netdev);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	BUG_ON(!adapterData);
	SMSC_DEBUG(DBG_PWR,"In ResetWakeupEvents. \n");

	if(!(adapterData->WolWakeupOpts & (WAKE_PHY | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_MAGIC)))
		return retVal;

	smsc7500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);
	if(adapterData->WolWakeupOpts & WAKE_PHY) {
		//Disable the energy detect power-down mode
		CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_MODE_CSR, PHY_MODE_CSR_EDPWRDOWN));

		//Disable energy-detect wake-up
		CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_PHY_RST));
		CHECK_RETURN_STATUS(set_reg_bits(dev, PMT_CTL, PMT_CTL_WUPS)); //Clear wake-up status

		/* If 100 force mode, re-set the NWAY */
		if(((adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) == 0) &&
			((adapterData->actualLinkMode & LINK_SPEED_100HD) || 
			(adapterData->actualLinkMode & LINK_SPEED_100FD))) {
			CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_BCR, PHY_BCR_AUTO_NEG_ENABLE));
		}

		Disable_PHY_wakeupInterrupt(dev, adapterData->systemSuspendPHYEvent);
	}else{

		if(adapterData->WolWakeupOpts & (WAKE_BCAST | WAKE_MCAST | WAKE_ARP | WAKE_UCAST)){
			CHECK_RETURN_STATUS(reset_reg_bits(dev, WUCSR, WUCSR_WUEN)); //Disable Wake-up frame detection
		}
		if(adapterData->WolWakeupOpts & WAKE_MAGIC){//Set Magic packet detection
			CHECK_RETURN_STATUS(reset_reg_bits(dev, WUCSR, WUCSR_MPEN)); //Disable magic frame detection
		}
		//Disable wake-up frame interrupt
		CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_WOL_EN));
		CHECK_RETURN_STATUS(set_reg_bits(dev, PMT_CTL, PMT_CTL_WUPS)); //Clear wake-up status
	}

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine sets linkdown wakeup event.
Arguments:
    netdev 			- pointer to our device
    wakeUpMode		-	WAKEPHY_OFF,
						WAKEPHY_NEGO_COMPLETE,
						WAKEPHY_ENERGY
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int set_linkdown_wakeup_events(struct usbnet *dev, int wakeUpMode)
{
	u32 dwValue;
	int retVal = SMSC7500_FAIL;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	BUG_ON(!dev);

	SMSC_DEBUG(DBG_PWR,"In set_linkdown_wakeup_events. \n");

	CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
	dwValue |= PMT_CTL_ED_EN;
	CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));

	SMSC_DEBUG(DBG_PWR,"Setting PHY in PD/ED Mode\n");

	if(wakeUpMode == WAKEPHY_ENERGY){
		/* If we are in 100 force mode, set the NWAY */
		if(((adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) == 0) &&
			((adapterData->actualLinkMode & LINK_SPEED_100HD) || 
			(adapterData->actualLinkMode & LINK_SPEED_100FD))) {
			CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_BCR, PHY_BCR_AUTO_NEG_ENABLE));
		}

		//Enable the energy detect power-down mode
		CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_MODE_CSR, PHY_MODE_CSR_EDPWRDOWN));

		CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_INT_SRC, 0xFFFF)); //Write to clear

		//Enable interrupt source
		CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ENERGY_ON | PHY_INT_MASK_ANEG_COMP));

		CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
		CHECK_RETURN_STATUS(set_reg_bits(dev, PMT_CTL, PMT_CTL_SUS_MODE_1));
		CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

		SMSC_DEBUG(DBG_PWR,"Setting Suspend1 mode\n");

	}else if (wakeUpMode == WAKEPHY_NEGO_COMPLETE){
		//Disable the energy detect power-down mode
		CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_MODE_CSR, PHY_MODE_CSR_EDPWRDOWN));
		CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_INT_SRC, 0xFFFF)); //Write 1 to clear
		//Enable interrupt source
		CHECK_RETURN_STATUS(set_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ANEG_COMP));

		CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
		CHECK_RETURN_STATUS(set_reg_bits(dev, PMT_CTL, PMT_CTL_SUS_MODE_0));
		CHECK_RETURN_STATUS(smsc7500_set_feature(dev,USB_DEVICE_REMOTE_WAKEUP));

		SMSC_DEBUG(DBG_PWR,"Setting Suspend0 mode\n");

	}

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine resets linkdown wakeup event.
Arguments:
    netdev 			- pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int reset_linkdown_wakeup_events(struct usbnet *dev)
{
	int retVal = SMSC7500_FAIL;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	SMSC_DEBUG(DBG_PWR,"In reset_linkdown_wakeup_events. \n");

	smsc7500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);

	//Disable the energy detect power-down mode
	CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_MODE_CSR, PHY_MODE_CSR_EDPWRDOWN));

	//Disable ENERGYON interrupt source
	CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ENERGY_ON));
	if(dev->linkDownSuspend == WAKEPHY_ENERGY){
		CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ENERGY_ON | PHY_INT_MASK_ANEG_COMP));
	}else if (dev->linkDownSuspend == WAKEPHY_NEGO_COMPLETE){
		CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_INT_MASK, PHY_INT_MASK_ANEG_COMP));
	}

	//Disable energy-detect wake-up
	CHECK_RETURN_STATUS(reset_reg_bits(dev, PMT_CTL, PMT_CTL_ED_EN));

	/* If 100 force mode, re-set the NWAY */
	if(((adapterData->LinkAdvertisedCapabilites & LINK_AUTO_NEGOTIATE) == 0) &&
		((adapterData->actualLinkMode & LINK_SPEED_100HD) || 
		(adapterData->actualLinkMode & LINK_SPEED_100FD))) {
		CHECK_RETURN_STATUS(reset_phy_bits(dev, PHY_BCR, PHY_BCR_AUTO_NEG_ENABLE));
	}

    retVal = 0;
DONE:
    return retVal;
}

/**************************************************************************************
Routine Description:
    This routine enables PHY interrupt
Arguments:
    dev 			- pointer to our device
    interrupt		- Bit mask for PHY interrupt
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int enable_PHY_wakeupInterrupt(struct usbnet *dev, u32 interrupt)
{
	u32 dwValue = 0;
	int retVal = SMSC7500_FAIL;

	BUG_ON(!dev);

	SMSC_DEBUG(DBG_PWR,"In enable_PHY_wakeupInterrupt. \n");

	CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_INT_SRC, 0xFFFF)); //Write to clear

	//Enable interrupt source
	CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_INT_MASK,&dwValue));
	dwValue |= interrupt;
	CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_INT_MASK,dwValue));

	if(dwValue){
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
		dwValue &= ~PMT_CTL_PHY_RST;
		dwValue |= PMT_CTL_ED_EN;
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
	}

	retVal = SMSC7500_SUCCESS;
DONE:
	return retVal;

}

/**************************************************************************************
Routine Description:
    This routine disables linkdown interrupt in the PHY
Arguments:
    dev 			- pointer to our device
    interrupt		- Bit mask for PHY interrupt
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int Disable_PHY_wakeupInterrupt(struct usbnet *dev, u32 interrupt)
{
	u32 dwValue;
	int retVal = SMSC7500_FAIL;

	BUG_ON(!dev);

	SMSC_DEBUG(DBG_PWR,"In Disable_PHY_wakeupInterrupt. \n");

	CHECK_RETURN_STATUS(smsc7500_write_phy(dev, PHY_INT_SRC, 0xFFFF)); //Write 1 to clear

	//Disable interrupt source
	CHECK_RETURN_STATUS(smsc7500_read_phy(dev,PHY_INT_MASK,&dwValue));
	dwValue &= PHY_INT_MASK_ALL;
	dwValue &= ~interrupt;
	CHECK_RETURN_STATUS(smsc7500_write_phy(dev,PHY_INT_MASK,dwValue));

	if(dwValue == 0){ //All interrupt sources are disabled
		CHECK_RETURN_STATUS(smsc7500_read_reg(dev, PMT_CTL, &dwValue));
		dwValue &= ~(PMT_CTL_PHY_RST | PMT_CTL_ED_EN);
		CHECK_RETURN_STATUS(smsc7500_write_reg(dev, PMT_CTL, dwValue));
	}

	retVal = SMSC7500_SUCCESS;

DONE:
	return retVal;

}

/**************************************************************************************
Routine Description:
	Suspend device.
Arguments:

Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11))
static int smsc7500_suspend (struct usb_interface *intf,  u32 state)
#else
static int smsc7500_suspend (struct usb_interface *intf, pm_message_t state)
#endif
{

	struct usbnet		*dev = usb_get_intfdata(intf);
	int retVal = SMSC7500_SUCCESS;
	u32 dwValue;

    SMSC_DEBUG(DBG_PWR,"---->Smsc7500_suspend\n");
    BUG_ON(!dev);
    BUG_ON(!dev->udev);

    smsc7500_read_reg(dev,WUCSR,&dwValue);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33))
    if(dev->udev->auto_pm) //Internal pm event, autosuspend
#else
    if (state.event & PM_EVENT_AUTO)
#endif
#else
    if(0)
#endif //CONFIG_PM
#else
    if(0)
#endif
    {
    	retVal = smsc7500_autosuspend(intf, state);
    }else
    {//It is system suspend
    	retVal = smsc7500_system_suspend(intf, state);
    }

    SMSC_DEBUG(DBG_PWR,"<----Smsc7500_suspend\n");

	return retVal;
}

/**************************************************************************************
Routine Description:
	Auto suspend device.
Arguments:

Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_autosuspend (struct usb_interface *intf, pm_message_t state)
{

	struct usbnet		*dev = usb_get_intfdata(intf);
	u32 Value32;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	int suspendFlag = dev->suspendFlag;
	int retVal = SMSC7500_SUCCESS;

	SMSC_DEBUG(DBG_PWR,"---->smsc7500_autosuspend, suspendFlag = 0x%x\n", suspendFlag);

	dev->suspendFlag = 0;

	if (netif_running (dev->net))
	{
		adapterData->wakeupOptsBackup = adapterData->WolWakeupOpts;

		if(smsc7500_read_phy(dev, PHY_BSR, &Value32) < 0){
			return SMSC7500_FAIL;
		}
		if(smsc7500_read_phy(dev, PHY_BSR, &Value32) < 0){
			return SMSC7500_FAIL;
		}

		if (!(Value32 & PHY_BSR_LINK_STATUS))//last minute link check
		{//Link is down
			if(dev->netDetach){//Highest priority
				if(!dev->pmLock){//Resume immediately, then detach device from USB bus.
					smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
				}
				retVal = SMSC7500_FAIL;
				goto _SUSPEND_EXIT;
			}else if(dev->linkDownSuspend || (suspendFlag & AUTOSUSPEND_LINKDOWN)){//Always check it to save more power
				dev->suspendFlag |= AUTOSUSPEND_LINKDOWN;
				retVal = set_linkdown_wakeup_events(dev, dev->linkDownSuspend);
			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC){//suspend on s0
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC;
				adapterData->WolWakeupOpts = (WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP);
				retVal = SetWakeupEvents(dev->net);

				//Save PHY interrupt event, so we can clear it after waking up.
				adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_ENERGY_ON | PHY_INT_MASK_ANEG_COMP;
				retVal = enable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);

			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//suspend on s3
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
				retVal = SetWakeupOnSuspend3(dev->net);
				if(retVal != SMSC7500_FAIL){
					//Save PHY interrupt event, so we can clear it after waking up.
					adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_ENERGY_ON | PHY_INT_MASK_ANEG_COMP;
					retVal = enable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
				}

			}else{
				SMSC_WARNING("auto suspend event is null\n");
				retVal = SMSC7500_FAIL;
			}

		}else{//link is up

			 if(suspendFlag & AUTOSUSPEND_DYNAMIC){//suspend on s0
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC;
				adapterData->WolWakeupOpts = (WAKE_UCAST | WAKE_BCAST | WAKE_MCAST | WAKE_ARP);
				retVal = SetWakeupEvents(dev->net);

				//Save PHY interrupt event, so we can clear it after waking up.
				adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN;
				retVal = enable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
			}else if(suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//suspend on s3
				dev->suspendFlag |= AUTOSUSPEND_DYNAMIC_S3;
				retVal = SetWakeupOnSuspend3(dev->net);
				if(retVal != SMSC7500_FAIL){
					//Save PHY interrupt event, so we can clear it after waking up.
					adapterData->dynamicSuspendPHYEvent = PHY_INT_MASK_LINK_DOWN;
					retVal = enable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
				}
			}else{// if(suspendFlag & AUTOSUSPEND_LINKDOWN){//Resume immediately
				retVal = SMSC7500_FAIL;
			}
		}

		if(retVal != SMSC7500_SUCCESS){
			SMSC_WARNING("Failed to suspend device\n");
			dev->suspendFlag = 0;
			if(!dev->pmLock){//Resume immediately
				smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
			}
			goto _SUSPEND_EXIT;
		}else{
			smscusbnet_FreeQueue(dev);
			del_timer_sync(&dev->LinkPollingTimer);
		}

	}else{//Interface down
		u32 Value32;
		SMSC_DEBUG(DBG_PWR,"Interface is down, set suspend2 mode\n");
		smsc7500_read_reg(dev, PMT_CTL, &Value32);
		Value32 &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
		Value32 |= PMT_CTL_SUS_MODE_2;
		smsc7500_write_reg(dev, PMT_CTL, Value32);

		dev->suspendFlag |= AUTOSUSPEND_INTFDOWN;
	}

_SUSPEND_EXIT:

	SMSC_DEBUG(DBG_PWR,"<----smsc7500_autosuspend\n");
	return retVal;
}

/**************************************************************************************
Routine Description:
	This routine will stop Rx/Tx path and Set wakeup events.
Arguments:

Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_system_suspend (struct usb_interface *intf, pm_message_t state)
{

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_DEBUG(DBG_PWR,"---->smsc7500_system_suspend\n");

	dev->idleCount = 0;
	dev->suspendFlag = 0;

	if (dev->net && netif_running (dev->net) && netif_device_present (dev->net))
	{
		netif_device_detach (dev->net);
		smscusbnet_FreeQueue(dev);

		dev->StopLinkPolling=TRUE;
		del_timer_sync(&dev->LinkPollingTimer);

		tx_stop_queue(dev,TX_QUEUE_SRC_WOL);
		smsc7500_stop_tx_path(dev);
		smsc7500_stop_rx_path(dev);
		SetWakeupEvents(dev->net);

	}else{
		// Put Lan7500 in Suspend2
		u32 Value32;
		SMSC_DEBUG(DBG_PWR,"Setting Suspend2 mode\n");
		smsc7500_read_reg(dev, PMT_CTL, &Value32);
		Value32 &= (~(PMT_CTL_SUS_MODE | PMT_CTL_WUPS | PMT_CTL_PHY_RST));
		Value32 |= PMT_CTL_SUS_MODE_2;
		smsc7500_write_reg(dev, PMT_CTL, Value32);

		dev->suspendFlag |= AUTOSUSPEND_INTFDOWN;
	}

	SMSC_DEBUG(DBG_PWR,"<----smsc7500_system_suspend\n");

	return SMSC7500_SUCCESS;
}

/**************************************************************************************
Routine Description:
	This routine will call autoresume or system resume function.
Arguments:
	intf		- usb interface
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_resume(struct usb_interface *intf)
{
	int retVal = SMSC7500_SUCCESS;

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_DEBUG(DBG_PWR,"--->in Smsc7500_resume\n");
	BUG_ON(!dev);
	BUG_ON(!dev->udev);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	 //autoresume
    if(dev->suspendFlag)
#else
    if(0)
#endif
    {
    	retVal = smsc7500_autoresume(intf);
    }else
    {
    	retVal = smsc7500_system_resume(intf);
    }
	SMSC_DEBUG(DBG_PWR,"------->out of in Smsc7500_resume\n");
	return retVal;
}

/**************************************************************************************
Routine Description:
	This routine will reset wakeup event, start tasklet and tx queue.
Arguments:
	intf		- usb interface
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_autoresume(struct usb_interface *intf)
{
	struct usbnet		*dev = usb_get_intfdata(intf);
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);
	u32 dwValue;

	SMSC_DEBUG(DBG_PWR,"--->in smsc7500_autoresume\n");

	clear_bit(EVENT_DEV_RECOVERY, &dev->flags);

	if (dev->suspendFlag & AUTOSUSPEND_INTFDOWN){
		SMSC_DEBUG(DBG_PWR,"Resume from interface down\n");
		goto _EXIT_AUTORESUME;
	}

	if(!dev->pmLock){
		smscusbnet_defer_myevent(dev, EVENT_IDLE_RESUME);
	}

	if(dev->suspendFlag & AUTOSUSPEND_LINKDOWN){
		reset_linkdown_wakeup_events(dev);
	}else if(dev->suspendFlag & AUTOSUSPEND_DYNAMIC){
		Disable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);
		ResetWakeupEvents(dev->net);
	}else if(dev->suspendFlag & AUTOSUSPEND_DYNAMIC_S3){//Resume from suspend3
		smsc7500_clear_feature(dev,USB_DEVICE_REMOTE_WAKEUP);

		Disable_PHY_wakeupInterrupt(dev, adapterData->dynamicSuspendPHYEvent);

		if(smsc7500_read_reg(dev, PMT_CTL, &dwValue) < 0){
			SMSC_WARNING("Failed to read PMT_CTL");
		}
		dwValue &= (~(PMT_CTL_SUS_MODE | PMT_CTL_PHY_RST));
		dwValue |= PMT_CTL_SUS_MODE_2;

		if(smsc7500_write_reg(dev, PMT_CTL, dwValue) < 0){
			SMSC_WARNING("Failed to write PMT_CTL");
		}
		dwValue &= ~PMT_CTL_WUPS;
		dwValue |= PMT_CTL_WUPS_WOL;

		if(smsc7500_write_reg(dev, PMT_CTL, dwValue) < 0){ //Should not change suspend_mode while clearing WUPS_sts[1]
			SMSC_WARNING("Failed to write PMT_CTL");
		}

	}
	tasklet_schedule (&dev->bh);

	if(adapterData->wakeupOptsBackup){
		adapterData->WolWakeupOpts = adapterData->wakeupOptsBackup;
		adapterData->wakeupOptsBackup = 0;
	}

    tx_wake_queue(dev,TX_QUEUE_SRC_WOL);

_EXIT_AUTORESUME:
    dev->idleCount = 0;
    dev->suspendFlag = 0;

	SMSC_DEBUG(DBG_PWR,"------->out of in smsc7500_autoresume\n");
	return SMSC7500_SUCCESS;
}

/**************************************************************************************
Routine Description:
	This routine will reset wakeup event, start tasklet and tx queue.
Arguments:
	intf		- usb interface
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_system_resume(struct usb_interface *intf)
{
	int retVal=0;

	struct usbnet		*dev = usb_get_intfdata(intf);

	SMSC_DEBUG(DBG_PWR,"--->in smsc7500_system_resume\n");

    if(netif_running (dev->net) && !netif_device_present (dev->net)){
       netif_device_attach (dev->net);
    }

    retVal=smsc7500_reset(dev);
    smsc7500_start_tx_path(dev);
	smsc7500_start_rx_path(dev);
    ResetWakeupEvents(dev->net);

    tx_wake_queue(dev,TX_QUEUE_SRC_WOL);

	init_timer(&(dev->LinkPollingTimer));
	dev->StopLinkPolling=FALSE;
	dev->LinkPollingTimer.function=smscusbnet_linkpolling;
	dev->LinkPollingTimer.data=(unsigned long) dev;
	dev->LinkPollingTimer.expires=jiffies+HZ;
	add_timer(&(dev->LinkPollingTimer));
	tasklet_schedule (&dev->bh);

    dev->idleCount = 0;

	SMSC_DEBUG(DBG_PWR,"------->out of in smsc7500_system_resume\n");
	return SMSC7500_SUCCESS;
}

/**************************************************************************************
Routine Description:
	This routine will reinit device and restart tasklet.
Arguments:
	   dev 			- pointer to our device
Return Value:
	SMSC7500_SUCCESS	- Success
	SMSC7500_FAIL		- Operation fail
***********************************************************************************/
static int smsc7500_device_recovery(struct usbnet *dev)
{
    u32 dwReadBuf;

    BUG_ON(!dev);

    if (dev->net && netif_device_present (dev->net))
    {

        SMSC_WARNING("Device recovery is in progress\n");

        if (smsc7500_read_reg(dev,INT_STS,&dwReadBuf)< 0)return SMSC7500_FAIL;
		printk("INT_STS =0x%x\n", dwReadBuf);

        smscusbnet_FreeQueue(dev);

        dev->StopLinkPolling=TRUE;
        del_timer_sync(&dev->LinkPollingTimer);

        tx_stop_queue(dev,TX_QUEUE_SRC_WOL);

        if(smsc7500_stop_tx_path(dev) < 0)return SMSC7500_FAIL;
        if(smsc7500_stop_rx_path(dev) < 0)return SMSC7500_FAIL;

        if(dwReadBuf & INT_STS_TXE_INT){// reset only when TXE occurs
            if(smsc7500_reset(dev) < 0)return SMSC7500_FAIL;
        }

        if(smsc7500_start_tx_path(dev) < 0)return SMSC7500_FAIL;
        if(smsc7500_start_rx_path(dev) < 0)return SMSC7500_FAIL;

        tx_wake_queue(dev,TX_QUEUE_SRC_WOL);

        init_timer(&(dev->LinkPollingTimer));
        dev->StopLinkPolling=FALSE;
        dev->LinkPollingTimer.function=smscusbnet_linkpolling;
        dev->LinkPollingTimer.data=(unsigned long) dev;
        dev->LinkPollingTimer.expires=jiffies+HZ;
        add_timer(&(dev->LinkPollingTimer));

        tasklet_schedule (&dev->bh);

        SMSC_WARNING("Device recovery is done\n");
    }

    return SMSC7500_SUCCESS;
}

static int Phy_gig_workaround(struct usbnet *dev)
{
	int retVal=0, Timeout;
	u32 dwData;
	PADAPTER_DATA  adapterData=(PADAPTER_DATA)(dev->data[0]);

	/* Only internal phy */
	adapterData->dwPhyAddress=1;

	/* Set the phy in Gig loopback */
	if ((retVal = smsc7500_write_phy(dev, PHY_BCR, 0x4040)) < 0) {
		SMSC_WARNING("Phy setting Gig loopback failed: 0x%x", retVal);
		return SMSC7500_FAIL;
	}
	/* Wait for the link up */
	smsc7500_read_phy(dev, PHY_BSR, &dwData);
	Timeout = 0;
	do {
		if ((retVal = smsc7500_read_phy(dev, PHY_BSR, &dwData)< 0)) {
			SMSC_WARNING("Failed to read PHY_BSR: 0x%x", retVal);
			return SMSC7500_FAIL;
		}
		msleep(10);
		Timeout++;
	} while ( (!(dwData & PHY_BSR_LINK_STATUS)) && (Timeout < 1000));

	if(Timeout >= 1000)
	{
		SMSC_WARNING("Timeout waiting for PHY link up\n");
		return SMSC7500_FAIL;
	}

	/* phy reset */
	if((retVal = set_reg_bits(dev, PMT_CTL, PMT_CTL_PHY_RST)) < 0){
		SMSC_WARNING("Failed to access PMT_CTL: %d", retVal);
		return retVal;
	}
	Timeout = 0;
	do {
		if ((retVal = smsc7500_read_reg(dev,PMT_CTL,&dwData)< 0)) {
			SMSC_WARNING("Failed to read PMT_CTL: %d", retVal);
			return retVal;
		}
		msleep(100);
		Timeout++;
	} while ( (dwData & PMT_CTL_PHY_RST) && (Timeout < 100));

	if(Timeout >= 100)
	{
		SMSC_WARNING("Timeout waiting for PHY Reset\n");
		return SMSC7500_FAIL;
	}

	return 0;
}

static int Phy_reset(struct usbnet *dev)
{
	int retVal=0, Timeout;
	u32 dwData;

	if ((retVal = smsc7500_write_phy(dev, PHY_BCR, PHY_BCR_RESET)) < 0) {
		SMSC_WARNING("Phy reset failed: 0x%x", retVal);
		return SMSC7500_FAIL;
	}

	Timeout = 0;
	do {
		if ((retVal = smsc7500_read_phy(dev, PHY_BCR, &dwData)< 0)) {
			SMSC_WARNING("Failed to read PHY_BCR: 0x%x", retVal);
			return SMSC7500_FAIL;
		}
		msleep(10);
		Timeout++;
	} while ( (dwData & PHY_BCR_RESET) && (Timeout < 1000));

	if(Timeout >= 1000)
	{
		SMSC_WARNING("Timeout waiting for PHY Reset\n");
		return SMSC7500_FAIL;
	}

	return 0;
}

#ifdef USE_DEBUG
void static LanDumpRegs(struct usbnet *dev){
	u32 dwData, i;
    for(i=0; i<NO_OF_SYS_REGISTER; i++){
        smsc7500_read_reg(dev, lanRegMap[i], &dwData);
        SMSC_DEBUG(DBG_TRACE, "offset 0x%x  = 0x%08X", lanRegMap[i], dwData);
     }
}
#endif //USE_DEBUG

/**************************************************************************************
Routine Description:
	Module init function.
Arguments:
Return Value:
***********************************************************************************/
static int __init smsc7500_init(void)
{
 	return usb_register(&smsc7500_driver);
}

/**************************************************************************************
Routine Description:
	Module exit function.
Arguments:
Return Value:
***********************************************************************************/
static void __exit smsc7500_exit(void)
{
 	usb_deregister(&smsc7500_driver);
}

module_init(smsc7500_init);
module_exit(smsc7500_exit);

MODULE_AUTHOR("Sean(Xiang) Chen");
MODULE_DESCRIPTION("SMSC7500 USB 2.0 Ethernet Devices");
MODULE_LICENSE("GPL");

