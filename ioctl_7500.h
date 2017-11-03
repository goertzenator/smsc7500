#ifndef IOCTL_7500_H_
#define IOCTL_7500_H_

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

 * File: ioctl_500.h

 */


#define SMSC7500_DRIVER_SIGNATURE	(0x1227A5BAUL+DRIVER_VERSION)

#define SMSC7500_APP_SIGNATURE		(0x537BD128UL+DRIVER_VERSION)



#define SMSC7500_IOCTL				(SIOCDEVPRIVATE + 0xB)

enum _SMSC7500_STATS{
		rxFcsErrors,
		rxAlignmentErrors,
		rxFragmentErrors,
		rxJabberErrors,
		rxRuntFrameErrors,
		rxFrameTooLongError,
		rxDroppedFrames,
		rxUnicastByteCount,
		rxBroadcastByteCount,
		rxMulticastByteCount,
		rxUnicastFrames,
		rxBroadcastFrames,
		rxMulticastFrames,
		rxPauseFrames,
		rx64BFrames,
		rx64To127Frames,
		rx128To255Frames,
		rx256To511Frames,
		rx512To1023Frames,
		rx1024To1518Frames,
		rxOver1518Frames,

		txFcsErrors,
		txExcessDeferralErrors,
		txCarrierErrors,
		txBadBytes,
		txSingleCollisions,
		txMultipleCollisions,
		txExcessCollision,
		txLateCollision,
		txUnicastByteCount,
		txBroadcastByteCount,
		txMulticastByteCount,
		txUnicastFrames,
		txBroadcastFrames,
		txMulticastFrames,
		txPauseFrames,
		tx64BFrames,
		tx64To127Frames,
		tx128To255Frames,
		tx256To511Frames,
		tx512To1023Frames,
		tx1024To1518Frames,
		txOver1518Frames,

		NO_OF_STAS_CNT
};

enum{
    COMMAND_BASE =	0x974FB832UL,
    COMMAND_GET_SIGNATURE,
    COMMAND_LAN_GET_REG,
    COMMAND_LAN_SET_REG,
    COMMAND_PHY_GET_REG,
    COMMAND_PHY_SET_REG,
    COMMAND_DUMP_LAN_REGS,
    COMMAND_DUMP_MAC_REGS,
    COMMAND_DUMP_PHY_REGS,
    COMMAND_DUMP_EEPROM,
    COMMAND_GET_MAC_ADDRESS,
    COMMAND_SET_MAC_ADDRESS,
    COMMAND_LOAD_MAC_ADDRESS,
    COMMAND_SAVE_MAC_ADDRESS,
    COMMAND_SET_DEBUG_MODE,
    COMMAND_SET_POWER_MODE,
    COMMAND_GET_POWER_MODE,
    COMMAND_SET_LINK_MODE,
    COMMAND_GET_LINK_MODE,
    COMMAND_GET_CONFIGURATION,
    COMMAND_DUMP_TEMP,
    COMMAND_READ_BYTE,
    COMMAND_READ_WORD,
    COMMAND_READ_DWORD,
    COMMAND_WRITE_BYTE,
    COMMAND_WRITE_WORD,
    COMMAND_WRITE_DWORD,
    COMMAND_CHECK_LINK,
    COMMAND_GET_ERRORS,
    COMMAND_SET_EEPROM,
    COMMAND_GET_EEPROM,
    COMMAND_WRITE_EEPROM_FROM_FILE,
    COMMAND_READ_EEPROM_TO_FILE,
    COMMAND_VERIFY_EEPROM_WITH_FILE,

//the following codes are intended for cmd7500 only
//  they are not intended to have any use in the driver

    COMMAND_RUN_SERVER,
    COMMAND_RUN_TUNER,
    COMMAND_GET_FLOW_PARAMS,
    COMMAND_SET_FLOW_PARAMS,
    COMMAND_SET_AMDIX_STS,
    COMMAND_GET_AMDIX_STS,
    COMMAND_DUMP_STATISTICS
};

enum {
	SYS_ID_REV,
	SYS_FPGA_REV,
	SYS_BOND_CTL,
	SYS_INT_STS,
	SYS_HW_CFG,
	SYS_PMT_CTL,
	SYS_LED_GPIO_CFG,
	SYS_GPIO_CFG,
	SYS_GPIO_WAKE,
	SYS_DP_SEL,
	SYS_DP_CMD,
	SYS_DP_ADDR,
	SYS_DP_DATA,
	SYS_BURST_CAP,
	SYS_INT_EP_CTL,
	SYS_BULK_IN_DLY,
	SYS_E2P_CMD,
	SYS_E2P_DATA,
	SYS_DBG_STRAP,

	SYS_REF_CTL,
	SYS_VLAN_TYPE,

	SYS_FCT_RX_CTL,
	SYS_FCT_TX_CTL,
	SYS_FCT_RX_FIFO_END,
	SYS_FCT_TX_FIFO_END,
	SYS_FCT_FLOW,

	SYS_HS_ATTR,
	SYS_FS_ATTR,
	SYS_STRING_ATTR0,
	SYS_STRING_ATTR1,
	SYS_FLAG_ATTR,

	SYS_MAC_CR,
	SYS_MAC_RX,
	SYS_MAC_TX,
	SYS_FLOW,
	SYS_RAND_SEED,
	SYS_ERR_STS,
	SYS_RX_ADDRH,
	SYS_RX_ADDRL,
	SYS_MII_ACCESS,
	SYS_MII_DATA,

	SYS_WUCSR,
	SYS_WUF_CFGX,
	SYS_WUF_MASKX,
	SYS_ADDR_FILTX,
	SYS_WUCSR2,
	SYS_IPV4_ADDRX,
	SYS_IPV6_ADDRX,
	NO_OF_SYS_REGISTER
};

enum {
	REG_PHY_BCR,
	REG_PHY_BSR,
	REG_PHY_ID1,
	REG_PHY_ID2,
	REG_PHY_AN_ADV,
	REG_PHY_AN_LPA,
	REG_PHY_AN_EXP,
	REG_PHY_AN_NP_TX,
	REG_PHY_AN_NP_RX,
	REG_PHY_MSCR,
	REG_PHY_MSSR,
	REG_PHY_EXT_ST,
	REG_PHY_LINK_CONTROL,
	REG_PHY_MODE_CSR,
	REG_PHY_SPECIAL_MODE,
	REG_PHY_EXT_CSR,
	REG_PHY_AR_ADDR,
	REG_PHY_AR_DATA,
	REG_PHY_CSI,
	REG_PHY_CHANNEL_QUALITY,
	REG_PHY_INT_SRC,
	REG_PHY_INT_MASK,
	REG_PHY_SCSR,
	NO_PHY_REGS
};


typedef struct _SMSC7500_IOCTL_DATA {

	unsigned long dwSignature;

	unsigned long dwCommand;

	unsigned long Data[0x90];

	char Strng1[30];

	char Strng2[10];

} SMSC7500_IOCTL_DATA, *PSMSC7500_IOCTL_DATA;


#endif /*IOCTL_7500_H_*/
