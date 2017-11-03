/*++

	The information contained in this file is confidential and proprietary to
	Standard Microsystems Corporation.  No part of this file may be reproduced
	or distributed, in any form or by any means for any purpose, without the
	express written permission of Standard Microsystems Corporation.

	Standard Microsystems Corporation, ALL RIGHTS RESERVED.
	(c) COPYRIGHT 2008-2009

Module Name:

	smsc7500.h

Abstract:
    This module contains structure definitons and function prototypes.

Revision History:
	4/23/2009	schen, Initial version

Notes:

--*/


#ifndef _SMSC7500_H
#define _SMSC7500_H

/******************************************************************************************/
/************	System Control and Status Registers		************************************/
/******************************************************************************************/

enum _SYS_CS_REG{
	ID_REV			= 0x0000,
	FPGA_REV		= 0x0004,
	BOND_CTL		= 0x0008,
	INT_STS			= 0x000C,
	HW_CFG			= 0x0010,
	PMT_CTL			= 0x0014,
	LED_GPIO_CFG	= 0x0018,
	GPIO_CFG		= 0x001C,
	GPIO_WAKE		= 0x0020,
	DP_SEL			= 0x0024,
	DP_CMD			= 0x0028,
	DP_ADDR			= 0x002C,
	DP_DATA			= 0x0030,
	BURST_CAP		= 0x0034,
	INT_EP_CTL		= 0x0038,
	BULK_IN_DLY		= 0x003C,
	E2P_CMD			= 0x0040,
	E2P_DATA		= 0x0044,
	DBG_STRAP		= 0x0048,
	HS_ATTR			= 0x004C,
	FS_ATTR			= 0x0050,
	STRING_ATTR0	= 0x0054,
	STRING_ATTR1	= 0x0058,
	FLAG_ATTR		= 0x005C,
	REF_CTL			= 0x0060,
	VLAN_TYPE		= 0x0064,

	FCT_RX_CTL		= 0x0090,
	FCT_TX_CTL		= 0x0094,
	FCT_RX_FIFO_END	= 0x0098,
	FCT_TX_FIFO_END	= 0x009C,
	FCT_FLOW		= 0x00A0,

	MAC_CR			= 0x0100,
	MAC_RX			= 0x0104,
	MAC_TX			= 0x0108,
	FLOW			= 0x010C,
	RAND_SEED		= 0x0110,
	ERR_STS			= 0x0114,
	RX_ADDRH		= 0x0118,
	RX_ADDRL		= 0x011C,
	MII_ACCESS		= 0x0120,
	MII_DATA		= 0x0124,

	WUCSR			= 0x0140,
	WUF_CFGX		= 0x0144,

	WUF_MASKX		= 0x0170,

	ADDR_FILTX		= 0x0300,

	WUCSR2			= 0x0500,
	WOL_FIFO_STS	= 0x0504,

	IPV6_ADDRX		= 0x0510,
	IPV4_ADDRX		= 0x0590,

	MAX_ADDR_OF_REGISTER
};

/**************	Device ID and Revision Register (ID_REV) ************************/
#define ID_REV_CHIP_ID_MASK_   			0xffff0000UL
#define ID_REV_CHIP_REV_MASK_  			0x0000ffffUL

#define GetChipIdFromID_REV(dwReg)     	((dwReg & ID_REV_CHIP_ID_MASK_) >> 16)
#define GetChiprevFromID_REV(dwReg)    	(dwReg & ID_REV_CHIP_REV_MASK_)
/**************	FPGA Revision Register (FPGA_REV) *******************************/
#define	FPGA_REV_MASK		(0xFFUL)
/**************	Bond Out Control Register (BOND_CTL)*****************************/

/**************	Interrupt Status Register (INT_STS)******************************/
#define	INT_STS_RDFO_INT				0x00400000UL
#define	INT_STS_TXE_INT					0x00200000UL
#define	INT_STS_MACRTO_INT				0x00100000UL
#define	INT_STS_TX_DIS_INT				0x00080000UL
#define	INT_STS_RX_DIS_INT				0x00040000UL
#define	INT_STS_PHY_INT_				0x00020000UL
#define	INT_STS_MAC_ERR_INT				0x00008000UL
#define	INT_STS_TDFU					0x00004000UL
#define	INT_STS_TDFO					0x00002000UL
#define	INT_STS_GPIOS	    			0x00000FFFUL
/**************	Hardware configuration Register (HW_CFG)*************************/
#define	HW_CFG_SMDET_STS       			0x00008000UL
#define HW_CFG_SMDET_EN	       			0x00004000UL
#define HW_CFG_EEM		       			0x00002000UL
#define HW_CFG_RST_PROTECT     			0x00001000UL
#define HW_CFG_PORT_SWAP       			0x00000800UL
#define HW_CFG_PHY_BOOST       			0x00000600UL
#define HW_CFG_PHY_BOOST_NORMAL			0x00000000UL
#define HW_CFG_PHY_BOOST_4				0x00002000UL
#define HW_CFG_PHY_BOOST_8				0x00004000UL
#define HW_CFG_PHY_BOOST_12				0x00006000UL
#define HW_CFG_LEDB             		0x00000100UL
#define HW_CFG_BIR	            		0x00000080UL
#define HW_CFG_SBP	            		0x00000040UL
#define HW_CFG_IME              		0x00000020UL
#define HW_CFG_MEF              		0x00000010UL
#define HW_CFG_ETC						0x00000008UL
#define HW_CFG_BCE               		0x00000004UL
#define HW_CFG_LRST               		0x00000002UL
#define HW_CFG_SRST               		0x00000001UL
/**************	Power Management Control Register (PMT_CTL)***********************/
#define PMT_CTL_PHY_PWRUP				0x00000400UL
#define PMT_CTL_RES_CLR_WKP_EN			0x00000100UL
#define PMT_CTL_DEV_RDY					0x00000080UL
#define	PMT_CTL_SUS_MODE				0x00000060UL
/*Mode definitions*/
#define	PMT_CTL_SUS_MODE_0				0x00000000UL
#define	PMT_CTL_SUS_MODE_1				0x00000020UL
#define	PMT_CTL_SUS_MODE_2				0x00000040UL
#define	PMT_CTL_SUS_MODE_3				0x00000060UL

#define PMT_CTL_PHY_RST					0x00000010UL
#define PMT_CTL_WOL_EN					0x00000008UL
#define PMT_CTL_ED_EN					0x00000004UL
#define PMT_CTL_WUPS					0x00000003UL
/*WUPS definitions*/
#define PMT_CTL_WUPS_NO				    0x00000000UL
#define PMT_CTL_WUPS_ED				    0x00000001UL
#define PMT_CTL_WUPS_WOL				0x00000002UL
#define PMT_CTL_WUPS_MULTI			    0x00000003UL
/**************	LED General Purpose IO configuration Register (LED_GPIO_CFG)***********************/
#define LED_GPIO_CFG_LED2_FUN_SEL		0x80000000UL
#define LED_GPIO_CFG_LED10_FUN_SEL		0x40000000UL
#define LED_GPIO_CFG_LEDGPIO_EN			0x0000F000UL
/*LEDGPIO_EN definitions*/
#define LED_GPIO_CFG_LEDGPIO_EN_0		0x00001000UL
#define LED_GPIO_CFG_LEDGPIO_EN_1		0x00002000UL
#define LED_GPIO_CFG_LEDGPIO_EN_2		0x00004000UL
#define LED_GPIO_CFG_LEDGPIO_EN_3		0x00008000UL

#define LED_GPIO_CFG_GPBUF				0x00000F00UL
/*GPBUF definitions*/
#define LED_GPIO_CFG_GPBUF_0 			0x00000100UL
#define LED_GPIO_CFG_GPBUF_1 			0x00000200UL
#define LED_GPIO_CFG_GPBUF_2 			0x00000400UL
#define LED_GPIO_CFG_GPBUF_3 			0x00000800UL

#define LED_GPIO_CFG_GPDIR				0x000000F0UL
/*GPDIR definitions*/
#define LED_GPIO_CFG_GPDIR_0 			0x00000010UL
#define LED_GPIO_CFG_GPDIR_1 			0x00000020UL
#define LED_GPIO_CFG_GPDIR_2 			0x00000040UL
#define LED_GPIO_CFG_GPDIR_3 			0x00000080UL

#define LED_GPIO_CFG_GPDATA   			0x0000000FUL
#define LED_GPIO_CFG_GPDATA_0			0x00000001UL
#define LED_GPIO_CFG_GPDATA_1			0x00000002UL
#define LED_GPIO_CFG_GPDATA_2			0x00000004UL
#define LED_GPIO_CFG_GPDATA_3			0x00000008UL
/**************	General Purpose IO configuration Register (GPIO_CFG)***********************/
#define GPIO_CFG_SHIFT              	24
#define GPIO_CFG_GPEN					0xFF000000UL
#define GPIO_CFG_GPBUF					0x00FF0000UL
#define GPIO_CFG_GPDIR					0x0000FF00UL
#define GPIO_CFG_GPDATA	            	0x000000FFUL
/**************	General Purpose IO Wake Enable and polarity Register (GPIO_WAKE)************/
#define GPIO_WAKE_PHY_LINKUP_EN			0x80000000UL
#define GPIO_WAKE_POL					0x0FFF0000UL
#define GPIO_WAKE_POL_SHIFT				16
#define GPIO_WAKE_WK					0x00000FFFUL
/**************	Data Port Select Register (DP_SEL)************/
#define DP_SEL_DPRDY					0x80000000UL
#define DP_SEL_RSEL						0x0000000FUL
/*RSEL definitions*/
#define DP_SEL_URX						0x00000000UL
#define DP_SEL_VHF						0x00000001UL
#define VHF_HASH_LEN					16
#define VHF_VLAN_LEN					128
#define DP_SEL_LSO_HEAD					0x00000002UL
#define DP_SEL_FCT_RX					0x00000003UL
#define DP_SEL_FCT_TX					0x00000004UL
#define DP_SEL_DESCRIPTOR				0x00000005UL
#define DP_SEL_WOL						0x00000006UL
/**************	Data Port Command Register (DP_CMD)************/
#define DP_CMD_WRITE					0x01UL
#define DP_CMD_READ						0x00UL
/**************	Data Port Address Register (DP_ADDR)************/
/**************	Data Port Data Register (DP_DATA)************/
/**************	Burst Cap Register (BURST_CAP)************/
#define BURST_CAP_MASK					0x0000000FUL
/**************	interrupt Endpoint Control Register (INT_EP_CTL)************/
#define	INT_EP_CTL_INTEP_ON				0x80000000UL
#define	INT_EP_CTL_RDFO_EN				0x00400000UL
#define	INT_EP_CTL_TXE_EN				0x00200000UL
#define	INT_EP_CTL_MACROTO_EN			0x00100000UL
#define	INT_EP_CTL_TX_DIS_EN			0x00080000UL
#define	INT_EP_CTL_RX_DIS_EN			0x00040000UL
#define INT_EP_CTL_PHY_EN_				0x00020000UL

#define	INT_EP_CTL_MAC_ERR_EN			0x00008000UL
#define	INT_EP_CTL_TDFU_EN 				0x00004000UL
#define	INT_EP_CTL_TDFO_EN 				0x00002000UL
#define	INT_EP_CTL_RX_FIFO_EN			0x00001000UL
#define	INT_EP_CTL_GPIOX_EN				0x00000FFFUL
/**************	Bulk-In Delay Register (BULK_IN_DLY)**********************/
#define BULK_IN_DLY_MASK	        	0xFFFFUL
/**************	MII Access Register (MII_ACCESS)**********************/
#define MII_ACCESS_PHY_ADDR				0x0000F800UL
#define MII_ACCESS_PHY_ADDR_SHIFT		11
#define MII_ACCESS_MIIRINAD				0x000007C0UL
#define MII_ACCESS_MIIRINAD_SHIFT		6
#define MII_ACCESS_RW					0x00000002UL
#define MII_ACCESS_BUSY			 		0x00000001UL
/**************	MII Data Register (MII_DATA)**********************/
#define MII_DATA_MASK					0x0000FFFFUL
/**************	Strap Debug Register (DBG_STRAP)*************************/
/************** Receive filtering Engine Control Register (REF_CTL) ***************/
#define  REF_CTL_TCPUDP_CKM				0x00001000UL	// Enable TCP/UDP Checksum Validation
#define  REF_CTL_IP_CKM					0x00000800UL	// Enable IP Checksum Validation
#define  REF_CTL_AB						0x00000400UL	// Accept Broadcast Frames
#define  REF_CTL_AM						0x00000200UL	// Accept Multicast Frames
#define  REF_CTL_AU						0x00000100UL	// Accept All Unicast Frames
#define  REF_CTL_VS						0x00000080UL	// Enable VLAN Tag Stripping
#define  REF_CTL_UF						0x00000040UL	// Untag Frame Filtering
#define  REF_CTL_VF						0x00000020UL	// Enable VLAN Filtering
#define  REF_CTL_SPF					0x00000010UL	// Enable Source Address Pefect Fitlering
#define  REF_CTL_MHF					0x00000008UL	// Enable Multicast DA Only Hash Filtering
#define  REF_CTL_DHF					0x00000004UL	// Enable Unicast DA Hash Filtering
#define  REF_CTL_DPF					0x00000002UL	// Enable DA Pefect Filtering
#define  REF_CTL_RST_RF					0x00000001UL	// Reset Receive Filtering Engine (NEVER SET IT)
/************** VLAN Type Register (VLAN_TYPE) ***************/
#define VLAN_TYPE_MASK					0x0000FFFF
/************** EEPROM Command Register (E2P_CMD) ***************/
#define	E2P_CMD_BUSY					0x80000000UL
#define	E2P_CMD_MASK					0x70000000UL
/*EPC Command definitions*/
#define	E2P_CMD_READ					0x00000000UL
#define	E2P_CMD_EWDS					0x10000000UL
#define	E2P_CMD_EWEN					0x20000000UL
#define	E2P_CMD_WRITE					0x30000000UL
#define	E2P_CMD_WRAL					0x40000000UL
#define	E2P_CMD_ERASE					0x50000000UL
#define	E2P_CMD_ERAL					0x60000000UL
#define	E2P_CMD_RELOAD					0x70000000UL
/**********************/
#define	E2P_CMD_TIMEOUT					0x00000400UL
#define	E2P_CMD_LOADED					0x00000200UL
#define	E2P_CMD_ADDR					0x000001FFUL
#define MAX_EEPROM_SIZE			512
/************** EEPROM Command Register (E2P_DATA) ***************/
#define	E2P_DATA_MASK					0x000000FFUL
/************** FIFO Controller RX FIFO Control Register (FCT_RX_CTL) ***************/
#define FCT_RX_CTL_EN					0x80000000UL
#define FCT_RX_CTL_RST					0x40000000UL
#define FCT_RX_CTL_SBF					0x02000000UL
#define FCT_RX_CTL_OVERFLOW				0x01000000UL
#define FCT_RX_CTL_FRM_DROP				0x00800000UL
#define FCT_RX_CTL_RX_NOT_EMPTY			0x00400000UL
#define FCT_RX_CTL_RX_EMPTY				0x00200000UL
#define FCT_RX_CTL_RX_DISABLED			0x00100000UL
#define FCT_RX_CTL_RXUSED				0x0000FFFFUL
/************** FIFO Controller TX FIFO Control Register (FCT_TX_CTL) ***************/
#define FCT_TX_CTL_EN					0x80000000UL
#define FCT_TX_CTL_RST					0x40000000UL
#define FCT_TX_CTL_TX_NOT_EMPTY			0x00400000UL
#define FCT_TX_CTL_TX_EMPTY				0x00200000UL
#define FCT_TX_CTL_TX_DISABLED			0x00100000UL
#define FCT_TX_CTL_TXUSED				0x0000FFFFUL
/************** FCT RX FIFO End Register (FCT_RX_FIFO_END) ***************/
#define FCT_RX_FIFO_END_MASK			0x0000007FUL
/************** FCT TX FIFO End Register (FCT_TX_FIFO_END) ***************/
#define FCT_TX_FIFO_END_MASK			0x0000003FUL
/************** FCT Flow Control Threshold Register (FCT_FLOW) ***************/
#define FCT_FLOW_THRESHOLD_OFF			0x00007F00UL
#define FCT_FLOW_THRESHOLD_OFF_SHIFT	8
#define FCT_FLOW_THRESHOLD_ON			0x0000007FUL
/************** HS Descriptor Attributes Register (HS_ATTR) ***************/
#define HS_ATTR_POLL_INT				0x00FF0000UL
#define HS_ATTR_DEV_DESC_SIZE			0x0000FF00UL
#define HS_ATTR_CFG_DESC_SIZE			0x0000000FUL
/************** FS Descriptor Attributes Register (FS_ATTR) ***************/
#define FS_ATTR_POLL_INT				0x00FF0000UL
#define FS_ATTR_DEV_DESC_SIZE			0x0000FF00UL
#define FS_ATTR_CFG_DESC_SIZE			0x0000000FUL
/************** String Descriptor Attributes Register 0 (STRING_ATTR0) *********/
#define STRING_ATTR0_CFG_SIZE			0xFF000000UL
#define STRING_ATTR0_SER_SIZE			0x00FF0000UL
#define STRING_ATTR0_PRO_SIZE			0x0000FF00UL
#define STRING_ATTR0_MAN_SIZE			0x000000FFUL
/************** String Descriptor Attributes Register 1 (STRING_ATTR1) *********/
#define STRING_ATTR1_INTF_SIZE			0x0000000FUL
/************** Flag Attributes Register (FLAG_ATTR) *********/
#define FLAG_ATTR_PORT_SWAP				0x00800000UL
#define FLAG_ATTR_RMT_WKP				0x00020000UL
#define FLAG_ATTR_PWR_SEL				0x00010000UL
#define FLAG_ATTR_PME_FLAGS				0x000000FFUL
/********************** MAC Control Register (MAC_CR) ***************************/
#define MAC_CR_ADP						0x00002000UL
#define MAC_CR_ADD						0x00001000UL
#define MAC_CR_ASD						0x00000800UL
#define MAC_CR_INT_LOOP					0x00000400UL
#define MAC_CR_BOLMT					0x000000C0UL
#define MAC_CR_FDPX						0x00000008UL
#define MAC_CR_CFG						0x00000006UL
/*MAC_CR_CFG definitions*/
#define MAC_CR_CFG_10					0x00000000UL
#define MAC_CR_CFG_100					0x00000002UL
#define MAC_CR_CFG_1000					0x00000004UL
/*************************/
#define MAC_CR_RST						0x00000001UL
/********************** MAC Receive Register (MAC_RX) ***************************/
#define MAC_RX_MAX_SIZE					0x3FFF0000UL
#define MAC_RX_MAX_SIZE_SHIFT			16
#define MAC_RX_FCS_STRIP				0x00000010UL
#define MAC_RX_FSE						0x00000004UL
#define MAC_RX_RXD						0x00000002UL
#define MAC_RX_RXEN						0x00000001UL
/********************** MAC Transmit Register (MAC_TX) ***************************/
#define MAC_TX_BFCS						0x00000004UL
#define MAC_TX_TXD						0x00000002UL
#define MAC_TX_TXEN						0x00000001UL
/********************** Flow Control Register (FLOW) ***************************/
#define FLOW_FORCE_FC					0x80000000UL
#define FLOW_TX_FCEN					0x40000000UL
#define FLOW_RX_FCEN					0x20000000UL
#define FLOW_FPF						0x10000000UL
#define FLOW_PAUSE_TIME					0x0000FFFFUL
/********************** Random Number Seed Value Register (RAND_SEED) ***************************/
#define RAND_SEED_MASK					0x0000FFFFUL
/********************** Error Status Register (ERR_STS) ***************************/
#define ERR_STS_FCS_ERR					0x00000100UL
#define ERR_STS_LFRM_ERR				0x00000080UL
#define ERR_STS_RUNT_ERR				0x00000040UL
#define ERR_STS_COLLISION_ERR			0x00000010UL
#define ERR_STS_ALIGN_ERR				0x00000008UL
#define ERR_STS_URUN_ERR				0x00000004UL
/********************** MAC Receive Address High Register (RX_ADDRH) ***************************/
#define RX_ADDRH_MASK					0x0000FFFFUL
/********************** MAC Receive Address Low Register (RX_ADDRL) ***************************/
/********************** Wakeup Control and Status Register (WUCSR) ***************************/
#define WUCSR_PFDA_FR					0x00000080UL
#define WUCSR_WUFR					    0x00000040UL
#define WUCSR_MPR						0x00000020UL
#define WUCSR_BCAST_FR				    0x00000010UL
#define WUCSR_PFDA_EN				    0x00000008UL
#define WUCSR_WUEN						0x00000004UL
#define WUCSR_MPEN						0x00000002UL
#define WUCSR_BCST_EN					0x00000001UL
/********************** Wakeup Filter x Configuration Register (WUF_CFGX) ***************************/
#define WUF_CFGX_EN						0x80000000UL
#define WUF_CFGX_ATYPE					0x03000000UL
/*CFGX_ATYPE definitions*/
#define WUF_CFGX_ATYPE_UNICAST			0x00000000UL
#define WUF_CFGX_ATYPE_MULTICAST		0x02000000UL
#define WUF_CFGX_ATYPE_ALL				0x03000000UL

#define WUF_CFGX_PATTERN_OFFSET			0x007F0000UL
#define WUF_CFGX_PATTERN_OFFSET_SHIFT	16
#define WUF_CFGX_CRC16					0x0000FFFFUL
#define WUF_NUM							8
/********************** Wakeup Filter x Byte Mask  Register (WUF_MASKX) ***************************/
#define WUF_MASKX_AVALID				0x80000000UL
#define WUF_MASKX_ATYPE					0x40000000UL
/********************** Wakeup Filter x Byte Mask  Register (ADDR_FILTX) ***************************/
#define ADDR_FILTX_FB_VALID				0x80000000UL
#define ADDR_FILTX_FB_TYPE				0x40000000UL
#define ADDR_FILTX_FB_ADDRHI			0x0000FFFFUL
#define ADDR_FILTX_SB_ADDRLO			0xFFFFFFFFUL
/********************** Wakeup Control and Status Register (WUCSR2) ***************************/
#define WUCSR2_NS_RCD					0x00000040UL
#define WUCSR2_ARP_RCD					0x00000020UL
#define WUCSR2_TCPSYN_RCD				0x00000010UL
#define WUCSR2_NS_OFFLOAD				0x00000004UL
#define WUCSR2_ARP_OFFLOAD				0x00000002UL
#define WUCSR2_TCPSYN_OFFLOAD			0x00000001UL
/********************** Device Ipv4 Address Register (IPV4_ADDR) ***************************/

/******************************************************************************************/
/*******************************	PHY Registers	***************************************/
/******************************************************************************************/
enum _PHY_REGS{
	PHY_BCR,
	PHY_BSR,
	PHY_ID1,
	PHY_ID2,
	PHY_AN_ADV,
	PHY_AN_LPA,
	PHY_AN_EXP,
	PHY_AN_NP_TX,
	PHY_AN_NP_RX,
	PHY_MSCR,
	PHY_MSSR,
	PHY_EXT_ST				= 15,
	PHY_LINK_CONTROL,
	PHY_MODE_CSR,
	PHY_SPECIAL_MODE,
	PHY_EXT_CSR,
	PHY_AR_ADDR,
	PHY_AR_DATA,
	PHY_CSI					= 27,
	PHY_CHANNEL_QUALITY,
	PHY_INT_SRC,
	PHY_INT_MASK,
	PHY_SCSR,
	NO_OF_PHY_REGS
};
/************************** Basic Control Register (PHY_BCR)***********************************/
#define PHY_BCR_RESET					0x8000U
#define PHY_BCR_LOOPBACK				0x4000U
#define PHY_BCR_SPEED_SELECT			0x2000U
#define PHY_BCR_AUTO_NEG_ENABLE			0x1000U
#define PHY_BCR_POWER_DOWN              0x0800U
#define PHY_BCR_ISOLATE                	0x0400U
#define PHY_BCR_RESTART_AUTO_NEG	    0x0200U
#define PHY_BCR_DUPLEX_MODE				0x0100U
#define PHY_BCR_COLLISION_TEST			0x0080U
#define PHY_BCR_SPEED_SEL				0x0040U
/************************** Basic Status Register (PHY_BSR)***********************************/
#define PHY_BSR_100_T4					0x8000U
#define PHY_BSR_100_TXFULL				0x4000U
#define PHY_BSR_100_TXHALF				0x2000U
#define PHY_BSR_10_TXFUL				0x1000U
#define PHY_BSR_10_HALF					0x0800U
#define PHY_BSR_100T2_FULL				0x0400U
#define PHY_BSR_100T2_HALF				0x0200U
#define PHY_BSR_EXTEND_ST				0x0100U
#define PHY_BSR_MFPS					0x0040U
#define PHY_BSR_AUTO_NEG_COMP	    	0x0020U
#define PHY_BSR_REMOTE_FAULT			0x0010U
#define PHY_BSR_AN_ABILITY				0x0008U
#define PHY_BSR_LINK_STATUS				0x0004U
#define PHY_BSR_JABBER					0x0002U
#define PHY_BSR_EXTEND_CAPA				0x0001U
/************************** PHY Identifier 2 Register (PHY_ID2)***********************************/
#define PHY_ID2_ID						0xFC00U
#define PHY_ID2_ID_SHIFT				10
#define PHY_ID2_MODEL					0x03F0U
#define PHY_ID2_MODEL_SHIFT				4
#define PHY_ID2_REV						0x000FU
/************************** Auto Negotiation Advertisement Register (PHY_AN_ADV)***********************************/
#define PHY_AN_ADV_NPAGE				0x8000U
#define PHY_AN_ADV_RF					0x2000U
#define PHY_AN_ADV_ASYMP				0x0800U
#define PHY_AN_ADV_PAUSE				0x0400U
#define PHY_AN_ADV_100FDX				0x0100U
#define PHY_AN_ADV_100HDX				0x0080U
#define PHY_AN_ADV_10FDX				0x0040U
#define PHY_AN_ADV_10HDX				0x0020U
#define PHY_AN_ADV_ALL_CAPS				(PHY_AN_ADV_ASYMP | PHY_AN_ADV_PAUSE | PHY_AN_ADV_100FDX | PHY_AN_ADV_100HDX | PHY_AN_ADV_10FDX | PHY_AN_ADV_10HDX)
#define PHY_AN_ADV_SF					0x001FU
/************************** Auto Negotiation Link Partner Register (PHY_AN_LPA)***********************************/
#define PHY_AN_LPA_NPAGE				0x8000U
#define PHY_AN_LPA_ACK					0x4000U
#define PHY_AN_LPA_RF					0x2000U
#define PHY_AN_LPA_ASYMP				0x0800U
#define PHY_AN_LPA_PAUSE				0x0400U
#define PHY_AN_LPA_T4					0x0200U
#define PHY_AN_LPA_100FDX				0x0100U
#define PHY_AN_LPA_100HDX				0x0080U
#define PHY_AN_LPA_10FDX				0x0040U
#define PHY_AN_LPA_10HDX				0x0020U
#define PHY_AN_LPA_SF					0x001FU
/************************** Auto Negotiation Expansion Register (PHY_AN_EXP)***********************************/
#define PHY_AN_EXP_PAR_DETECT_FAULT		0x0010U    /* Parallel Detection Fault */
#define PHY_AN_LP_NEXT_PAGE_CAPS  		0x0008U    /* LP is NextPage capable */
#define PHY_AN_NEXT_PAGE_CAPS			0x0004U    /* We're NextPage capable (always 1 for this PHY) */
#define PHY_AN_PAGE_RXD         	  	0x0002U    /* Recv'd a new Page from LP  */
#define PHY_AN_LP_ANCAPS		       	0x0001U    /* LP has Auto Neg Capability */
/************************** Auto Negotiation Next Page TX Register (PHY_AN_NP_TX)***********************************/
#define PHY_AN_NPTX_NEXT_PAGE          	0x8000U    /* 1 = no next page ability, 0 = next page capabile */
#define PHY_AN_NPTX_MSG_PAGE            0x2000U    /* formatted(1)/unformatted(0) pg */
#define PHY_AN_NPTX_ACKNOWLDGE2         0x1000U    /* 1 = will comply with msg, 0 = cannot comply with msg */
#define PHY_AN_NPTX_TOGGLE				0x0800U    /* Toggles between exchanges of different NP */
#define PHY_AN_NPTX_MSG_CODE_FIELD		0x07FFU    /* NP msg code or unformatted data */
/************************** Auto Negotiation Next Page RX Register (PHY_AN_NP_RX)***********************************/
#define PHY_AN_NPRX_NEXT_PAGE          	0x8000U    /* 1 = no next page ability, 0 = next page capabile */
#define PHY_AN_NPRX_MSG_PAGE            0x2000U    /* formatted(1)/unformatted(0) pg */
#define PHY_AN_NPRX_ACKNOWLDGE2         0x1000U    /* 1 = will comply with msg, 0 = cannot comply with msg */
#define PHY_AN_NPRX_TOGGLE				0x0800U    /* Toggles between exchanges of different NP */
#define PHY_AN_NPRX_MSG_CODE_FIELD		0x07FFU    /* NP msg code or unformatted data */
/************************** Master/Slave Control Register (PHY_MSCR)***********************************/
#define PHY_MSCR_TEST_MODE				0xE000U
#define PHY_MSCR_TEST_MODE_4			0x8000U    	/* Transmitter Distortion test */
#define PHY_MSCR_TEST_MODE_3			0x6000U    	/* Slave Transmit Jitter test */
#define PHY_MSCR_TEST_MODE_2			0x4000U    	/* Master Transmit Jitter test */
#define PHY_MSCR_TEST_MODE_1			0x2000U		/* Transmit Waveform test */
#define PHY_MSCR_TEST_MODE_NORMAL		0x0000U		/* Normal Mode */
#define PHY_MSCR_MANUAL_EN				0x1000U
#define PHY_MSCR_MANUAL_VALUE			0x0800U
#define PHY_MSCR_PORT_TYPE				0x0400U
#define PHY_MSCR_1000FDX				0x0200U
#define PHY_MSCR_1000HDX				0x0100U
/************************** Master/Slave Status Register (PHY_MSSR)***********************************/
#define PHY_MSSR_CONFIG_FAULT			0x8000U
#define PHY_MSSR_CONFIG_RESO			0x4000U
#define PHY_MSSR_LOCAL_ST				0x2000U
#define PHY_MSSR_REMOTE_ST				0x1000U
#define	PHY_MSSR_LP_1000FDX				0x0800U
#define	PHY_MSSR_LP_1000HDX				0x0400U
#define PHY_MSSR_IDLE_ERR_CNT			0x000FU
/************************** Extended Status Register (PHY_EXT_ST)***********************************/
#define PHY_EXT_ST_1000X_FDX			0x8000U
#define PHY_EXT_ST_1000X_HDX			0x4000U
#define PHY_EXT_ST_1000T_FDX			0x2000U
#define PHY_EXT_ST_1000T_HDX			0x1000U
/************************** Link Control Register (PHY_LINK_CONTROL)***********************************/
#define PHY_LINK_CONTROL_SOC			0x0300U /*Speed Optimize control*/
#define PHY_LINK_CONTROL_SOC_7			0x0000U /*7 attempts*/
#define PHY_LINK_CONTROL_SOC_5			0x0100U /*5 attempts*/
#define PHY_LINK_CONTROL_SOC_4			0x0200U /*4 attempts*/
#define PHY_LINK_CONTROL_SOC_3			0x0300U /*3 attempts*/
#define PHY_LINK_BIST_START				0x00C0U /*Start a build test*/
/*BIST_START definitions*/
#define PHY_LINK_BIST_START_NORMAL		0x0000U
#define PHY_LINK_BIST_START_DIGITAL		0x0040U
#define PHY_LINK_BREAK_THRES			0x0030U
/*BREAK_THRES definitions*/
#define PHY_LINK_BREAK_THRES_8			0x0000U
#define PHY_LINK_BREAK_THRES_9			0x0010U
#define PHY_LINK_BREAK_THRES_10			0x0020U
#define PHY_LINK_BREAK_THRES_11			0x0030U
#define PHY_LINK_BREAK_EN				0x0008U
#define PHY_LINK_POWER_OP_DIS			0x0004U
#define PHY_LINK_SPEED_1000_EN			0x0002U
#define PHY_LINK_LRST					0x0001U
/************************** 10/100 Mode Control/Status Register (PHY_MODE_CSR)***********************************/
#define PHY_MODE_CSR_EDSHORT			0x8000U
#define PHY_MODE_CSR_FASTRIP			0x4000U
#define PHY_MODE_CSR_EDPWRDOWN			0x2000U
#define PHY_MODE_CSR_ED_PWM				0x1000U
#define PHY_MODE_CSR_MD_PREBP			0x0400U
#define PHY_MODE_CSR_FAR_LOOPBACK		0x0200U
#define PHY_MODE_CSR_AN_TEST			0x0100U
#define PHY_MODE_CSR_SPEED_OPTIMIZE_EN	0x0080U
#define PHY_MODE_CSR_AN_NP_EN			0x0040U
#define PHY_MODE_CSR_AUTO_MDIX_DIS		0x0020U
#define PHY_MODE_CSR_NP_DIS				0x0010U
#define PHY_MODE_CSR_PHYADBP			0x0008U
#define PHY_MODE_CSR_FORCE_GOODLINK		0x0004U
#define PHY_MODE_CSR_ENERGY_ON			0x0002U
#define PHY_MODE_CSR_SEMI_XOVER_EN		0x0001U
/************************** 10/100 Special Modes Register (PHY_SPECIAL_MODE)***********************************/
#define PHY_SPECIAL_MODE_EN_RXDV		0x8000U
#define PHY_SPECIAL_MODE_10T_LP_DIS		0x4000U
#define PHY_SPECIAL_MODE_1000_CEC_SR	0x0080U
#define PHY_SPECIAL_MODE_MCLK_OUT		0x0040U
#define PHY_SPECIAL_MODE_ADDR			0x001FU
/************************** Extended Mode Control/Status Register (PHY_EXT_CSR)***********************************/
#define PHY_EXT_CSR_MOD					0xF800U
#define PHY_EXT_CSR_TXFIFO_DEPTH		0x0600U
#define PHY_EXT_CSR_MAC_INTF			0x01C0U
/*PHY_EXT_CSR_MAC_INTF definitions*/
#define PHY_EXT_CSR_MAC_INTF_GMIINMII	0x0000U
#define PHY_EXT_CSR_MAC_INTF_RDMII		0x00C0U
#define PHY_EXT_CSR_LED_MODE			0x0030U
/*LED mode definitions*/
#define PHY_EXT_CSR_LED_MODE_2			0x0010U
#define PHY_EXT_CSR_LED_MODE_3			0X0020U
#define PHY_EXT_CSR_LED_MODE_4			0X0030U
/*************************/
#define PHY_EXT_CSR_CLK125DIS			0x0008U
#define PHY_EXT_CSR_MDIX_AB			0x0004U
#define PHY_EXT_CSR_MDIX_CD			0x0002U
#define PHY_EXT_CSR_CP_DETECT			0x0001U
/************************** Advanced Address Port Register (PHY_AR_ADDR)***********************************/
#define PHY_AR_ADDR_READ				0x8000U
#define PHY_AR_ADDR_TEST				0x7E00U
#define PHY_AR_ADDR_TEST1				0x0180U
#define PHY_AR_ADDR_ADDR				0x007FU
/************************** Advanced Read Data Port Register (PHY_AR_DATA)***********************************/
/************************** Control/Status Indication Register (PHY_CSI)***********************************/
#define PHY_CSI_SQEOFF					0x0800U
#define PHY_CSI_XPOL					0x0010U
/************************** Channel Quality Status Register (PHY_CHANNEL_QUALITY)***********************************/
#define PHY_CHANNEL_QUALITY_CBLN		0xF000U
/************************** Interrupt Source Flags Register (PHY_INT_SRC)***********************************/
#define PHY_INT_SRC_ENERGY_ON			0x0080U
#define PHY_INT_SRC_ANEG_COMP			0x0040U
#define PHY_INT_SRC_REMOTE_FAULT		0x0020U
#define PHY_INT_SRC_LINK_DOWN			0x0010U
/************************** Interrupt Mask Register (PHY_INT_MASK)***********************************/
#define PHY_INT_MASK_ALL				0x07FFU
#define PHY_INT_MASK_ENERGY_ON			0x0080U
#define PHY_INT_MASK_ANEG_COMP			0x0040U
#define PHY_INT_MASK_REMOTE_FAULT		0x0020U
#define PHY_INT_MASK_LINK_DOWN			0x0010U
/************************** PHY Special Control/Status Register (PHY_SCSR)***********************************/
#define PHY_SCSR_AN_DONE				0x0100U
#define PHY_SCSR_SPD					0x001CU
#define PHY_SCSR_SPD_10HALF				0x0004U
#define PHY_SCSR_SPD_10FULL				0x0014U
#define PHY_SCSR_SPD_100HALF			0x0008U
#define PHY_SCSR_SPD_100FULL			0x0018U
#define PHY_SCSR_SCRMB_DIS				0x0001U

/******************************************************************************************/
/*******************************	Advanced PHY Registers	*******************************/
/******************************************************************************************/
enum _ADVANCED_PHY_REGS{
	AP_US1,
	AP_US2,
	AP_RX_EFP_HIGH,
	AP_RX_EFP_MID,
	AP_RX_EFP_LOW,
	AP_CRCERR_HIGH,
	AP_CRCERR_MID,
	AP_CRCERR_LOW,
	AP_RXERR_DATA,
	AP_RXERR_IDLE,
	AP_TX_HIGH,
	AP_TX_MID,
	AP_TX_LOW
};

/*************************************** User Status 1 (AP_US1)***********************************/
#define AP_US1_PLLREADY				0x8000U
#define AP_US1_CLKREF_SEL			0x0100U
/************************************** User Status 2 (AP_US2)***********************************/
#define AP_US2_XOVER_AB				0x8000U
#define AP_US2_XOVER_CD				0x4000U
#define AP_US2_SPD_OPTIMIZE			0x2000U

/******************************************************************************************/
/*******************************	End  of register definitions*******************************/
/******************************************************************************************/
typedef unsigned char BYTE;

#define BCAST_LEN   		6
#define MCAST_LEN   		3
#define ARP_LEN     		2
#define STATUS_WORD_LEN		4
#define TX_TOTAL_CONTROL_WORD_LEN		8
#define RX_TOTAL_CONTROL_WORD_LEN		8
#define RXW_PADDING			2
#define MAX_EEPROM_SIZE				512
/***********************Tx/Rx Command/Status definitions*************************************/
// Tx MAT Cmds
#define		TX_CMD_A_LSO				0x08000000UL	//Large Send Offload Enable
#define		TX_CMD_A_IPE				0x04000000UL	//IP Checksum Offload Enable
#define		TX_CMD_A_TPE				0x02000000UL	//TCP/UDP Checksum Offload Enable
#define		TX_CMD_A_IVTG				0x01000000UL	//Insert VLAN Tag
#define		TX_CMD_A_RVTG				0x00800000UL	//Replace VLAN Tag
#define		TX_CMD_A_FCS				0x00400000UL	//Insert FCS and Pad
#define     TX_CMD_A_LEN				0x000FFFFFUL	//Frame Length

#define     TX_CMD_B_MSS				0x3FFF0000UL	//Maximum Segment Size
#define     TX_CMD_B_MSS_SHIFT			16
#define		TX_MSS_MIN					8
#define     TX_CMD_B_VTAG				0x0000FFFFUL	//VLAN Tag

// Rx sts Wd
#define	RX_CMD_A_ICE		0x80000000UL	// IP Checksum Error
#define	RX_CMD_A_TCE		0x40000000UL	// TCP/UDP Checksum Error
#define	RX_CMD_A_IPV		0x20000000UL	// IP Version
#define	RX_CMD_A_PID		0x18000000UL	// Protocol ID
#define	RX_CMD_A_PID_NIP	0x00000000UL	// Protocol ID: None IP
#define	RX_CMD_A_PID_TCP	0x08000000UL	// Protocol ID: TCP and IP
#define	RX_CMD_A_PID_UDP	0x10000000UL	// Protocol ID: UDP and IP
#define	RX_CMD_A_PID_PP		0x18000000UL	// Protocol ID: IP
#define	RX_CMD_A_PFF		0x04000000UL	// Perfect Filter Passed
#define	RX_CMD_A_BAM		0x02000000UL	// Broadcast Frame
#define	RX_CMD_A_MAM		0x01000000UL	// Multicase Frame
#define	RX_CMD_A_FVTG		0x00800000UL	// Frame is VLAN tagged
#define	RX_CMD_A_RED		0x00400000UL	// Receive Error Detected
#define	RX_CMD_A_RWT		0x00200000UL	// Receive Watchdog Timer Expired
#define	RX_CMD_A_RUNT		0x00100000UL	// Runt Frame
#define	RX_CMD_A_LONG		0x00080000UL	// Frame Too Long
#define	RX_CMD_A_RXE		0x00040000UL	// Rx Error
#define	RX_CMD_A_DRB		0x00020000UL	// Alignment Error
#define	RX_CMD_A_FCS		0x00010000UL	// FCS Error
#define	RX_CMD_A_UAM		0x00008000UL	// Unicast Frame
#define	RX_CMD_A_LCSM		0x00004000UL	// Ignore TCP/UDP Checksum
#define	RX_CMD_A_LEN		0x00003FFFUL	// Frame Length

#define	RX_CMD_B_CSUM		0xFFFF0000UL	// Raw L3 Checksum
#define	RX_CMD_B_CSUM_SHIFT	16
#define	RX_CMD_B_VTAG		0x0000FFFFUL	// VATG tag

/*************************End of Tx/Rx Command/Status definitions*************************/

/*************************** Vendor Requests ***********************************/
#define	USB_VENDOR_REQUEST_WRITE_REGISTER	0xA0
#define	USB_VENDOR_REQUEST_READ_REGISTER	0xA1
#define	USB_VENDOR_REQUEST_GET_STATS		0xA2


/************* Interrupt Endpoint status word bitfields ****************************/
#define	INT_ENP_RFHF_		(0x00040000UL)
#define	INT_ENP_TX_STOP_	ETH_INT_STS_TX_STOP_
#define	INT_ENP_RX_STOP_	ETH_INT_STS_RX_STOP_
#define	INT_ENP_PHY_INT_	ETH_INT_STS_PHY_INT_
#define	INT_ENP_TXE_		ETH_INT_STS_TXE_
#define	INT_ENP_TDFU_		ETH_INT_STS_TDFU_
#define	INT_ENP_TDFO_		ETH_INT_STS_TDFO_
#define	INT_ENP_RXDF_		ETH_INT_STS_RXDF_
#define	INT_ENP_GPIOS_		ETH_INT_STS_GPIOS_

#define HIBYTE(word)  		((BYTE)(((u16)(word))>>8))
#define LOBYTE(word)  		((BYTE)(((u16)(word))&0x00FFU))
#define HIWORD(dWord) 		((u16)(((u32)(dWord))>>16))
#define LOWORD(dWord) 		((u16)(((u32)(dWord))&0x0000FFFFUL))

/************************ Request Link Status *********************************************/
#define LINK_INIT               	0xFFFFUL
#define LINK_OFF                	0x0000UL
#define LINK_SPEED_10HD         	0x0001UL
#define LINK_SPEED_10FD         	0x0002UL
#define LINK_SPEED_100HD        	0x0004UL
#define LINK_SPEED_100FD        	0x0008UL
#define LINK_SPEED_MASTER_1000FD    0x0010UL
#define LINK_SPEED_SLAVE_1000FD     0x0020UL
#define LINK_SYMMETRIC_PAUSE    	0x0040UL
#define LINK_ASYMMETRIC_PAUSE	   	0x0080UL
#define LINK_AUTO_NEGOTIATE     	0x0100UL
#define LINK_SPEED_ALL				(LINK_SPEED_10HD | LINK_SPEED_10FD | LINK_SPEED_100HD | LINK_SPEED_100FD | LINK_SPEED_MASTER_1000FD | LINK_SPEED_SLAVE_1000FD)


/************************ Interrupt Endpoint Status *********************************************/
#define INT_END_RDFO                (0x00400000UL)
#define INT_END_TXE                 (0x00200000UL)
#define INT_END_TX_DIS              (0x00080000UL)
#define INT_END_RX_DIS              (0x00040000UL)
#define INT_END_PHY_INT             (0x00020000UL)
#define INT_END_MAC_ERR             (0x00008000UL)
#define INT_END_TDFU                (0x00004000UL)
#define INT_END_TDFO                (0x00002000UL)
#define INT_END_RXFIFO_HAS_FRAME    (0x00001000UL)
#define INT_END_GPIO_INT            (0x00000FFFUL)

#define HS_USB_PKT_SIZE				512
#define FS_USB_PKT_SIZE				64

#define DEFAULT_HS_BURST_CAP_SIZE	(32 *1024)
#define DEFAULT_FS_BURST_CAP_SIZE	(6 *1024 + 33 * FS_USB_PKT_SIZE)
#define DEFAULT_BULK_IN_DELAY		(0x00002000L)

/***************************************** Ethernet default ******************************/
#define ETH_ADDR_LEN				6
#define ETH_HEADER_SIZE				14
#define ETH_MAX_DATA_SIZE			1500
#define ETH_MAX_PACKET_SIZE			(ETH_MAX_DATA_SIZE + ETH_HEADER_SIZE)
#define MAX_SINGLE_PACKET_SIZE		2048
#define LAN7500_EEPROM_MAGIC		0x7500UL
#define EEPROM_MAC_OFFSET			0x01

#define MAX_RX_FIFO_SIZE			(20 * 1024UL) // 20  KB
#define MAX_TX_FIFO_SIZE			(12 * 1024UL) // 12  KB

#define FC_PAUSE_TIME				0xFFFF

#define WUFF_NUM				8
/****************************************/
#define TX_QUEUE_SRC_LINK			0x01
#define TX_QUEUE_SRC_WOL			0x04
/*************************** Miscellaneous ******************************************/
#define DUMMY_VALUE					0xFFFFFFFFUL

/* Definitions for Auto MDIX */
#define AMDIX_DISABLE_STRAIGHT          0x00000000
#define AMDIX_DISABLE_CROSSOVER         0x00000001
#define AMDIX_ENABLE                    0x00000002

/*****************  Structures ***************************/
/*******************************************************/

// Status block structures

enum{
	LEN_20,
	LEN_32
};

typedef struct _FRAME_CNT_{
	u32 cnt;
	BOOLEAN len;	//LEN_20 or LEN_32
}FRAME_CNT;

static struct {
	const char str[ETH_GSTRING_LEN];
} ethtool_stats_keys[] = {
	{ "rx_packets"},
	{ "tx_packets"},
	{ "rx_bytes"},
	{ "tx_bytes"},
	{ "rx_errors"},
	{ "tx_errors"},
	{ "tx_dropped"},
	{ "rx_no_buffer_count"},
	{ "rx_length_errors"},
	{ "rx_over_errors"},
	{ "rx_crc_errors"},
	{ "rx_frame_errors"},
	{ "rx_Fragment_Errors"},
	{ "rx_Jabber_Errors"},
	{ "rx_RuntFrame_Errors"},
	{ "rx_FrameTooLong_Error"},
	{ "rx_Dropped_Frames"},
	{ "rx_Unicast_Frames"},
	{ "rx_Broadcast_Frames"},
	{ "rx_Multicast_Frames"},
	{ "rx_Pause_Frames"},
	{ "tx_Fcs_Errors"},
	{ "tx_ExcessDeferral_Errors"},
	{ "tx_Carrier_Errors"},
	{ "tx_Single_Collisions"},
	{ "tx_Multiple_Collisions"},
	{ "tx_Excess_Collision"},
	{ "tx_Late_Collision"},
	{ "tx_Unicast_Frames"},
	{ "tx_Broadcast_Frames"},
	{ "tx_Multicast_Frames"},
	{ "tx_Pause_Frames"},
};


typedef struct _ADAPTER_DATA {
    u32 dwIdRev;
    u32 dwPhyAddress;
    u32 dwPhyId;
    u32 dwFpgaRev;
    u32 macAddrHi16;	//Mac address high 16 bits
    u32 MmacAddrLo32;   //Mac address Low 32 bits

    BOOLEAN UseTxCsum;
    BOOLEAN UseRxCsum;
    BOOLEAN vlanAcceleration;

    BOOLEAN LanInitialized;
    BYTE bPhyModel;
    BYTE bPhyRev;

    u32 linkRequest;	//Link requested when loading module
    u32 LinkAdvertisedCapabilites;	//Advertised capabilities
    u32 actualLinkMode;	//Negotiated or forced link mode
    BOOLEAN bGeneratePause;
    BOOLEAN bRespondToPause;
    u32 fctFlowReg;
    BOOLEAN applyPhyRxClkWorkaround;

    struct semaphore phy_mutex;
    struct semaphore internal_ram_mutex; //Mutex for internal ram operation
    struct semaphore RxFilterLock;

    spinlock_t TxQueueLock;
    BOOLEAN TxInitialized;
    u32 dwTxQueueDisableMask;
    BOOLEAN TxQueueDisabled;

    u32 MulticastHashTable[VHF_HASH_LEN];	//Hash table;
    u32 WolWakeupOpts;
    u32 wakeupOptsBackup;
    u32 dwWUFF[20];

    u32 LinkActLedCfg;

    u32 dynamicSuspendPHYEvent; //Store PHY interrupt source for dynamic suspend
    u32 systemSuspendPHYEvent; //Store PHY interrupt source for system suspend

    u16 eepromSize; 	//EEPROM size

    u32 rxFifoSize;
    u32 txFifoSize;

    u32 preFrameCnt[NO_OF_STAS_CNT];
    FRAME_CNT frameCnt[NO_OF_STAS_CNT];
} ADAPTER_DATA, *PADAPTER_DATA;

typedef struct _MAC_ADDR_IN_RAM{
	u32 signature;
	u32 MacAddrL;
	u32 MacAddrH;
	u16 crc;
	u16 crcComplement;
}MAC_ADDR_IN_RAM;

struct smsc7500_int_data {
    u32 IntEndPoint;
} __attribute__ ((packed));

struct USB_CONTEXT{
	struct usb_ctrlrequest req;
	struct completion notify;
};

enum{
	RAMSEL_URX,
	RAMSEL_VHF,
	RAMSEL_LSO_HEAD,
	RAMSEL_FCT_TX,
	RAMSEL_FCT_RX,
	RAMSEL_DESCRIPTOR,
	RAMSEL_WOL
};

enum{
	ASYNC_RW_SUCCESS,
	ASYNC_RW_FAIL,
	ASYNC_RW_TIMEOUT,
};

enum{
	SMSC7500_TIMEOUT 	= -2,
    SMSC7500_FAIL 		= -1,
    SMSC7500_SUCCESS 	= 0
};

/********************************************************/
/********  Debug definitions ***************************/
/*******************************************************/
#define DBG_TRACE       (0x01UL)
#define DBG_WARNING     (0x02UL)
#define DBG_INIT        (0x80000000UL)
#define DBG_CLOSE       (0x40000000UL)
#define DBG_INTR        (0x20000000UL)
#define DBG_PWR         (0x10000000UL)
#define DBG_IOCTL       (0x08000000UL)
#define DBG_LINK        (0x04000000UL)
#define DBG_RX          (0x02000000UL)
#define DBG_TX          (0x01000000UL)
#define DBG_MCAST       (0x00800000UL)
#define DBG_HOST        (0x00400000UL)
#define DBG_LINK_CHANGE (0x00200000UL)

extern u32 debug_mode;
#define USE_WARNING
#define USE_TRACE

#ifdef USE_DEBUG //Can be enabled in makefile
#define USE_DEBUG_MSG
#define USE_ASSERT
#endif //USE_DEBUG


/*******************************************************
* Macro: SMSC_WARNING
* Description:
*    It can be used anywhere you want to display warning information
*    For any release version it should not be left in
*      performance sensitive Tx and Rx code paths.
*    To use this macro define USE_TRACE or
*      USE_WARNING and set bit 1 of debug_mode
*******************************************************/
#ifdef USE_WARNING //Always defined
#define SMSC_WARNING(msg, args...)               \
    if(debug_mode&DBG_WARNING) {                    \
        printk("SMSC_7500_WARNING: ");\
        printk(__FUNCTION__);\
        printk(": " msg "\n",## args);\
    }
#else
#define SMSC_WARNING(msg, args...)
#endif

#ifdef USE_TRACE //Always defined
#define SMSC_TRACE(dbgBit,msg,args...)   \
    if(debug_mode&dbgBit) {                 \
        printk("SMSC_7500: " msg "\n", ## args);    \
    }
#else
#define SMSC_TRACE(dbgBit,msg,args...)
#endif

/*******************************************************
* Macro: SMSC_DEBUG
* Description:
*    It can be used anywhere you want to display information
*    For any release version it should not be left in
*      performance sensitive Tx and Rx code paths.
*******************************************************/

#ifdef USE_DEBUG_MSG
#define SMSC_DEBUG(dbgBit,msg,args...)   \
    if(debug_mode&dbgBit) {                 \
        printk("SMSC_7500: " msg "\n", ## args);    \
    }
#else
#define SMSC_DEBUG(dbgBit,msg,args...)
#endif

/*******************************************************
* Macro: SMSC_ASSERT
* Description:
*    This macro is used to test assumptions made when coding.
*    It can be used anywhere, but is intended only for situations
*      where a failure is fatal.
*    If code execution where allowed to continue it is assumed that
*      only further unrecoverable errors would occur and so this macro
*      includes an infinite loop to prevent further corruption.
*    Assertions are only intended for use during developement to
*      insure consistency of logic through out the driver.
*    A driver should not be released if assertion failures are
*      still occuring.
*******************************************************/
#ifdef USE_ASSERT
    #define SMSC_ASSERT(condition)                                                  \
    if(!(condition)) {                                                              \
        printk("SMSC_7500_ASSERTION_FAILURE: \n");\
        printk("    Condition = " #condition "\n");\
        printk("    Function  = ");printk(__FUNCTION__);printk("\n");\
        printk("    File      = " __FILE__ "\n");\
        printk("    Line      = %d\n",__LINE__);    \
        while(1);\
    }
#else
#define SMSC_ASSERT(condition)
#endif

#endif    // _LAN7500_H



