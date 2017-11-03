============================================================================================ 
Linux v1.04.12 Driver Release Notes for SMSC LAN7500 USB 2.0 to Ethernet 10/100/1000 Adapter 
============================================================================================ 
 
Contents: 
 
1. Platforms and OS versions supported 
2. Device support 
3. Driver structure and file description 
4. Building and installing the driver 
5. Release history 
6. Testing  
7. Known issues 
8. Driver initialization parameter 
9. Debugging 
 
1. Platforms and kernel versions supported 
------------------------------------------ 
 
    - x86, ARM, PPC
    - kernel up to 3.17.x
 
2. Device support 
----------------- 
 
This release supports: 
    -LAN7500 (VID = 0x424, PID = 0x7500) USB to Ethernet 10/100/1000 device 
 
3. Driver structure and file description 
---------------------------------------- 
 
    Driver module: 
 
    smsc7500.ko:        lan7500 driver. 
 
    Files: 
        smsc7500usbnet.h	- usbnet header file 
        smsc7500usbnet.c	- usbnet source file 
        smsclan7500.h		- lan7500 hardware specific header file 
        smsclan7500.c		- lan7500 hardware specific source file 
        ioctl_7500.h		- ioctl header file definitions 
        smsc7500version.h	- Driver version number header file. 
	Makefile		- Makefile 
	readme.txt		- This file
	
 
4. Building and installing the driver 
------------------------------------- 
    The following instructions work fine for a PC build environment, embedded 
    platforms may need slight build modifications, consult your platform documentation. 
    a. Obtain the kernel source tree for the platform in use and build it. 
    b. run 'make' command to smsc7500.ko module 
    c. load the module:  
        insmod smsc7500.ko 
    d. Plug in the lan7500 device into the USB port. 
    e. Configure the ethernet interface eth<n> in the usual way for an ethernet device. 
 
5.  Release history 
-------------------
	V1.04.11 (10/28/2014)
        - Resolved occational gig link issue
        - Compiled kernel up to 3.17.x
	V1.04.10 (8/23/2013)
        - Resolved compile errors for kernel up to 3.10.x
	V1.04.09 (3/14/2012)
        - Resolved compile errors for kernel up to 3.8.x
        - Enabled tx_hold_on_completion by default. 
	V1.04.08 (12/14/2012)
        - Fixes crash when adjusting the size of the skb

	V1.04.07 (07/06/2012)
        - Fixed bug for Link status report to ethtool
	
    V1.04.06 (05/21/2012)
        - Fixed bug for Link status report
        - Fixed bug for MultiPacket TX

    V1.04.03 (01/25/2012)
        - Fixed bug in forcing Gig link mode

    V1.04.02 (03/25/2011)
        - Added interface for the utility to read/write Eeprom from/to a file

    v1.04.01 (02/28/2011)
	- Added bulkin_delay module parameter

    v1.04.00 (02/02/2011)
	- Fixed attach/detach bug.

    v1.03.00 (12/13/2010)
	- Added support for kerrnel up to 2.6.37
	- EDPD bug fix

    v1.02.00 (08/06/2010)
	- Makefile changes

    v1.01.00 (07/27/2010)
	- Phy Rx clock fix

    v1.00.01 (07/21/2010)
	- Ethtool fixes
	- Added set_mac_address() for kernel less than 29

    v1.00.00 (05/18/2010)
	- Merged in to single module smsc7500.ko
	- ethtool bug fixes
	- Automdix crossover/straight cable support
	- Removed support for cmd7500
        - Removed unwanted warnings
        - Fixed statistics counter problem
	- Added more ethtool Options
	- Bug Fixes for suspend/resume issue

    v0.07.00 (04/02/2010)
	- ndo frame-work support
	- fixed compilation issues on 2.6.32

    v0.06.00 (11/25/2009, bug fixes)

    v0.05.00 (6/24/2009, initial release) 

6. Testing  
---------- 
   Developer testing before going to SQA.
   
7. Driver Initialization parameters 
----------------------------------- 
 
There are several possible parameters for each driver module 

 a. smsc7500.ko 
        - operational_mode:  
            0:      low_latency mode. Device uses bulk in pipe continuous  
                    reader for ethernet rx. This is the default mode. 
            1:      low power mode. Device uses the interrupt pipe to  
                    detect ethernet rx, only then submits bulk in's to  
                    read rx data. Once no rx data available stops bulk in 
                    submissions. 
        - rx_queue_size:    controls the size of the rx queue 
                    default is 60 
        - tx_queue_size:    controls the size of the tx queue 
                    default is 60 
        - link_mode:  A bit wise field that specifies any combination of 6 or fewer  
                      link speeds 
            0x01:   10HD 
            0x02:   10FD 
            0x04:   100HD 
            0x08:   100FD 
            0x10:   1000MFD	
            0x20:   1000SFD
            0x40:   Symmetrical Pause 
            0x80:   Asymmetrical Pause 
            0x100:  Auto Negotiate 
        - mac_addr_hi16 & mac_addr_lo32: allow Ethernet Mac address override. 
        - scatter_gather: enabled kernel S/G support (needed for tx_Csum) 
        - tx_Csum:  Default to 0. Set to 1 to enable hw tx checksum support 
        - rx_Csum:  Default to 0. Set to 1 to enable hw rx checksum support 
        - TurboMode: receive mode 
            0:      One receive packet per bulk in transaction 
            1:      Multiple receive packet per bulk in transaction 
        - debug_mode: controls amount of debug verbosity (see debugging below) 
	- auto_mdix: Auto-Mdix setting. Enabled by default.
		0 - Disabled straight cable
		1 - Disabled crossover cable
		2 - Enabled
        - linkdownsuspend: 
            0:  Disabled. 
            1:  Enabled with normal power savings (recommended for maximum compatibility) 
            2:  Enabled with maximum power savings. 
             
        - LinkActLedCfg: if ture, enable separate Link and Activity LEDs.
               
	    - dynamicsuspend: if enabled, suspend device automatically if no 
			traffic for 3 seconds, then waking up device if any Tx/Rx traffic.  
						   
	    - netdetach: if enabled, detach device from USB bus if link is down, then re-
			attaching device to USB bus if link is back.						   

        - tso: tso support. default to 0, Set to 1 to enable. (need to enable tx_Csum and scatter_gather).
