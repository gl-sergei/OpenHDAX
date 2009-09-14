/*
 *  HDAPCIRegisters.h
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 12/6/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */
 
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLocks.h>

#ifndef __HDA_PCI_REGISTERS__
#define __HDA_PCI_REGISTERS__

// Регистры контроллера Azalia.
// Частично взяты из Intel'овской доки, частью выдраны из hdaudio.h драйверов OSS 
#define HDA_GCAP			0x00	/* Global Capabilities  */
#define HDA_VMIN			0x02	/* Minor Version */
#define HDA_VMAJ	        0x03	/* Major Version */
#define HDA_OUTPAY			0x04	/* Output Payload Capability */
#define HDA_INPAY			0x06	/* Input Payload Capability */
#define HDA_GCTL			0x08	/* Global Control */
#define CRST				0x00000001	/* Controller reset */
#define HDA_WAKEEN			0x0C	/* Wake Enable */
#define HDA_STATESTS		0x0E /* State Change Status */
#define HDA_GSTS			0x10	/* Global Status */
#define HDA_OUTSTRMPAY		0x18	/* Output Stream Payload Capability */
#define HDA_INSTRMPAY		0x1A	/* Input Stream Payload Capability */
#define HDA_INTCTL			0x20	/* Interrupt Control */
#define HDA_INTSTS			0x24	/* Interrupt Status */
#define HDA_WALCLK			0x30	/* Wall Clock Counter */
#define HDA_SSYNC			0x38	/* Stream Synchronization */

#define HDA_CORBLBASE		0x40	/* CORB Lower Base Address */
#define HDA_CORBUBASE		0x44	/* CORB Upper Base Address */
#define HDA_CORBWP			0x48	/* CORB Write Pointer */
#define HDA_CORBRP			0x4A	/* CORB Read Pointer */
#define HDA_CORBCTL			0x4C	/* CORB Control */
#define HDA_CORBSTS			0x4D	/* CORB Status */
#define HDA_CORBSIZE		0x4E	/* CORB Size */

#define HDA_RIRBLBASE		0x50	/* RIRB Lower Base Address */
#define HDA_RIRBUBASE		0x54	/* RIRB Upper Base Address */
#define HDA_RIRBWP			0x58	/* RIRB Write Pointer */
#define HDA_RINTCNT			0x5A	/* Response Interrupt Count */
#define HDA_RIRBCTL 		0x5C	/* RIRB Control */
#define HDA_RIRBSTS 		0x5D	/* RIRB Status */
#define HDA_RIRBSIZE 		0x5E	/* RIRB Size */


/* CORB/RIRB control, read/write pointer */
#define RBCTL_DMA_EN		0x02	/* enable DMA */
#define RBCTL_IRQ_EN		0x01	/* enable IRQ */
#define RBRWP_CLR			0x8000	/* read/write pointer clear */

#define MAX_CORB_ENTRIES	256
#define MAX_RIRB_ENTRIES	256

#define RIRB_EX_UNSOL_EV	(1<<4)

/* RIRB int mask: overrun[2], response[0] */
#define RIRB_INT_RESPONSE	0x01
#define RIRB_INT_OVERRUN	0x04
#define RIRB_INT_MASK		0x05



/* INTCTL and INTSTS */
#define INT_ALL_STREAM		0xff	   /* all stream interrupts */
#define INT_CTRL_EN			0x40000000 /* controller interrupt enable bit */
#define INT_GLOBAL_EN		0x80000000 /* global interrupt enable bit */

/*#define	INTCTL_BIT_GIE		0x80000000	//audiohd ones
#define	INTCTL_BIT_CIE		0x40000000
#define	INTCTL_BIT_SIE		0x3FFFFFFF*/

#define	INTSTS_BIT_GIS		0x80000000
#define	INTSTS_BIT_CIS		0x40000000


/* STATESTS int mask: SD2,SD1,SD0 */
#define MAX_CODECS			3
#define STATESTS_INT_MASK	0x07

#define HDA_IC				0x60
#define HDA_IR				0x64
#define HDA_IRS				0x68
#define	IRS_VALID			(1 << 1)
#define	IRS_BUSY			(1 << 0)
#define HDA_DPLBASE			0x70
#define HDA_DPUBASE			0x74
#define DPLBASE_ENABLE		0x1		/* Enable position buffer */


#define HDA_SD_BASE			0x80
#define HDA_SD_LEN			0x20


/*
 * Offset of Stream Descriptor Registers
 */
#define HDA_SD_CTL			0x0
#define HDA_SD_STS			0x3
#define HDA_SD_LPIB			0x4
#define HDA_SD_CBL			0x8
#define HDA_SD_LVI			0xC
#define HDA_SD_FIFOSIZE		0x10
#define HDA_SD_FORMAT		0x12
#define HDA_SD_BDLPL		0x18
#define HDA_SD_BDLPU		0x1C
#define HDA_SD_LPIBA		0x2004

/* SD_CTL bits */
#define SD_CTL_STREAM_RESET	0x01	/* stream reset bit */		// SD_CTL_SRST
#define SD_CTL_DMA_START	0x02	/* stream DMA start bit */	// SD_CTL_SRUN
#define SD_CTL_STREAM_TAG_MASK	(0xf << 20)
#define SD_CTL_STREAM_TAG_SHIFT	20

/* SD_CTL and SD_STS */
#define SD_INT_DESC_ERR		0x10	/* descriptor error interrupt */	/* SD_CTL_DEIE,SD_STS_DESE */
#define SD_INT_FIFO_ERR		0x08	/* FIFO error interrupt */			/* SD_CTL_FEIE,SD_STS_FIFOE */
#define SD_INT_COMPLETE		0x04	/* completion interrupt */			/* SD_CTL_IOCE,SD_STS_BCIS */
#define SD_INT_MASK		(SD_INT_DESC_ERR|SD_INT_FIFO_ERR|\
				 SD_INT_COMPLETE)										/* SD_CTL_INTS,SD_STS_INTRS */

/* SD_STS */
#define SD_STS_FIFO_READY	0x20	/* FIFO ready */ /* SD_STS_FIFORY */


class HDAPCIRegisters : public OSObject {

	OSDeclareDefaultStructors(HDAPCIRegisters)

	IOPCIDevice *pciDevice;
	IOMemoryMap *deviceMap;
	IOLock *mutex;
	volatile void *registers;

public:

	virtual bool init(IOPCIDevice *device);
	static HDAPCIRegisters *withPCIDevice(IOPCIDevice *device);
	
	virtual void free();

	virtual bool initPCIConfigSpace();

	virtual void lock();
	virtual void unlock();

	virtual IOPCIDevice *getDevice();

	/*
	 * RESTRICTED GENERAL REGISTERS ACCESS METHODS
	 * use register-specific methods instead
	 *
	 */

	inline void write32(unsigned int offset, UInt32 value) {
		OSWriteLittleInt32(registers, offset, value);
	}

    inline void write16(unsigned int offset, UInt16 value) {
		OSWriteLittleInt16(registers, offset, value);
	}

    inline void write8(unsigned int offset, UInt8 value) {
		*((UInt8*)registers + offset) = value;
	}

    inline UInt32 read32(unsigned int offset) {
		return OSReadLittleInt32(registers, offset);
	}

    inline UInt16 read16(unsigned int offset) {
		return OSReadLittleInt16(registers, offset);
	}

    inline UInt8 read8(unsigned int offset) {
		return *((UInt8*)registers + offset);
	}

	/* END OF RESTRICTED METHODS */

};

#endif
