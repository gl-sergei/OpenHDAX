/*
 *  Intel High Definition Audio Device Driver for OS X
 *
 *  Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 *  Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 *  was a great sample and probably start point for this driver.
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __HDACONTROLLER_H__
#define __HDACONTROLLER_H__


#include <IOKit/audio/IOAudioDevice.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOLocks.h>

#include "dma.h"

#define HDA_REG_AS(regs,reg,type)	(*((type*)(regs + reg)))

#define PCI_CLASS_MULTIMEDIA_AUDIO	0x0401
#define PCI_CLASS_MULTIMEDIA_OTHER	0x0480
#define PCI_VENDOR_ID				0x00
#define PCI_REVISION_ID				0x08
#define PCI_COMMAND					0x04
#define PCI_DEVICE_ID				0x02
#define PCI_INTERRUPT_LINE			0x3c
#define PCI_BASE_ADDRESS_0			0x10

#define PCI_MEM_BASE_ADDRESS_0		0x10
#define PCI_MEM_BASE_ADDRESS_1		0x14
#define PCI_MEM_BASE_ADDRESS_2		0x18
#define PCI_MEM_BASE_ADDRESS_3		0x1c
#define PCI_BASE_ADDRESS_1			0x14
#define PCI_BASE_ADDRESS_2			0x18
#define PCI_BASE_ADDRESS_3			0x1c
#define PCI_BASE_ADDRESS_4			0x20
#define PCI_BASE_ADDRESS_5			0x24
#define PCI_COMMAND_IO         		0x01
#define PCI_COMMAND_MEMORY			0x02
#define PCI_COMMAND_MASTER			0x04
#define PCI_COMMAND_PARITY			0x40
#define PCI_COMMAND_SERR			0x100

#define PCI_STATUS					0x06
#define PCI_SUBSYSTEM_VENDOR_ID		0x2c
#define PCI_SUBSYSTEM_ID			0x2e


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
#define HDA_WAKESTS			0x0E	/* Wake Status */
#define HDA_STATESTS		HDA_WAKESTS /* State Change Status */
#define HDA_GSTST			0x10	/* Global Status */
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

#define	INTCTL_BIT_GIE		0x80000000	//audiohd ones
#define	INTCTL_BIT_CIE		0x40000000
#define	INTCTL_BIT_SIE		0x3FFFFFFF


/* STATESTS int mask: SD2,SD1,SD0 */
#define MAX_CODECS			3
#define STATESTS_INT_MASK	0x07

#define HDA_ICOI	        0x60	/* Immediate Command Output Interface */
#define HDA_ICII	        0x64	/* Immediate Command Input Interface */
#define HDA_ICIS			0x68	/* Immediate Command Status */
#define HDA_DPIBLBASE 		0x70	/* DMA Position Buffer Lower Base */
#define HDA_DPIBUBASE 		0x74	/* DMA Position Buffer Upper Base */

/* aliases */
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


/* some defaults */
#define HDA_SAMPLER_MAX			48000
#define HDA_MAX_CHANNELS		2
#define HDA_MAX_PRECISION		16
#define HDA_BDLE_BUF_ALIGN		128
#define HDA_BDLE_NUMS			4

#define	PLAY_STARTED		0x00000001
#define	PLAY_EMPTY			0x00000002
#define	PLAY_PAUSED			0x00000004
#define	RECORD_STARTED		0x00000008

#define	HDA_SAMPR48000		48000

/* general terminology */
#define	AUDIO_PLAY			0x0001		/* output */
#define	AUDIO_RECORD			0x0002		/* input */
#define	AUDIO_BOTH			(AUDIO_PLAY|AUDIO_RECORD)
#define	AUDIO_NO_SLEEP			0x0004
#define	AUDIO_SLEEP			0x0008

/*
 * Generic minimum/maximum limits for number of channels, both modes
 */
#define	AUDIO_CHANNELS_MONO	(1)
#define	AUDIO_CHANNELS_STEREO	(2)
#define	AUDIO_MIN_PLAY_CHANNELS	(AUDIO_CHANNELS_MONO)
#define	AUDIO_MAX_PLAY_CHANNELS	(AUDIO_CHANNELS_STEREO)
#define	AUDIO_MIN_REC_CHANNELS	(AUDIO_CHANNELS_MONO)
#define	AUDIO_MAX_REC_CHANNELS	(AUDIO_CHANNELS_STEREO)

/*
 * Generic minimum/maximum limits for sample precision
 */
#define	AUDIO_PRECISION_8		(8)
#define	AUDIO_PRECISION_16		(16)

/*
 * Audio encoding types
 */
#define	AUDIO_ENCODING_NONE	(0)	/* no encoding assigned	*/
#define	AUDIO_ENCODING_ULAW	(1)	/* u-law encoding	*/
#define	AUDIO_ENCODING_ALAW	(2)	/* A-law encoding	*/
#define	AUDIO_ENCODING_LINEAR	(3)	/* Signed Linear PCM encoding	*/
#define	AUDIO_ENCODING_DVI	(104)	/* DVI ADPCM		*/
#define	AUDIO_ENCODING_LINEAR8	(105)	/* 8 bit UNSIGNED	*/

typedef volatile void * Registers;


struct CommandRingBuffer {
	PH64 phaddr;					// physical address
	UInt32 *buf;					// virtual address
	UInt16 rp, wp;					// read/write pointers
	int cmds;						// number of pending requests
	UInt32 res;						// last read value
};

// Buffer Descriptor List entry of stream descriptor
struct BDLEntry {
	UInt64 addr;
	UInt32 len;
	UInt32
			ioc: 1,
			reserved: 31;
};

class IOPCIDevice;
class IOMemoryMap;
class org_barnaul_driver_HDACodec;
class org_barnaul_driver_HDAController;

#define HDAController org_barnaul_driver_HDAController
#define HDACodec org_barnaul_driver_HDACodec

class HDAController : public IOAudioDevice
{
	OSDeclareDefaultStructors(HDAController)
	
	friend class HDACodec;
  
public:  
    IOPCIDevice						*pciDevice;
    IOMemoryMap						*deviceMap;
    HDACodec			*audioEngine;
	
	unsigned int codecMask;

    Registers						deviceRegisters;			// PCI registers map
	HDADMABuffer					*commandBuffer;				// CORB and RIRB buffer (allocated once for both)
	CommandRingBuffer				corb;						// CORB
	CommandRingBuffer				rirb;						// RIRB
    IOFilterInterruptEventSource	*interruptEventSource;

	/* regs spinlock */
	IOSimpleLock					*regsSpinLock;
	/* command lock mutex */
	IOLock							*commandMutex;
	/* mutex from audiohd.c */
	IOLock							*mutex;

	/* flags */
	bool isSingleCommand;
	
	/* debug */
	int interruptsHandled;
	int interruptsAquired;
	int unsolicited;

	/* playback */
	HDADMABuffer					*playbackBufferDescriptor;
	HDADMABuffer					*playbackBuffer;
	HDADMABuffer					*recordBufferDescriptor;
	HDADMABuffer					*recordBuffer;
	HDADMABuffer					*positionBuffer;
	
	int								inputStreams;
	int								outputStreams;
	int								streamsNums;
	int								numStreams;
	
	int								playbackBufferPosition;
	int								recordBufferPosition;
	
	bool							dma64ok;
	
	// direct copypast from Open Solaris audiohd driver
	
	UInt32							flags;
	
	int								csamples;
	int								psamples;
	int								psampleRate;
	int								csampleRate;
	int								pchannels;
	int								pprecision;
	int								cchannels;
	int								cprecision;
	int								playbackInterruptFrequence;
	int								recordInterruptFrequence;
	int								playbackBufferSize;
	int								recordBufferSize;
	
	bool							outputsMuted;

	unsigned int					playbackStreamTag;		/* tag of playback stream */
	unsigned int					recordStreamTag;		/* tag of record stream */
	unsigned int					playbackRegistryBase;	/* regbase for play stream */
	unsigned int					recordRegistryBase;		/* regbase for record stream */
	unsigned int					playbackLeftGain;		/* left gain for playback */
	unsigned int					playbackRightGain;		/* right gain for playback */
	unsigned int					playbackGainMax;		/* max gain for playback */
	unsigned int					recordLeftGain;			/* left gain for recording */
	unsigned int					recordRightGain;		/* right gain for recording */
	unsigned int					recordGainMax;			/* max gain for record */
	unsigned int					playbackFormat;
	unsigned int					recordFormat;
	unsigned int					outputPorts;			/* active outputs */
	unsigned int					inputPorts;				/* active inputs */

    virtual bool initHardware(IOService *provider);
	virtual bool initPCIConfigSpace(IOPCIDevice *provider);
    virtual bool createAudioEngine();
	virtual bool initCommandIO();
	virtual void enableInterrupts();
	virtual void disableInterrupts();
	virtual void clearInterrupts();
    virtual void free();
	virtual bool resetController();
	virtual bool initController();
	virtual bool initHDA();
	virtual bool allocateRegsSpinLock();
	virtual bool freeRegsSpinLock();
	virtual bool allocateCommandMutex();
	virtual bool freeCommandMutex();
	virtual bool allocateMutex();
	virtual bool freeMutex();
	virtual bool filterInterrupt(IOInterruptEventSource *source);
	virtual void handleInterrupt(IOInterruptEventSource *source, int count);
	
	virtual void stopAllDMA();
	virtual void softInit();
	virtual bool allocatePlaybackBuffers();
	virtual bool allocateRecordBuffers();
	virtual void freePlaybackBuffers();
	virtual void freeRecordBuffers();
	virtual bool allocatePositionBuffer();
	virtual void freePositionBuffer();
	
	/* CORB/RIRB block (not just but command IO in general */
	virtual bool allocateRingBuffers();
	virtual void freeRingBuffers();
	virtual void disableRingBuffers();
	virtual bool corbSendCommand(UInt32 val);
	virtual void updateRirb();
	virtual UInt32 rirbGetResponse();
	virtual bool singleSendCommand(UInt32 val);
	virtual UInt32 singleGetResponse();
//public:
	/* public CORB/RIRB interface */
	virtual bool sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para);
	virtual UInt32 getResponse();
	virtual void commandLock();
	virtual void commandUnlock();
	/* end of CORB/RIRB block */
	
//private:
    
	inline void regsWrite32(UInt16 offset, UInt32 value) {
		OSWriteLittleInt32(deviceRegisters, offset, value);
	}

    inline void regsWrite16(UInt16 offset, UInt16 value) {
		OSWriteLittleInt16(deviceRegisters, offset, value);
	}

    inline void regsWrite8(UInt16 offset, UInt8 value) {
		*((UInt8*)deviceRegisters + offset) = value;
	}

    inline UInt32 regsRead32(UInt16 offset) {
		return OSReadLittleInt32(deviceRegisters, offset);
	}

    inline UInt16 regsRead16(UInt16 offset) {
		return OSReadLittleInt16(deviceRegisters, offset);
	}

    inline UInt8 regsRead8(UInt16 offset) {
		return *((UInt8*)deviceRegisters + offset);
	}

	
    static IOReturn volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);

    static IOReturn gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
};

#endif /* __HDACONTROLLER_H__ */
