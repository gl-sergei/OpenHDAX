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
#include "HDAPCIRegisters.h"
#include "HDACommandTransmitter.h"


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
class org_barnaul_driver_HDAController;

#define HDAController org_barnaul_driver_HDAController

class HDAIOEngine;
class HDAAudioWidget;

class HDAController : public IOAudioDevice
{
	OSDeclareDefaultStructors(HDAController)
	
public:  
    IOPCIDevice						*pciDevice;
	HDAPCIRegisters					*deviceRegs;
	HDACommandTransmitter			*commandTransmitter;
	HDAAudioWidget					*widgets;

	unsigned int codecMask;

	IOFilterInterruptEventSource	*interruptEventSource;

	/* mutex from audiohd.c */
	IOLock							*mutex;

	/* debug */
	int interruptsHandled;
	int interruptsAquired;
	int unsolicited;

	/* position */
	HDADMABuffer					*positionBuffer;
	
	int								inputStreams;
	int								outputStreams;
	int								streamsNums;
	int								numStreams;
	
	bool							dma64ok;
	
	UInt32							interruptStatus;
	
	virtual bool init(OSDictionary *dictionary);
    virtual bool initHardware(IOService *provider);
    virtual bool createAudioEngine();
	virtual void enableInterrupts();
	virtual void disableInterrupts();
	virtual void clearInterrupts();
	virtual void stop(IOService *provider);
    virtual void free();
	virtual bool resetController();
	virtual bool initController();
	virtual bool initHDA();
	virtual bool allocateMutex();
	virtual bool freeMutex();
	virtual void handleInterrupt(IOFilterInterruptEventSource *source, int count);
	virtual bool filterInterrupt(IOFilterInterruptEventSource *source);

	virtual void enablePositionBuffer();
	virtual void disablePositionBuffer();
	virtual void stopAllDMA();
	virtual bool allocatePositionBuffer();
	virtual void freePositionBuffer();
	
	virtual IOReturn newUserClient( task_t owningTask, void*, UInt32 type, IOUserClient **handler);
	
//private:
    
	inline void regsWrite32(unsigned int offset, UInt32 value) {
		deviceRegs->write32(offset, value);
	}

    inline void regsWrite16(unsigned int offset, UInt16 value) {
		deviceRegs->write16(offset, value);
	}

    inline void regsWrite8(unsigned int offset, UInt8 value) {
		deviceRegs->write8(offset, value);
	}

    inline UInt32 regsRead32(unsigned int offset) {
		return deviceRegs->read32(offset);
	}

    inline UInt16 regsRead16(unsigned int offset) {
		return deviceRegs->read16(offset);
	}

    inline UInt8 regsRead8(unsigned int offset) {
		return deviceRegs->read8(offset);
	}

	virtual HDACommandTransmitter *getCommandTransmitter();
	virtual HDAPCIRegisters *getPCIRegisters();
	virtual HDADMABuffer *getPositionBuffer();

	virtual unsigned int getStreamBaseRegByTag(int stream);
};

#endif /* __HDACONTROLLER_H__ */
