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

#ifndef __HDACODEC_H__
#define __HDACODEC_H__

#include <IOKit/audio/IOAudioEngine.h>
#include "HDAController.h"

/* max. connections to a widget */
#define HDA_MAX_CONNECTIONS	32

/* max. codec address */
#define HDA_MAX_CODEC_ADDRESS	0x0f

/*
 * currently, only the format of 48K sample rate, 16-bit
 * 2-channel is supported.
 */
#define	HDA_FMT_PCMOUT	0x0011
#define	HDA_FMT_PCMIN	0x0011


class IOInterruptEventSource;

class HDACodec : public IOAudioEngine
{
    OSDeclareDefaultStructors(HDACodec)
    
	friend class HDAController;
	
    HDAController					*audioDevice;
	unsigned int					codecAddress;
    
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    
	IOAudioStream					*outputAudioStream;
	
	UInt32							afg, mfg;
	UInt32							vendorId;
	UInt32							subsystemId;
	UInt32							revisionId;
	UInt32							*wcaps;
	int								numNodes;
	UInt32							startNid;
	
	int nitrs;
	
	HDACommandTransmitter			*commandTransmitter;
	
public:

    virtual bool init(HDAController *device, unsigned int addr);
    virtual void free();
    
    virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize);

    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);

    virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    
	virtual void setupStream(UInt32 nid, UInt32 streamTag, int channelId, int format);
	virtual bool initDACNode(UInt32 nid);
	virtual bool initMixerNode(UInt32 nid, int indexNum);
	virtual bool unmuteOutputNode(UInt32 nid);
	virtual bool enablePinOutNode(UInt32 nid);
	virtual bool disablePinOutNode(UInt32 nid);
	virtual bool setPCMFormat(int dir, int format);

	virtual bool startPlayback(int stream);
	virtual bool resetStream(int stream);
	virtual bool fillPlaybackBuffer();
	virtual bool stopPlayback(int stream);
	virtual bool setFormat(int stream, int dir, int sample_rate, int channels, int precision, int encoding);
	virtual void getMaxGain(unsigned int &pgain, unsigned int &rgain);
	virtual bool setGain(int dir, int gain, int channel);
	virtual bool muteOutputs(bool mute);
	virtual unsigned int getCurrentSampleFrame(int stream);

	
	/* lowlevel interface */
	virtual void powerUp();
	virtual void powerDown();
	virtual UInt32 codecRead(UInt16 nid, int direct, unsigned int verb, unsigned int parm);
	virtual bool codecWrite(UInt16 nid, int direct, unsigned int verb, unsigned int parm);
	virtual UInt32 paramRead(UInt16 nid, unsigned int param);
	virtual void setupFunctionGroupNodes();
	virtual int getSubNodes(UInt32 nid, UInt32 &startId);
	virtual bool readWidgetCaps(UInt32 fgNode);
};


#endif /* __HDACODEC_H__ */
