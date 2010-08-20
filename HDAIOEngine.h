/*
 *  HDAIOStream.h
 *  OpenHDAX
 *
 *  Created by Юрий Гагарин on 6/28/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HDA_IO_ENGINE__

#include <IOKit/audio/IOAudioEngine.h>
#include "HDAController.h"
#include "HDAWidget.h"

class HDAIOEngine : public IOAudioEngine
{

    OSDeclareDefaultStructors(HDAIOEngine)

protected:

	HDAAudioWidget **pins;
	HDAAudioWidget **mixers;
	HDAAudioWidget **selectors;
	HDAAudioWidget **convertors;
	
	unsigned streamId, channelId, streamIndex, regbase;
	unsigned hdaFormat;
	IOAudioStreamDirection streamDirection;
	HDAPCIRegisters *regs;
	HDADMABuffer *positionBuffer;
	HDADMABuffer *sampleBuffer;
	HDADMABuffer *sampleBufferDescriptor;
	HDAController *controller;
	IOAudioStreamFormat currentStreamFormat;

	virtual void setWidgets(HDAAudioWidget **&dest, HDAAudioWidget **source, int count);
	virtual bool allocateSampleBuffer();
	virtual void freeSampleBuffer();
	virtual unsigned calcStreamFormat(unsigned rate, unsigned channels, unsigned bps);
	virtual unsigned calcStreamFormat(const IOAudioStreamFormat *format, const IOAudioSampleRate *sampleRate);
	virtual bool resetStream();
	virtual bool setConvertorsFormat(unsigned format);
	virtual void setupBDLE();
	virtual bool startStream();
	virtual bool stopStream();

	IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize);

	virtual bool createAudioStream();
	virtual bool createControls();

public:

    virtual bool init(IOAudioStreamDirection streamDirection, HDAController *controller, unsigned streamId, unsigned channelId);
    virtual void free();

	virtual void setPins(HDAAudioWidget **widgets, int count);
	virtual void setMixers(HDAAudioWidget **widgets, int count);
	virtual void setSelectors(HDAAudioWidget **widgets, int count);
	virtual void setConvertors(HDAAudioWidget **widgets, int count);

	virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
	virtual IOReturn performAudioEngineStart();
	virtual IOReturn performAudioEngineStop();
	virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);
    virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
	static IOReturn volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
	virtual IOReturn volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
	static IOReturn muteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
	virtual IOReturn muteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);

	virtual void handleStreamInterrupt(unsigned streamId);

    virtual UInt32 getCurrentSampleFrame();

};

#endif

