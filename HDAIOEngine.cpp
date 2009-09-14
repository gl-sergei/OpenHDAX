/*
 *  HDAIOStream.cpp
 *  OpenHDAX
 *
 *  Created by Юрий Гагарин on 6/28/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDAIOEngine.h"
#include "HDACodecCommon.h"

#include <IOKit/audio/IOAudioControl.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioDefines.h>

static const int numberOfSamples = 8192 * 4;
static const int sampleBufferSize = numberOfSamples * HDA_MAX_CHANNELS * (HDA_MAX_PRECISION / 8);

#define super IOAudioEngine

OSDefineMetaClassAndStructors(HDAIOEngine, IOAudioEngine)

bool HDAIOEngine::init(IOAudioStreamDirection streamDirection, HDAController *controller, unsigned streamId, unsigned channelId)
{

	regs = controller->getPCIRegisters();
	positionBuffer = controller->getPositionBuffer();
	this->controller = controller;
	this->channelId = channelId;
	this->streamId = streamId;
	this->streamDirection = streamDirection;
	streamIndex = streamId - 1;
	regbase = HDA_SD_BASE + HDA_SD_LEN * streamIndex;

	pins = mixers = selectors = convertors = NULL;
	regs->retain();
	positionBuffer->retain();
	controller->retain();

	IOLog("HDAIOEngine with streamId = %d and channelId = %d created\n", streamId, channelId);
	IOLog("regs = %p, positionBuffer = %p, streamIndex = %d", regs, positionBuffer, streamIndex);

	if (!allocateSampleBuffer())
	{
		IOLog("failed to allocate sample buffer\n");
		return false;
	}

	/* calling super::init */
	IOLog("calling super::init");
	if (!super::init(NULL))
	{
		return false;
	}

	return true;
}

void HDAIOEngine::free()
{
	IOLog("HDAIOEngine[%p]::free()\n", this);
	
	if (pins) {
		IOLog("HDAIOEngine[%p]::free() delete [] pins\n", this);
		delete [] pins;
	}
	if (mixers) {
		IOLog("HDAIOEngine[%p]::free() delete [] mixers\n", this);
		delete [] mixers;
	}
	if (selectors) {
		IOLog("HDAIOEngine[%p]::free() delete [] selectors\n", this);
		delete [] selectors;
	}
	if (convertors) {
		IOLog("HDAIOEngine[%p]::free() delete [] convertors\n", this);
		delete [] convertors;
	}
	IOLog("HDAIOEngine[%p]::free() free sample buffer\n", this);
	freeSampleBuffer();
	if (regs) {
		IOLog("HDAIOEngine[%p]::free() release regs\n", this);
		regs->release();
	}
	if (positionBuffer) {
		IOLog("HDAIOEngine[%p]::free() release position buffer\n", this);
		positionBuffer->release();
	}
	if (controller) {
		IOLog("HDAIOEngine[%p]::free() release controller\n", this);
		controller->release();
	}

	IOLog("HDAIOEngine[%p]::free() super free\n", this);
	super::free();
}

void HDAIOEngine::setWidgets(HDAAudioWidget **&dest, HDAAudioWidget **source, int count)
{
	int i;

	IOLog("HDAIOEngine::setWidgets(%p, %p, %d)\n", dest, source, count);
	if (dest) delete [] dest;
	dest = new HDAAudioWidget *[count + 1];

	for (i = 0; i < count; i++)
	{
		dest[i] = source[i];
//		dest[i]->print();
	}
	dest[count] = NULL;

}

bool HDAIOEngine::allocateSampleBuffer() {

	const int dma64ok = true;

	IOLog("allocateSampleBuffer: size = %d, dma64ok=%d\n", sampleBufferSize, (int)dma64ok);

	/* allocate DMA for data buffer of playback stream */
	sampleBuffer = HDADMABuffer::withSize(sampleBufferSize, dma64ok);
	if (!sampleBuffer)
		return false;

	/* allocate DMA for buffer descriptor list of playback stream */
	sampleBufferDescriptor = HDADMABuffer::withSize(sizeof(BDLEntry) * HDA_BDLE_NUMS);
	if (!sampleBufferDescriptor)
		return false;

	IOLog("buffer successfully allocated\n");

	return true;
}

void HDAIOEngine::freeSampleBuffer() {

	IOLog("HDAIOEngine[%p]::freeSampleBuffer\n", this);

	if (sampleBuffer) {
		IOLog("HDAIOEngine[%p]::freeSampleBuffer sampleBuffer->release()\n", this);
		sampleBuffer->release();
	}

	if (sampleBufferDescriptor) {
		IOLog("HDAIOEngine[%p]::freeSampleBuffer sampleBufferDescriptor->release()\n", this);
		sampleBufferDescriptor->release();
	}

}

void HDAIOEngine::setPins(HDAAudioWidget **widgets, int count)
{
	IOLog("HDAIOEngine::setPins(%p, %d)\n", widgets, count);
	setWidgets(pins, widgets, count);
}

void HDAIOEngine::setMixers(HDAAudioWidget **widgets, int count)
{
	IOLog("HDAIOEngine::setMixers(%p, %d)\n", widgets, count);
	setWidgets(mixers, widgets, count);
}

void HDAIOEngine::setSelectors(HDAAudioWidget **widgets, int count)
{
	IOLog("HDAIOEngine::setSelectors(%p, %d)\n", widgets, count);
	setWidgets(selectors, widgets, count);
}

void HDAIOEngine::setConvertors(HDAAudioWidget **widgets, int count)
{
	IOLog("HDAIOEngine::setConvertors(%p, %d)\n", widgets, count);
	setWidgets(convertors, widgets, count);
}

bool HDAIOEngine::initHardware(IOService *provider)
{

	HDAAudioWidget **w;
	bool result = true;

	IOLog("HDAIOEngine::initHardware()\n");

    if (!super::initHardware(provider)) {
        return false;
    }

	// associate all DAC/ADCs with specified streamId, channelId
	if (convertors)
	{
		IOLog("setup convertors");
		for (w = convertors; *w != NULL; w++)
		{
			result = result && (*w)->setConverterStreamChannel(streamId, channelId);
			if (/*(*w)->wcaps.in_amp_present && */(streamDirection == kIOAudioStreamDirectionInput))
			{
				result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_INPUT, 0, AC_GAIN_MAX);
				result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, AC_AMP_MUTE);
			}
			if ((*w)->wcaps.out_amp_present && (streamDirection == kIOAudioStreamDirectionOutput))
				result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, AC_GAIN_MAX);
			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}

	if (mixers)
	{
		IOLog("setup mixers");
		for (w = mixers; *w != NULL; w++)
		{
//				if (streamDirection == kIOAudioStreamDirectionInput)
				{
					/* unmute input of mixer */
					result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_INPUT, 0, AC_GAIN_MAX / 2);
				}

//				if (streamDirection == kIOAudioStreamDirectionOutput)
				{
					/* output left amp of mixer */
					/* output right amp of mixer */
					result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, AC_GAIN_MAX / 2);
				}
			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}


	if (pins)
	{
		IOLog("setup pins again");
		for (w = pins; *w != NULL; w++)
		{
			unsigned ctrl = 0;
			if (streamDirection == kIOAudioStreamDirectionOutput)
				ctrl |= AC_PINCTL_OUT_EN;
			if ((*w)->pincaps.headphone_drive_capable)
				ctrl |= AC_PINCTL_HP_EN;
			if (streamDirection == kIOAudioStreamDirectionInput)
				ctrl |= AC_PINCTL_IN_EN | AC_PINCTL_VREF_80;
			result = result && (*w)->setPinControl(ctrl);
			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}

	if (pins)
	{
		IOLog("setup pins");
		for (w = pins; *w != NULL; w++)
		{
			/* unmute pins */
			if (/*(*w)->wcaps.in_amp_present && */(streamDirection == kIOAudioStreamDirectionInput))
			{
				(*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_INPUT, 0, AC_AMP_MUTE);
				(*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, AC_AMP_MUTE);
			}
			if ((*w)->wcaps.out_amp_present && (streamDirection == kIOAudioStreamDirectionOutput))
			{
				(*w)->setAmplifierGainMute(AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, AC_GAIN_MAX);
				(*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_OUTPUT, 0, AC_GAIN_MAX);
			}
//			result = result && (*w)->setAmplifierGainMute(AC_AMP_SET_LEFT | AC_AMP_SET_RIGHT | AC_AMP_SET_OUTPUT | AC_AMP_SET_INPUT, 0, AC_GAIN_MAX);
			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}

	IOLog("set description\n");
    setDescription("Realtek ALC260");
    
	IOLog("set initial sample rate\n");
    IOAudioSampleRate initialSampleRate;
    initialSampleRate.whole = 48000;
    initialSampleRate.fraction = 0;
    setSampleRate(&initialSampleRate);

    setNumSampleFramesPerBuffer(numberOfSamples);

	createAudioStream();
	createControls();

	IOLog("HDAIOEngine::initHardware() done\n");

	return true;
}

void HDAIOEngine::stop(IOService *provider)
{
	/*
	 * implement stop method
	 */
    IOLog("HDAIOEngine[%p]::stop(%p)\n", this, provider);

	super::stop(provider);
}

unsigned HDAIOEngine::calcStreamFormat(unsigned rate, unsigned channels, unsigned bps)
{

	static const struct { unsigned rate; unsigned format; } rateBits[] = {
		{ 8000, 0x0500 }, /* 1/6 x 48 */
		{ 11025, 0x4300 }, /* 1/4 x 44 */
		{ 16000, 0x0200 }, /* 1/3 x 48 */
		{ 22050, 0x4100 }, /* 1/2 x 44 */
		{ 32000, 0x0a00 }, /* 2/3 x 48 */
		{ 44100, 0x4000 }, /* 44 */
		{ 48000, 0x0000 }, /* 48 */
		{ 88200, 0x4800 }, /* 2 x 44 */
		{ 96000, 0x0800 }, /* 2 x 48 */
		{ 176400, 0x5800 },/* 4 x 44 */
		{ 192000, 0x1800 }, /* 4 x 48 */
		{ 0 } /* terminator */
	};

	int i;
	unsigned result = 0;

	for (i = 0; rateBits[i].rate; i++)
		if (rateBits[i].rate == rate) {
			result = rateBits[i].format;
			break;
		}
	if (!rateBits[i].rate) {
		IOLog("invalid rate %d\n", rate);
		return 0;
	}
	if (channels == 0 || channels > 8) {
		IOLog("invalid channels %d\n", channels);
		return 0;
	}
	result |= channels - 1;

	switch (bps) {
		case 8:
			result |= 0x00;
			break;
		case 16:
			result |= 0x10;
			break;
		case 20:
			result |= 0x20;
			break;
		case 24:
			result |= 0x30;
			break;
		case 32:
			result |= 0x40;
			break;
		default:
			IOLog("invalid bps %d\n", bps);
			return 0;
	}

	return result;
}

unsigned HDAIOEngine::calcStreamFormat(const IOAudioStreamFormat *format, const IOAudioSampleRate *sampleRate)
{
	return calcStreamFormat(sampleRate->whole, format->fNumChannels, format->fBitDepth);
}

bool HDAIOEngine::resetStream() {
	UInt8 btmp;
	int i;
	
	btmp = regs->read8(regbase + HDA_SD_CTL);
	
	/* stop stream */
	btmp &= ~HDA_RIRBSIZE;
	regs->write8(regbase + HDA_SD_CTL, btmp);
	
	/* wait 40us for stream to stop as HD spec */
	IODelay(40);
	
	/* reset stream */
	btmp |= SD_CTL_STREAM_RESET;
	regs->write8(regbase + HDA_SD_CTL, btmp);
	
	for (i = 0; i < 50; i++) {
		IODelay(10);
		btmp = regs->read8(regbase + HDA_SD_CTL);
		btmp &= SD_CTL_STREAM_RESET;
		if (btmp)
			break;
	}
	
	if (!btmp) {
		IOLog("Failed to reset stream %d\n", streamId);
		return false;
	}
	
	/* Need any RESET# assertion time, 300us ??? */
	IODelay(50);
	
	/* exit reset stream */
	btmp &= ~SD_CTL_STREAM_RESET;
	regs->write8(regbase + HDA_SD_CTL, btmp);
	
	for (i = 0; i < 50; i++) {
		IODelay(10);
		btmp = regs->read8(regbase + HDA_SD_CTL);
		btmp &= SD_CTL_STREAM_RESET;
		if (!btmp)
			break;
	}
	
	if (btmp) {
		IOLog("Failed to exit reset state for stream %d, btmp=0x%02x\n", streamId, (int)btmp);
		return false;
	}
	
	return true;
}

bool HDAIOEngine::setConvertorsFormat(unsigned format)
{

	HDAAudioWidget **w;
	bool result = true;

	if (convertors)
	{
		IOLog("settings convertors format 0x%x ", format);
		for (w = convertors; *w != NULL; w++)
		{
			result = result && (*w)->setStreamFormat(format);
			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}
	return result;
}

void HDAIOEngine::setupBDLE()
{
	UInt64 bufPhysAddr;
	BDLEntry *entry;
	char *buf;
	int i;
	
	entry = (BDLEntry*)sampleBufferDescriptor->getVirtualAddress();
	buf = (char*)sampleBuffer->getVirtualAddress();
	bufPhysAddr = sampleBuffer->getPhysicalAddress().whole64();
	
	for (i = 0; i < HDA_BDLE_NUMS; i++) {
		entry->addr = bufPhysAddr;
		entry->len = sampleBufferSize / HDA_BDLE_NUMS;
		entry->ioc = 0;
		bufPhysAddr += sampleBufferSize / HDA_BDLE_NUMS;
		entry++;
	}
	(entry - 1)->ioc = 1;			// we need an interrupt to be generated at the end of whole cyclic buffer

}

bool HDAIOEngine::startStream()
{
	PH64 sbdPhysAddr;

	IOLog("HDAIOEngine::startStream\n");

	resetStream();
	setupBDLE();

	sbdPhysAddr = sampleBufferDescriptor->getPhysicalAddress();
	regs->write32(regbase + HDA_SD_BDLPL, sbdPhysAddr.low32());
	regs->write32(regbase + HDA_SD_BDLPU, sbdPhysAddr.hi32());
	regs->write16(regbase + HDA_SD_LVI, HDA_BDLE_NUMS - 1);
	regs->write32(regbase + HDA_SD_CBL, sampleBufferSize);
	
	regs->write32(regbase + HDA_SD_FORMAT, hdaFormat);
	setConvertorsFormat(hdaFormat);
	
	/* clear status */
	regs->write8(regbase + HDA_SD_STS, SD_INT_DESC_ERR | SD_INT_FIFO_ERR | SD_INT_COMPLETE);
	
	/* set playback stream tag */
	regs->write8(regbase + HDA_SD_CTL + 2, streamId << 4 | 4);

	/* enable SIE */
	regs->write8(HDA_INTCTL, regs->read8(HDA_INTCTL) | (1 << streamId));

	/* enable interrupt and start dma */
	regs->write8(regbase + HDA_SD_CTL, SD_INT_MASK | SD_CTL_DMA_START);

	if (streamDirection == kIOAudioStreamDirectionInput && pins)
	{
		if (pins)
		{
			HDAAudioWidget **w;
			IOLog("turn on pins input and vref");
			for (w = pins; *w != NULL; w++)
			{
				(*w)->setPinControl(AC_PINCTL_IN_EN | AC_PINCTL_VREF_80);
				IOLog(" ...0x%x", (*w)->nid);
			}
			IOLog(" done\n");
		}
	}

	return true;
}

bool HDAIOEngine::stopStream()
{
	IOLog("HDAIOEngine::stopStream\n");
	
	/* stop DMA */
	regs->write8(regbase + HDA_SD_CTL,
				regs->read8(regbase + HDA_SD_CTL) & ~(SD_CTL_DMA_START | SD_INT_MASK));
	regs->write8(regbase + HDA_SD_STS, SD_INT_MASK);

	/* disable SIE */
	regs->write8(regbase + HDA_INTCTL,
				regs->read8(regbase + HDA_INTCTL) & ~(1 << streamId));

	if (streamDirection == kIOAudioStreamDirectionInput && pins)
	{
		if (pins)
		{
			HDAAudioWidget **w;
			IOLog("turn off pins input and vref");
			for (w = pins; *w != NULL; w++)
			{
				(*w)->setPinControl(0);
				IOLog(" ...0x%x", (*w)->nid);
			}
			IOLog(" done\n");
		}
	}
	
	return true;
}

IOAudioStream *HDAIOEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
{
    IOAudioStream *audioStream;
    
    // For this sample device, we are only creating a single format and allowing 44.1KHz and 48KHz
    audioStream = new IOAudioStream;
    if (audioStream) {
        if (!audioStream->initWithAudioEngine(this, direction, 1)) {
            audioStream->release();
			audioStream = NULL;
        } else {
            IOAudioSampleRate rate;
            IOAudioStreamFormat format = {
                2,												// num channels
                kIOAudioStreamSampleFormatLinearPCM,			// sample format
                kIOAudioStreamNumericRepresentationSignedInt,	// numeric format
                16,												// bit depth
                16,												// bit width
                kIOAudioStreamAlignmentHighByte,				// high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,			// little endian
                true,											// format is mixable
                0												// driver-defined tag - unused by this driver
            };
            
            // As part of creating a new IOAudioStream, its sample buffer needs to be set
            // It will automatically create a mix buffer should it be needed
            audioStream->setSampleBuffer(sampleBuffer, sampleBufferSize);
            
            // This device only allows a single format and a choice of 2 different sample rates
            rate.fraction = 0;
            rate.whole = 48000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            
            // Finally, the IOAudioStream's current format needs to be indicated
            audioStream->setFormat(&format);
			currentStreamFormat = format;
        }
    }
    
    return audioStream;
}

bool HDAIOEngine::createAudioStream()
{

	IOLog("HDAIOEngine::createAudioStream()\n");

	if (convertors)
	{
		IOAudioStream *audioStream = createNewAudioStream(streamDirection, sampleBuffer->getVirtualAddress(), sampleBufferSize);
		if (!audioStream)
			return false;
		addAudioStream(audioStream);
		audioStream->release();
	}

	return true;
}

bool HDAIOEngine::createControls()
{

	IOLog("HDAIOEngine::createControls()\n");

	IOAudioControl *control;
	UInt32 usage;

	usage = (streamDirection == kIOAudioStreamDirectionOutput) ? kIOAudioControlUsageOutput : kIOAudioControlUsageInput;
	/* Left gain */
    control = IOAudioLevelControl::createVolumeControl(AC_GAIN_MAX,
                                                        0,
                                                        AC_GAIN_MAX,
                                                        0,
                                                        AC_GAIN_MAX * 256,
                                                        kIOAudioControlChannelIDDefaultLeft,
                                                        kIOAudioControlChannelNameLeft,
                                                        0,		// control ID - driver-defined
                                                        usage);

    if (!control) {
        return false;
    }

	control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
	addDefaultAudioControl(control);
	control->release();

	/* Right gain */
    control = IOAudioLevelControl::createVolumeControl(AC_GAIN_MAX,
                                                        0,
                                                        AC_GAIN_MAX,
                                                        0,
                                                        AC_GAIN_MAX * 256,
                                                        kIOAudioControlChannelIDDefaultRight,
                                                        kIOAudioControlChannelNameRight,
                                                        0,		// control ID - driver-defined
                                                        usage);

    if (!control) {
        return false;
    }

	control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
	addDefaultAudioControl(control);
	control->release();

    /* mute */
    control = IOAudioToggleControl::createMuteControl(false,	// initial state - unmuted
                                                        kIOAudioControlChannelIDAll,
                                                        kIOAudioControlChannelNameAll,
                                                        0,		// control ID - driver-defined
                                                        usage);

    if (!control) {
        return false;
    }

    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)muteChangeHandler, this);
    addDefaultAudioControl(control);
    control->release();

	return true;
}

void HDAIOEngine::handleStreamInterrupt(unsigned sId)
{
	if (sId == streamId)
		takeTimeStamp();
}

/*
 * Implementation of IOAudioEngine interface
 */
UInt32 HDAIOEngine::getCurrentSampleFrame()
{
	unsigned position;
	
	position = OSReadLittleInt32(positionBuffer->getVirtualAddress(), streamIndex * 8);
	position /= currentStreamFormat.fNumChannels * currentStreamFormat.fBitWidth / 8;

	if (position > numberOfSamples) {
		IOLog("DMA position is greater than number of sample frames\nposition = %d, numberOfSamples = %d\n", position, numberOfSamples);
		position = numberOfSamples;
	}
	
	return position;
}

IOReturn HDAIOEngine::performAudioEngineStart()
{

	IOLog("HDAIOEngine[%p]::performAudioEngineStart()\n", this);

	// hardcoded zero for stream which just started
	OSWriteLittleInt32(positionBuffer->getVirtualAddress(), streamIndex * 8, 0);
//	controller->enablePositionBuffer();

	startStream();

	takeTimeStamp(false);

	IOLog("done");

	return kIOReturnSuccess;
}

IOReturn HDAIOEngine::performAudioEngineStop()
{
    IOLog("HDAIOEngine[%p]::performAudioEngineStop()\n", this);
    
	stopStream();
	
    return kIOReturnSuccess;
}

IOReturn HDAIOEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("HDAIOEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
	const IOAudioStreamFormat *formatToSet = newFormat != NULL ? newFormat : audioStream->getFormat();
	const IOAudioSampleRate *sampleRateToset = newSampleRate != NULL ? newSampleRate : getSampleRate();
	
	unsigned newHdaFormat = calcStreamFormat(formatToSet, sampleRateToset);

	if (!newHdaFormat)
	{
		IOLog("wrong format\n");
		return kIOReturnInvalid;
	}
	
	hdaFormat = newHdaFormat;
	regs->write32(regbase + HDA_SD_FORMAT, hdaFormat);
	setConvertorsFormat(hdaFormat);
	currentStreamFormat = *formatToSet;
	
    return kIOReturnSuccess;
}

IOReturn HDAIOEngine::volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAIOEngine *audioEngine;
    
    audioEngine = (HDAIOEngine *)target;
    if (audioEngine) {
        result = audioEngine->volumeChanged(volumeControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAIOEngine::volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAIOEngine[%p]::volumeChanged(%p, %ld, %ld)\n", this, volumeControl, oldValue, newValue);
    
	HDAAudioWidget **w;
	unsigned lr, inout;
	
	lr = 0;
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultLeft)
		lr |= AC_AMP_SET_LEFT;
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultRight)
		lr |= AC_AMP_SET_RIGHT;
	
    if (volumeControl) {

        IOLog("\t-> Channel %ld\n", volumeControl->getChannelID());

		if (convertors)
		{
			IOLog("change volume on convertors");
			for (w = convertors; *w != NULL; w++)
			{
				inout = 0;
				if ((*w)->wcaps.in_amp_present && (streamDirection == kIOAudioStreamDirectionInput))
					inout |= AC_AMP_SET_INPUT;
				if ((*w)->wcaps.out_amp_present && (streamDirection == kIOAudioStreamDirectionOutput))
					inout |= AC_AMP_SET_OUTPUT;

				if (inout)
					(*w)->setAmplifierGainMute(lr | inout, 0, newValue);

				IOLog(" ...0x%x", (*w)->nid);
			}
			IOLog(" done\n");
		}

		if (mixers)
		{
			IOLog("change volume on mixers");
			for (w = mixers; *w != NULL; w++)
			{
				inout = 0;
				if ((*w)->wcaps.in_amp_present)
					inout |= AC_AMP_SET_INPUT;
				if ((*w)->wcaps.out_amp_present)
					inout |= AC_AMP_SET_OUTPUT;

				if (inout)
					(*w)->setAmplifierGainMute(lr | inout, 0, newValue);

				IOLog(" ...0x%x", (*w)->nid);
			}
			IOLog(" done\n");
		}

		if (pins)
		{
			IOLog("change volume on pins");
			for (w = pins; *w != NULL; w++)
			{
				inout = 0;
//				if ((*w)->wcaps.in_amp_present && (streamDirection == kIOAudioStreamDirectionInput))
//					inout |= AC_AMP_SET_INPUT;
				if ((*w)->wcaps.out_amp_present && (streamDirection == kIOAudioStreamDirectionOutput))
					inout |= AC_AMP_SET_OUTPUT;

				if (inout)
					(*w)->setAmplifierGainMute(lr | inout, 0, newValue);

/*				if ((*w)->wcaps.in_amp_present && (streamDirection == kIOAudioStreamDirectionInput))
				{
					if (newValue > AC_GAIN_MAX / 2)
						(*w)->setPinControl(AC_PINCTL_IN_EN | AC_PINCTL_VREF_80);
					else
						(*w)->setPinControl(AC_PINCTL_IN_EN | AC_PINCTL_VREF_50);
				}*/

				IOLog(" ...0x%x", (*w)->nid);
			}
			IOLog(" done\n");
		}
    }

    return kIOReturnSuccess;
}

IOReturn HDAIOEngine::muteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAIOEngine *audioEngine;
    
    audioEngine = (HDAIOEngine *)target;
    if (audioEngine) {
        result = audioEngine->muteChanged(muteControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAIOEngine::muteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAController[%p]::muteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
	HDAAudioWidget **w;
	unsigned muteParam;

	muteParam = 0;
    if (newValue)
		muteParam = AC_AMP_MUTE;

	if (pins)
	{
		IOLog("change mute on pins");
		for (w = pins; *w != NULL; w++)
		{
			if ((*w)->wcaps.in_amp_present && (streamDirection == kIOAudioStreamDirectionInput))
			{
				(*w)->setAmplifierGainMute(AC_AMP_SET_LEFT | AC_AMP_SET_INPUT, 0, muteParam);
				(*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_INPUT, 0, muteParam);
			}
			if ((*w)->wcaps.out_amp_present && (streamDirection == kIOAudioStreamDirectionOutput))
			{
				(*w)->setAmplifierGainMute(AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT, 0, muteParam);
				(*w)->setAmplifierGainMute(AC_AMP_SET_RIGHT | AC_AMP_SET_OUTPUT, 0, muteParam);
			}

			IOLog(" ...0x%x", (*w)->nid);
		}
		IOLog(" done\n");
	}

    return kIOReturnSuccess;
}

