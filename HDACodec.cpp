/*
 * Intel High Definition Audio Device Driver for OS X
 *
 * Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 * Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 * was a great sample and probably start point for this driver.
 */

#include "HDACodec.h"

#include <IOKit/IOLib.h>

#include <IOKit/IOFilterInterruptEventSource.h>

//#define INITIAL_SAMPLE_RATE	44100
//#define NUM_SAMPLE_FRAMES	16384
//#define NUM_CHANNELS		2
//#define BIT_DEPTH			16

//#define BUFFER_SIZE			(NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)

#define super IOAudioEngine

OSDefineMetaClassAndStructors(HDACodec, IOAudioEngine)

/*
 * Lowlevel interface
 */

void HDACodec::powerUp() {
	// not yet implemented
}

void HDACodec::powerDown() {
	// not yet implemented
}

UInt32 HDACodec::paramRead(UInt16 nid, unsigned int param) {
	return codecRead(nid, 0, AC_VERB_PARAMETERS, param);
}


/**
 * Send a single command and read the corresponding response.
 *
 * Returns the obtained response value, or -1 for an error.
 */
UInt32 HDACodec::codecRead(UInt16 nid, int direct, unsigned int verb, unsigned int parm) {
	UInt32 result;

	powerUp();
	audioDevice->commandLock();
	if (audioDevice->sendCommand(codecAddress, nid, direct, verb, parm))
		result = audioDevice->getResponse();
	else
		result = (UInt32)-1;
	audioDevice->commandUnlock();
	powerDown();

	return result;
}

bool HDACodec::codecWrite(UInt16 nid, int direct, unsigned int verb, unsigned int parm) {
	bool result;
	
	powerUp();
	audioDevice->commandLock();
	result = audioDevice->sendCommand(codecAddress, nid, direct, verb, parm);
	audioDevice->commandUnlock();
	powerDown();

	return result;
}

bool HDACodec::init(HDAController *device, unsigned int addr)
{
    bool result = false;
    
    IOLog("HDACodec[%p]::init(%p, %u)\n", this, device, addr);

    if (!device) {
        goto Done;
    }

    if (!super::init(NULL)) {
        goto Done;
    }
    
    audioDevice = device;
	codecAddress = addr;
	wcaps = NULL;
    
    result = true;
    
Done:

    return result;
}

/**
 * getSubNodes - get the range of sub nodes
 * @nid: NID to parse
 * @startId: the pointer to store the start NID
 *
 * Parse the NID and store the start NID of its sub-nodes.
 * Returns the number of sub-nodes.
 */
int HDACodec::getSubNodes(UInt32 nid, UInt32 &startId)
{
	UInt32 parm;

	parm = paramRead(nid, AC_PAR_NODE_COUNT);
	if (parm == (UInt32)-1)
		return 0;
	startId = (parm >> 16) & 0x7fff;
	return (int)(parm & 0x7fff);
}

/*
 * look for an AFG and MFG nodes
 */
void HDACodec::setupFunctionGroupNodes()
{
	int i, totalNodes;
	UInt32 nid;

	totalNodes = getSubNodes(AC_NODE_ROOT, nid);
	IOLog("totalNodes = %d\n", totalNodes);
	for (i = 0; i < totalNodes; i++, nid++) {
		unsigned int func;
		func = paramRead(nid, AC_PAR_FUNCTION_TYPE);
		switch (func & 0xff) {
		case AC_GRP_AUDIO_FUNCTION:
			IOLog("AFG discovered nid=%u\n", (unsigned int) nid);
			afg = nid;
			break;
		case AC_GRP_MODEM_FUNCTION:
			IOLog("MFG discovered nid=%u\n", (unsigned int) nid);
			mfg = nid;
			break;
		default:
			break;
		}
	}
	/* interest only */
	int afgSubNodes = getSubNodes(afg, nid);
	IOLog("afg subnodes number %d start at %d\n", (int)afgSubNodes, (int)nid);
}

/*
 * read widget caps for each widget and store in cache
 */
bool HDACodec::readWidgetCaps(UInt32 fgNode)
{
	int i;
	UInt32 nid;

	numNodes = getSubNodes(fgNode, startNid);
	wcaps = (UInt32*)IOMalloc(sizeof(UInt32) * numNodes);
	if (!wcaps)
		return false;
	nid = startNid;
	for (i = 0; i < numNodes; i++, nid++)
		wcaps[i] = paramRead(nid, AC_PAR_AUDIO_WIDGET_CAP);
	return true;
}

void HDACodec::setupStream(UInt32 nid, UInt32 streamTag, int channelId, int format) {
	if (!nid)
		return;

	IOLog("setupStream: nid=0x%x, stream=0x%x, channel=%d, format=0x%x\n",
						(unsigned int)nid, (unsigned int)streamTag, channelId, format);

	codecWrite(nid, 0, AC_VERB_SET_CHANNEL_STREAMID, (streamTag << 4) | channelId);

	IOSleep(1);

	codecWrite(nid, 0, AC_VERB_SET_STREAM_FORMAT, format);
}

bool HDACodec::setPCMFormat(int dir, int format) {
	if (dir == AUDIO_PLAY) {
		return codecWrite(0x2, 0, AC_VERB_SET_STREAM_FORMAT, format);
	} else {
		return codecWrite(0x4, 0, AC_VERB_SET_STREAM_FORMAT, format);
	}
}

bool HDACodec::startPlayback(int stream) {
	UInt8 ctmp;
	unsigned int regbase;
	PH64 sbdPhysAddr;
	
	IOLockLock(audioDevice->mutex);
	if (audioDevice->flags & PLAY_STARTED) {
		IOLockUnlock(audioDevice->mutex);
		return true;
	}
	
	regbase = audioDevice->playbackRegistryBase;
	if (audioDevice->flags & PLAY_PAUSED) {
		ctmp = audioDevice->regsRead8(regbase + HDA_SD_CTL);
		audioDevice->flags |= PLAY_STARTED;
		audioDevice->flags &= ~PLAY_PAUSED;
		audioDevice->regsWrite8(regbase + HDA_SD_CTL, ctmp | SD_CTL_DMA_START);
		IOLockUnlock(audioDevice->mutex);
		return true;
	}
	
	if (!resetStream(audioDevice->inputStreams)) {
		IOLog("failed to reset play stream\n");
		IOLockUnlock(audioDevice->mutex);
		return false;
	}
	
	audioDevice->flags |= PLAY_STARTED;
	if (!fillPlaybackBuffer()) {
		IOLockUnlock(audioDevice->mutex);
		return false;
	}
	
	sbdPhysAddr = audioDevice->playbackBufferDescriptor->getPhysicalAddress();
	audioDevice->regsWrite32(regbase + HDA_SD_BDLPL, sbdPhysAddr.low32());
	audioDevice->regsWrite32(regbase + HDA_SD_BDLPU, sbdPhysAddr.hi32());
	audioDevice->regsWrite16(regbase + HDA_SD_LVI, HDA_BDLE_NUMS - 1);
	audioDevice->regsWrite32(regbase + HDA_SD_CBL, audioDevice->playbackBufferSize * HDA_BDLE_NUMS);
	
	audioDevice->regsWrite32(regbase + HDA_SD_FORMAT, audioDevice->playbackFormat);
	setPCMFormat(AUDIO_PLAY, audioDevice->playbackFormat);
	
	/* clear status */
	audioDevice->regsWrite8(regbase + HDA_SD_STS, SD_INT_DESC_ERR | SD_INT_FIFO_ERR | SD_INT_COMPLETE);
	
	/* set playback stream tag */
	audioDevice->regsWrite8(regbase + HDA_SD_CTL + 2, audioDevice->playbackStreamTag << 4 | 4);

	/* enable the position buffer */
	if (!audioDevice->regsRead32(HDA_DPLBASE) & DPLBASE_ENABLE) {
		audioDevice->regsWrite32(HDA_DPLBASE, audioDevice->positionBuffer->getPhysicalAddress().low32() | DPLBASE_ENABLE);
	}
	
	/* enable SIE */
	audioDevice->regsWrite8(HDA_INTCTL, audioDevice->regsRead8(HDA_INTCTL) | (1 << audioDevice->playbackStreamTag));

	/* start counting BDL entries */
	numberOfPlaybackBDLEPassed = 0;

	/* enable interrupt and start dma */
	audioDevice->regsWrite8(regbase + HDA_SD_CTL, SD_INT_MASK | SD_CTL_DMA_START);
	
	IOLockUnlock(audioDevice->mutex);
	return true;
}

bool HDACodec::resetStream(int stream) {
	unsigned int base;
	UInt8 btmp;
	int i;
	
	base = HDA_SD_BASE + HDA_SD_LEN * stream;
	btmp = audioDevice->regsRead8(base + HDA_SD_CTL);
	
	/* stop stream */
	btmp &= ~HDA_RIRBSIZE;
	audioDevice->regsWrite8(base + HDA_SD_CTL, btmp);
	
	/* wait 40us for stream to stop as HD spec */
	IODelay(40);
	
	/* reset stream */
	btmp |= SD_CTL_STREAM_RESET;
	audioDevice->regsWrite8(base + HDA_SD_CTL, btmp);
	
	for (i = 0; i < 50; i++) {
		IODelay(10);
		btmp = audioDevice->regsRead8(base + HDA_SD_CTL);
		btmp &= SD_CTL_STREAM_RESET;
		if (btmp)
			break;
	}
	
	if (!btmp) {
		IOLog("Failed to reset stream %d\n", stream);
		return false;
	}
	
	/* Need any RESET# assertion time, 300us ??? */
	IODelay(50);
	
	/* exit reset stream */
	btmp &= ~SD_CTL_STREAM_RESET;
	audioDevice->regsWrite8(base + HDA_SD_CTL, btmp);
	
	for (i = 0; i < 50; i++) {
		IODelay(10);
		btmp = audioDevice->regsRead8(base + HDA_SD_CTL);
		btmp &= SD_CTL_STREAM_RESET;
		if (!btmp)
			break;
	}
	
	if (btmp) {
		IOLog("Failed to exit reset state for stream %d, btmp=0x%02x\n", stream, (int)btmp);
		return false;
	}
	
	return true;
}

bool HDACodec::fillPlaybackBuffer() {
	UInt64 bufPhysAddr;
	BDLEntry *entry;
	char *buf;
	int samples;
	int rs;
	int i;
	
	entry = (BDLEntry*)audioDevice->playbackBufferDescriptor->getVirtualAddress();
	buf = (char*)audioDevice->playbackBuffer->getVirtualAddress();
	bufPhysAddr = audioDevice->playbackBuffer->getPhysicalAddress().whole64();
	
	/* assume that 2-channel, 16-bit */
	samples = audioDevice->playbackBufferSize * HDA_BDLE_NUMS / 2;
	
	IOLockUnlock(audioDevice->mutex);
	// temporary line
	rs = samples;
	//rs = am_get_audio(statep->hda_handle, buf, AUDIO_NO_CHANNEL, samples);
	IOLockLock(audioDevice->mutex);
	
	/* if we cannot get sample */
	if (rs <= 0) {
		IOLog("fillPlaybackBuffer() failed to get play sample\n");
		audioDevice->flags &= ~PLAY_STARTED;
	}

	/*
	 * Because users can quickly start and stop audio streams, it
	 * is possible that playback already stopped before we re-grab
	 * mutex. In this case, we dispose fetched samples, and return
	 * AUDIO_FAILURE as we didn't get samples.
	 */
	if ((audioDevice->flags & PLAY_STARTED) == 0) {
		return false;
	}

	for (i = 0; i < HDA_BDLE_NUMS; i++) {
		entry->addr = bufPhysAddr;
		entry->len = audioDevice->playbackBufferSize;
		entry->ioc = 1;
		bufPhysAddr += audioDevice->playbackBufferSize;
		entry++;
	}

	//	(void) ddi_dma_sync(statep->hda_dma_play_bd.ad_dmahdl, 0,
	//    sizeof (sd_bdle_t) * AUDIOHD_BDLE_NUMS, DDI_DMA_SYNC_FORDEV);

	if (rs == samples)
		audioDevice->playbackBufferPosition = 0;
	else
		audioDevice->playbackBufferPosition = (rs << 1);

	return true;
}

bool HDACodec::stopPlayback(int stream) {
	unsigned int regbase;

	IOLockLock(audioDevice->mutex);
	regbase = audioDevice->playbackRegistryBase;
	audioDevice->regsWrite8(regbase + HDA_SD_CTL, 0);
	audioDevice->flags &= ~(PLAY_EMPTY | PLAY_STARTED);
	IOLockUnlock(audioDevice->mutex);
	
	return true;
}

void HDACodec::getMaxGain(unsigned int &pgain, unsigned int &rgain) {
	const int nidp = 0x08;
	const int nidr = 0x04;
	
	unsigned int ltmp;
	
	ltmp = paramRead(nidp, AC_PAR_AMP_OUT_CAP);
	pgain = (ltmp & AC_AMPCAP_NUM_STEPS) >> 8;
	ltmp = paramRead(nidr, AC_PAR_AMP_IN_CAP);
	rgain = (ltmp & AC_AMPCAP_NUM_STEPS) >> 8;
}

bool HDACodec::setGain(int dir, int gain, int channel) {
	unsigned int val;

	if (dir & AUDIO_PLAY) {
		val = AC_AMP_SET_OUTPUT | gain;
		if (channel == 0) {
			/* left channel */
			val |= AC_AMP_SET_LEFT;
			audioDevice->playbackLeftGain = gain;
		} else {
			/* right channel */
			val |= AC_AMP_SET_RIGHT;
			audioDevice->playbackRightGain = gain;
		}
		codecWrite(0x8, 0, AC_VERB_SET_AMP_GAIN_MUTE, val);
		codecWrite(0x9, 0, AC_VERB_SET_AMP_GAIN_MUTE, val);
	}
	if (dir & AUDIO_RECORD) {
		val = AC_AMP_SET_INPUT | gain;
		if (channel == 0) {
			/* left channel */
			val |= AC_AMP_SET_LEFT;
			audioDevice->recordLeftGain = gain;
		} else {
			/* right channel */
			val |= AC_AMP_SET_RIGHT;
			audioDevice->recordRightGain = gain;
		}
		codecWrite(0x4, 0, AC_VERB_SET_AMP_GAIN_MUTE, val);
	}
	return true;
}

bool HDACodec::muteOutputs(bool mute) {
	unsigned int val;
	
	if (audioDevice->outputsMuted == mute) {
		return true;
	}
	
	audioDevice->outputsMuted = mute;
	val = AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT;
	if (mute) {
		val |= AC_AMP_MUTE;
	}

	codecWrite(0x10, 0, AC_VERB_SET_AMP_GAIN_MUTE, val);
	codecWrite(0x0f, 0, AC_VERB_SET_AMP_GAIN_MUTE, val);
	
	return true;
}

/*
 * This is used to initialize DAC node of CODEC
 */
bool HDACodec::initDACNode(UInt32 nid) {
	if (codecWrite(nid, 0, AC_VERB_SET_CHANNEL_STREAMID, audioDevice->playbackStreamTag << 4))
		return codecWrite(nid, 0, AC_VERB_SET_AMP_GAIN_MUTE, AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT | AC_GAIN_MAX);

	return false;
}

/*
 * unmute specified one of a mixer's inputs, and set the
 * left & right output volume of mixer to specified value
 */
bool HDACodec::initMixerNode(UInt32 nid, int indexNum) {
	/* unmute input of mixer */
	if (codecWrite(nid, 0, AC_VERB_SET_AMP_GAIN_MUTE,
				AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_INPUT | AC_GAIN_MAX | 
				(indexNum << AC_AMP_SET_INDEX_SHIFT))) {

		/* aoutput left amp of mixer */
		codecWrite(nid, 0, AC_VERB_SET_AMP_GAIN_MUTE, AC_AMP_SET_LEFT | AC_AMP_SET_OUTPUT | audioDevice->playbackLeftGain);

		/* aoutput right amp of mixer */
		codecWrite(nid, 0, AC_VERB_SET_AMP_GAIN_MUTE, AC_AMP_SET_RIGHT | AC_AMP_SET_OUTPUT | audioDevice->playbackRightGain);

		return true;
	}

	return false;
}

/*
 * unmute an output pin
 */
bool HDACodec::unmuteOutputNode(UInt32 nid) {
	return codecWrite(nid, 0, AC_VERB_SET_AMP_GAIN_MUTE, 
			AC_AMP_SET_LEFT | AC_AMP_SET_RIGHT | AC_AMP_SET_OUTPUT | AC_GAIN_MAX);
}

bool HDACodec::enablePinOutNode(UInt32 nid) {
	UInt32 ltmp;

	ltmp = codecRead(nid, 0, AC_VERB_GET_PIN_WIDGET_CONTROL, 0);
	if (ltmp == (UInt32)-1)
		return false;

	if (!codecWrite(nid, 0, AC_VERB_SET_PIN_WIDGET_CONTROL, ltmp | AC_PINCTL_OUT_EN | AC_PINCTL_HP_EN))
		return false;

	return true;
}

bool HDACodec::disablePinOutNode(UInt32 nid) {
	UInt32 ltmp;

	ltmp = codecRead(nid, 0, AC_VERB_GET_PIN_WIDGET_CONTROL, 0);
	if (ltmp == (UInt32)-1)
		return false;

	if (!codecWrite(nid, 0, AC_VERB_SET_PIN_WIDGET_CONTROL, ltmp & ~AC_PINCTL_OUT_EN))
		return false;

	return true;
}

/*
 * setFormat()
 *
 * Description
 *	currently, only 48k sample rate, 16-bit precision,
 *	2-channel format is supported.
 */
bool HDACodec::setFormat(int stream, int dir, int sample_rate, int channels, int precision, int encoding) {
	/*
	 * Currently, force to 48k, 16bits, 2-channel
	 */
	if ((sample_rate != HDA_SAMPR48000) ||
	    (channels != AUDIO_CHANNELS_STEREO) ||
	    (precision != AUDIO_PRECISION_16) ||
	    (encoding != AUDIO_ENCODING_LINEAR))
		return false;

	/*
	 * we will support other format later
	 */
	IOLockLock(audioDevice->mutex);

	if (dir == AUDIO_PLAY) {
		audioDevice->psampleRate = sample_rate;
		audioDevice->pchannels = channels;
		audioDevice->pprecision = precision;
		audioDevice->playbackFormat = HDA_FMT_PCMOUT;
	} else {
		audioDevice->csampleRate = sample_rate;
		audioDevice->cchannels = channels;
		audioDevice->cprecision = precision;
		audioDevice->recordFormat = HDA_FMT_PCMIN;
	}

	IOLockUnlock(audioDevice->mutex);
	return true;
}

bool HDACodec::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOWorkLoop *workLoop;
    
    IOLog("HDACodec[%p]::initHardware(%p)\n", this, provider);
	nitrs = 0;
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // Setup the initial sample rate for the audio engine
    initialSampleRate.whole = HDA_SAMPR48000;
    initialSampleRate.fraction = 0;
    
	
	
	
	
	
	
	
	
	vendorId = paramRead(AC_NODE_ROOT, AC_PAR_VENDOR_ID);
	IOLog("codec vendor ID = 0x%x\n", (unsigned int)vendorId);
	subsystemId = paramRead(AC_NODE_ROOT, AC_PAR_SUBSYSTEM_ID);
	IOLog("codec subsystem ID = 0x%x\n", (unsigned int)subsystemId);
	revisionId = paramRead(AC_NODE_ROOT, AC_PAR_REV_ID);
	IOLog("codec revision ID = 0x%x\n", (unsigned int)revisionId);
	
	setupFunctionGroupNodes();
	if (!afg && !mfg) {
		IOLog("HDACodec[%p]::initHardware(%p): no AFG or MFG node found\n", this, provider);
		return false;
	}
	
	if (!readWidgetCaps(afg ? afg : mfg)) {
		IOLog("HDACodec[%p]::initHardware(%p): readWidgetCaps error\n", this, provider);
		return false;
	}

	if (!subsystemId) {
		UInt32 nid = afg ? afg : mfg;
		subsystemId = codecRead(nid, 0, AC_VERB_GET_SUBSYSTEM_ID, 0);
		IOLog("codec FG subsystem ID = 0x%x\n", (unsigned int)subsystemId);
	}
	
	
	/* this code from audiohd.c/init_codec need some other place probably */
	/* ALC260 specific code */
	/* initialize codec output */
	if (initDACNode(2))
		IOLog("init DAC node 2 ok\n");
	if (initMixerNode(0x8, 0))
		IOLog("init mixer node 0x8 ok\n");
	if (initMixerNode(0x9, 0))
		IOLog("init mixer node 0x9 ok\n");
	if (!audioDevice->outputsMuted) {
		IOLog("outputs not muted\n");
		if (unmuteOutputNode(0x10))
			IOLog("output node 0x10 unmuted\n");
		if (unmuteOutputNode(0x0f))
			IOLog("output node 0x0f unmuted\n");
	}
	/* HEADPHONE */
	if (enablePinOutNode(0x10))
		IOLog("headphone enabled\n");
	/* LINE_OUT */
	if (enablePinOutNode(0x0f))
		IOLog("line out enabled\n");
	getMaxGain(audioDevice->playbackGainMax, audioDevice->recordGainMax);
	setGain(AUDIO_PLAY, audioDevice->playbackGainMax, 0);
	setGain(AUDIO_PLAY, audioDevice->playbackGainMax, 1);
	setGain(AUDIO_RECORD, audioDevice->recordGainMax, 0);
	setGain(AUDIO_RECORD, audioDevice->recordGainMax, 1);
	/* no input initialized yet */
	/* end of ALC260 specific code */
	

    setDescription("Realtek ALC260");
    
    setSampleRate(&initialSampleRate);
    
    // Set the number of sample frames in each buffer
    setNumSampleFramesPerBuffer(audioDevice->playbackBufferSize / HDA_MAX_CHANNELS * 8 / HDA_MAX_PRECISION * HDA_BDLE_NUMS);
    
    workLoop = getWorkLoop();
    if (!workLoop) {
        goto Done;
    }
    
    // Create an interrupt event source through which to receive interrupt callbacks
    // In this case, we only want to do work at primary interrupt time, so
    // we create an IOFilterInterruptEventSource which makes a filtering call
    // from the primary interrupt interrupt who's purpose is to determine if
    // our secondary interrupt handler is to be called.  In our case, we
    // can do the work in the filter routine and then return false to
    // indicate that we do not want our secondary handler called
//    interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this, 
//                                    HDACodec::interruptHandler, 
//                                    HDACodec::interruptFilter,
//                                    audioDevice->getProvider(), 1);
//    if (!interruptEventSource) {
//        goto Done;
//    }
    
    // In order to allow the interrupts to be received, the interrupt event source must be
    // added to the IOWorkLoop
    // Additionally, interrupts will not be firing until the interrupt event source is 
    // enabled by calling interruptEventSource->enable() - this probably doesn't need to
    // be done until performAudioEngineStart() is called, and can probably be disabled
    // when performAudioEngineStop() is called and the audio engine is no longer running
    // Although this really depends on the specific hardware 
//    workLoop->addEventSource(interruptEventSource);

//	interruptEventSource->enable();

    // Allocate our input and output buffers - a real driver will likely need to allocate its buffers
    // differently
    outputBuffer = (SInt16 *)(audioDevice->playbackBuffer->getVirtualAddress());
    if (!outputBuffer) {
        goto Done;
    }
    
//    inputBuffer = (SInt16 *)IOMalloc(BUFFER_SIZE);
//    if (!inputBuffer) {
//        goto Done;
//    }
    
	outputAudioStream = NULL;

    // Create an IOAudioStream for each buffer and add it to this audio engine
    outputAudioStream = createNewAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, audioDevice->playbackBufferSize * HDA_BDLE_NUMS);
    if (!outputAudioStream) {
        goto Done;
    }
    
    addAudioStream(outputAudioStream);
    outputAudioStream->release();
    
//    audioStream = createNewAudioStream(kIOAudioStreamDirectionInput, inputBuffer, BUFFER_SIZE);
//    if (!audioStream) {
//        goto Done;
//    }
    
//    addAudioStream(audioStream);
//    audioStream->release();
    
    result = true;
    
Done:

    return result;
}

void HDACodec::free()
{
    IOLog("HDACodec[%p]::free()\n", this);
	
	IOLog("nitrs=%d\n", nitrs);
    
    // We need to free our resources when we're going away
    
	if (outputAudioStream) {
		outputAudioStream->release();
		outputAudioStream = NULL;
	}
	
//    if (interruptEventSource) {
//		interruptEventSource->disable();
//        interruptEventSource->release();
//        interruptEventSource = NULL;
//    }
    
    if (outputBuffer) {
        outputBuffer = NULL;
    }
    
//    if (inputBuffer) {
//        IOFree(inputBuffer, BUFFER_SIZE);
//        inputBuffer = NULL;
//    }
	
	if (wcaps) {
		IOFree(wcaps, sizeof(UInt32) * numNodes);
		wcaps = NULL;
	}
    
    super::free();
}

IOAudioStream *HDACodec::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
{
    IOAudioStream *audioStream;
    
    // For this sample device, we are only creating a single format and allowing 44.1KHz and 48KHz
    audioStream = new IOAudioStream;
    if (audioStream) {
        if (!audioStream->initWithAudioEngine(this, direction, 1)) {
            audioStream->release();
        } else {
            IOAudioSampleRate rate;
            IOAudioStreamFormat format = {
                2,												// num channels
                kIOAudioStreamSampleFormatLinearPCM,			// sample format
                kIOAudioStreamNumericRepresentationSignedInt,	// numeric format
                HDA_MAX_PRECISION,								// bit depth
                HDA_MAX_PRECISION,								// bit width
                kIOAudioStreamAlignmentHighByte,				// high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,			// little endian
                true,											// format is mixable
                0												// driver-defined tag - unused by this driver
            };
            
            // As part of creating a new IOAudioStream, its sample buffer needs to be set
            // It will automatically create a mix buffer should it be needed
            audioStream->setSampleBuffer(sampleBuffer, sampleBufferSize);
            
            // This device only allows a single format and a choice of 2 different sample rates
//            rate.fraction = 0;
//            rate.whole = 44100;
//            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.fraction = 0;
            rate.whole = 48000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            
            // Finally, the IOAudioStream's current format needs to be indicated
            audioStream->setFormat(&format);
        }
    }
    
    return audioStream;
}

void HDACodec::stop(IOService *provider)
{
    IOLog("HDACodec[%p]::stop(%p)\n", this, provider);
    
    // When our device is being stopped and torn down, we should go ahead and remove
    // the interrupt event source from the IOWorkLoop
    // Additionally, we'll go ahead and release the interrupt event source since it isn't
    // needed any more
//    if (interruptEventSource) {
//        IOWorkLoop *wl;
//        
//		interruptEventSource->disable();
//        wl = getWorkLoop();
//        if (wl) {
//            wl->removeEventSource(interruptEventSource);
//        }
//        
//        interruptEventSource->release();
//        interruptEventSource = NULL;
//    }

//	outputAudioStream->setStreamAvailable(false);
//	outputAudioStream->stop(provider);
	
    // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
    // There may be nothing needed here

    super::stop(provider);
}
    
IOReturn HDACodec::performAudioEngineStart()
{
    IOLog("HDACodec[%p]::performAudioEngineStart()\n", this);
    
    // The interruptEventSource needs to be enabled to allow interrupts to start firing
//    assert(interruptEventSource);
//    interruptEventSource->enable();

	if (!startPlayback(audioDevice->inputStreams)) {
		IOLog("start playback failed\n");
	}
	else
		IOLog("playback started !!!\n");

    // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
    // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
    // to indicate when that sample is being read from/written to.  The function takeTimeStamp() 
    // is provided to do that automatically with the current time.
    // By default takeTimeStamp() will increment the current loop count in addition to taking the current
    // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
    // to be incremented.  To accomplish that, false is passed to takeTimeStamp(). 
    takeTimeStamp(false);
    
    // Add audio - I/O start code here
    
//#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn HDACodec::performAudioEngineStop()
{
    IOLog("HDACodec[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
//    assert(interruptEventSource);
//    interruptEventSource->disable();

    // Add audio - I/O stop code here
    
//#error performAudioEngineStop() - driver will not work until audio engine stop code is added
	if (!stopPlayback(audioDevice->inputStreams)) {
		IOLog("stop playback failed\n");
	}
	else
		IOLog("playback stopped !!!\n");

    return kIOReturnSuccess;
}

unsigned int HDACodec::getCurrentSampleFrame(int stream) {

	unsigned int pos, regbase;

	regbase = audioDevice->playbackRegistryBase;
	pos = audioDevice->regsRead32(regbase + HDA_SD_LPIB);
	
	return pos;

/*	unsigned int pos;
	
	pos = OSReadLittleInt32(audioDevice->positionBuffer.buf, stream * 4);
	
	return pos;*/
}
    
UInt32 HDACodec::getCurrentSampleFrame()
{
    
    // In order for the erase process to run properly, this function must return the current location of
    // the audio engine - basically a sample counter
    // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
    // rather than after the current location.  The erase head will erase up to, but not including the sample
    // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be 
    // erased.
    
//#error getCurrentSampleFrame() - driver will not work until correct sample frame is returned

	unsigned int pos;

//	static int counter = 0;
//	++counter;

	pos = getCurrentSampleFrame(audioDevice->inputStreams);
//	if (counter == 200) {
//		IOLog("HDACodec[%p]::getCurrentSampleFrame()\n", this);
//		IOLog("sample frame = %u\n", pos);
//		counter = 0;
//	}

	if (pos > (unsigned int)audioDevice->playbackBufferSize * HDA_BDLE_NUMS)
		pos = 0;
	pos /= HDA_MAX_CHANNELS * HDA_MAX_PRECISION / 8;


    return pos;
}
    
IOReturn HDACodec::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("HDACodec[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                IOLog("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                IOLog("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                IOLog("/t Internal Error - unknown sample rate selected.\n");
                break;
        }
    }
    
    return kIOReturnSuccess;
}


void HDACodec::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop

    return;
}

bool HDACodec::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    HDACodec *audioEngine = OSDynamicCast(HDACodec, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }

    return false;
}

void HDACodec::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()

}

