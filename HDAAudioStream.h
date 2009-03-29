/*
 *  HDAAudioStream.h
 *  OpenHDAX
 *
 *  Created by Юрий Гагарин on 3/29/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include <IOKit/audio/IOAudioStream.h>
#include "dma.h"

class HDACodec;
class HDAController;

class HDAAudioStream : public IOAudioStream {

    OSDeclareDefaultStructors(HDAAudioStream)

protected:

	// components of stream
	HDADMABuffer					*streamBuffer;					// DMA ring buffer whith stream data, we own this link
	HDADMABuffer					*streamBufferDescriptor;		// BDLE description of streamBuffer, we own this link
	unsigned						baseAddr;						// base address of stream in PCI registers
	unsigned						streamTag;						// stream tag
	unsigned						nid;							// nid of our stream

	// controller and codec
	HDACodec						*codec;
	HDAController					*controller;

	// configuration
	IOAudioSampleRate				sampleRate;
	IOAudioStreamFormat				format;

public:

	virtual bool init(HDAController *controller, HDACodec *codec, unsigned nid);

	virtual void free();

};

