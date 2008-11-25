/*
 *  dma.h
 *
 * Simple IOKit DMA wrapper for 32-bit Intel OS X with 64-bit buffer addresses support.
 *
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 11/25/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __OPENHDAX_DMA__
#define __OPENHDAX_DMA__ 1

#include <IOKit/IOTypes.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

/*
 * PH64 - representation of the 64bit physical address
 */
class PH64 {

	UInt64 phaddr;

public:
	PH64();
	PH64(UInt64 addr);
	PH64(const PH64& addr);
	virtual ~PH64();
	virtual UInt32 low32() const;
	virtual UInt32 hi32() const;
	virtual UInt64 whole64() const;
	virtual PH64 offset(UInt64 offset) const;
};

class DMABuffer {

	IOBufferMemoryDescriptor *bufferMemoryDescriptor;	// buffer which we allocating
	IODMACommand *dmaCommand;							// we need this for 64bit allocating
	PH64 phaddr;										// physical address
	void *buf;											// virtual address

public:

	DMABuffer();

	virtual bool allocate(unsigned int size, bool allow64bit = false);
	virtual void free();

	virtual void *getVirtualAddress() const;
	virtual PH64 getPhysicalAddress() const;
};

#endif
