/*
 *  dma.cpp
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 11/25/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "dma.h"

#include <IOKit/IOLib.h>

/*
 * Physical 64-bit address routines
 */

PH64::PH64() {
	phaddr = 0;
}

PH64::PH64(UInt64 addr) {
	phaddr = addr;
}

PH64::PH64(const PH64& addr) {
	phaddr = addr.whole64();
}

PH64::~PH64() {
}

UInt32 PH64::low32() const {
	return phaddr & 0xffffffff;
}

UInt32 PH64::hi32() const {
	return (phaddr >> 32) & 0xffffffff;
}

UInt64 PH64::whole64() const {
	return phaddr;
}

PH64 PH64::offset(UInt64 offset) const {
	return phaddr + offset;
}


/*
 * DMA Buffer wrapper for private needs.
 */

#define super OSObject

OSDefineMetaClassAndStructors(HDADMABuffer, OSObject)

bool HDADMABuffer::init(unsigned int size, bool allow64bit) {

	if (!super::init())
		return false;

	bufferMemoryDescriptor = NULL;
	dmaCommand = NULL;
	phaddr = PH64();
	buf = NULL;
	
	return allocate(size, allow64bit);
}

HDADMABuffer *HDADMABuffer::withSize(unsigned int size, bool allow64bit) {

	HDADMABuffer *buffer = new HDADMABuffer;

	if (!buffer)
		return NULL;

	if (!buffer->init(size, allow64bit)) {
		buffer->release();
		return NULL;
	}

	return buffer;
}

bool HDADMABuffer::allocate(unsigned int size, bool allow64bit) {
	bufferMemoryDescriptor = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
				// task to hold the memory
				kernel_task, 
				// options
				kIOMemoryPhysicallyContiguous, 
				// size
				size, 
				allow64bit ?
				// physicalMask - 64 bit addressable and page aligned
				0xFFFFFFFFFFFFF000ULL :
				// physicalMask - 32 bit addressable and page aligned
				0x00000000FFFFF000ULL);
	if (!bufferMemoryDescriptor) {
		return false;
	}
	
	buf = bufferMemoryDescriptor->getBytesNoCopy();

	if (allow64bit) {
		dmaCommand = IODMACommand::withSpecification(
			// outSegFunc - Host endian since we read the address data with the cpu
			// and 64 bit wide quantities
			kIODMACommandOutputHost64, 
			// numAddressBits
			64, 
			// maxSegmentSize - zero for unrestricted physically contiguous chunks
			0,
			// mappingOptions - kMapped for DMA addresses
			IODMACommand::kMapped,
			// maxTransferSize - no restriction
			0,
			// alignment
			4096);
		dmaCommand->setMemoryDescriptor(bufferMemoryDescriptor);
		UInt64 offset = 0;
		// use the 64 bit variant to match outSegFunc
		IODMACommand::Segment64 segment;
		UInt32 numSeg = 1;
		// use the 64 bit variant to match outSegFunc
		dmaCommand->gen64IOVMSegments(&offset, &segment, &numSeg);
		IOLog("addr 0x%qx, len 0x%qx, nsegs %u\n",
				segment.fIOVMAddr, segment.fLength, numSeg);
		phaddr = PH64(segment.fIOVMAddr);
	} else {
		phaddr = PH64(bufferMemoryDescriptor->getPhysicalAddress());
	}

	bufferMemoryDescriptor->prepare(kIODirectionOutIn);
	
	return true;
}

void HDADMABuffer::free() {
	if (dmaCommand) {
		dmaCommand->clearMemoryDescriptor();
		dmaCommand->release();
		dmaCommand = NULL;
	}

	if (bufferMemoryDescriptor) {
		bufferMemoryDescriptor->complete();
		bufferMemoryDescriptor->release();
		bufferMemoryDescriptor = NULL;
	}

	super::free();
}

void* HDADMABuffer::getVirtualAddress() const {
	return buf;
}

PH64 HDADMABuffer::getPhysicalAddress() const {
	return phaddr;
}
