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

class HDAPCIRegisters : public OSObject {

	OSDeclareDefaultStructors(HDAPCIRegisters)

	IOPCIDevice *pciDevice;
	IOMemoryMap *deviceMap;
	IOSimpleLock *spinlock;
	volatile void *registers;

public:

	virtual bool init(IOPCIDevice *device);
	static HDAPCIRegisters *withPCIDevice(IOPCIDevice *device);
	
	virtual void free();

	virtual bool initPCIConfigSpace();

	virtual IOInterruptState lock();
	virtual void unlock(IOInterruptState state);

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
