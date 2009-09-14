/*
 *  HDAPCIRegisters.cpp
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 12/6/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDAPCIRegisters.h"

#include <IOKit/IOLib.h>

#define super OSObject

OSDefineMetaClassAndStructors(HDAPCIRegisters, OSObject)

bool HDAPCIRegisters::init(IOPCIDevice* device) {

	if (!super::init())
		return false;

	pciDevice = device;
	deviceMap = NULL;
	mutex = NULL;

	if (!pciDevice)
		return false;

    // Config a map for the PCI config base registers
    // We need to keep this map around until we're done accessing the registers
    deviceMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    if (!deviceMap)
        return false;

    IOLog("HDA PCI Regs map: Phys:%08x Virt:%08x len:%u\n",
		(unsigned) deviceMap->getPhysicalAddress(),
		(unsigned) deviceMap->getVirtualAddress(),
		(unsigned) deviceMap->getLength());

    // Get the virtual address for the registers - mapped in the kernel address space
    registers = (volatile void *)deviceMap->getVirtualAddress();
    if (!registers) {
		deviceMap->release();
		deviceMap = NULL;
        return false;
    }

    // Enable the PCI memory access - the kernel will panic if this isn't done before accessing the 
    // mapped registers
	pciDevice->setMemoryEnable(true);

	mutex = IOLockAlloc();
	if (!mutex) {
		deviceMap->release();
		deviceMap = NULL;
		return false;
	}

	return true;
}

HDAPCIRegisters *HDAPCIRegisters::withPCIDevice(IOPCIDevice *device) {

	HDAPCIRegisters *regs = new HDAPCIRegisters();

	if (!regs)
		return NULL;

	if (!regs->init(device)) {
		regs->release();
		return NULL;
	}
	
	return regs;
}

void HDAPCIRegisters::free() {

	if (deviceMap) {
		deviceMap->release();
		deviceMap = NULL;
	}

	if (mutex) {
		IOLockFree(mutex);
		mutex = NULL;
	}

	super::free();
}

void HDAPCIRegisters::lock() {
	IOLockLock(mutex);
}

void HDAPCIRegisters::unlock() {
	IOLockUnlock(mutex);
}

IOPCIDevice *HDAPCIRegisters::getDevice() {
	return pciDevice;
}

/*
 * ЧТО ЗДЕЬ ДЕЛАЕТСЯ, И ДОЛЖНО ЛИ ЭТО ДЕЛАТЬСЯ В HDAPCIRegisters?
 */
bool HDAPCIRegisters::initPCIConfigSpace()
{
	UInt16	reg16;
        
    reg16	= pciDevice->configRead16( kIOPCIConfigCommand );
    reg16  &= ~kIOPCICommandIOSpace;

    reg16	|= ( kIOPCICommandBusMaster
			|    kIOPCICommandMemorySpace
			|	 kIOPCICommandMemWrInvalidate );

	pciDevice->configWrite16( kIOPCIConfigCommand, reg16 );

	pciDevice->configWrite8( kIOPCIConfigCacheLineSize, 64 / sizeof( UInt32 ) );
	pciDevice->configWrite8( kIOPCIConfigLatencyTimer, 0xF8 );// max timer - low 3 bits ignored

    return true;
}
