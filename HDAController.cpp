/*
 * Intel High Definition Audio Device Driver for OS X
 *
 * Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 * Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 * was a great sample and probably start point for this driver.
 */

#include "HDAController.h"
#include "HDACodec.h"

#include <IOKit/audio/IOAudioControl.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioDefines.h>

#include <IOKit/IOLib.h>

#include <IOKit/pci/IOPCIDevice.h>

#define CORB_DELAY			10
#define CORB_LOOPS			1000

#define super IOAudioDevice

OSDefineMetaClassAndStructors(HDAController, IOAudioDevice)

/*
 * CORB and RIRB interface
 */
bool HDAController::allocateRingBuffers() {
	if (!commandBuffer.allocate(4096, true))
		return false;
	corb.phaddr = commandBuffer.getPhysicalAddress();
	corb.buf = (UInt32*)commandBuffer.getVirtualAddress();
	rirb.phaddr = corb.phaddr.offset(2048);
	rirb.buf = corb.buf + 512;

	IOLog("HDAController[%p]::allocateRingBuffers() corb: phaddr=%08qx, buf=%p\n", this, corb.phaddr.whole64(), corb.buf);
	IOLog("HDAController[%p]::allocateRingBuffers() rirb: phaddr=%08qx, buf=%p\n", this, rirb.phaddr.whole64(), rirb.buf);
	
	return true;
}

void HDAController::freeRingBuffers() {
	commandBuffer.free();
}

bool HDAController::allocatePlaybackBuffers() {
	IOLog("allocatePlaybackBuffers: size = %d, dma64ok=%d\n", (int)playbackBufferSize * HDA_BDLE_NUMS, (int)dma64ok);

	/* allocate DMA for data buffer of playback stream */
	if (!playbackBuffer.allocate(playbackBufferSize * HDA_BDLE_NUMS, dma64ok))
		return false;

//	for (int i = 0; i < playbackBufferSize * HDA_BDLE_NUMS; i++) {
//		((char*)playbackBuffer.buf)[i] = (char)i;
//	}

	/* allocate DMA for buffer descriptor list of playback stream */
	if (!playbackBufferDescriptor.allocate(sizeof(BDLEntry) * HDA_BDLE_NUMS))
		return false;

	return true;
}

bool HDAController::allocateRecordBuffers() {
	/* allocate DMA for data buffer of record stream */
	if (!recordBuffer.allocate(recordBufferSize * HDA_BDLE_NUMS, dma64ok))
		return false;

	/* allocate DMA for buffer descriptor list of record stream */
	if (!recordBufferDescriptor.allocate(sizeof(BDLEntry) * HDA_BDLE_NUMS))
		return false;

	return true;
}

void HDAController::freePlaybackBuffers() {
	playbackBuffer.free();
	playbackBufferDescriptor.free();
}

void HDAController::freeRecordBuffers() {
	recordBuffer.free();
	recordBufferDescriptor.free();
}

bool HDAController::allocatePositionBuffer() {
	if (!positionBuffer.allocate(numStreams * 8, dma64ok))
		return false;
	return true;
}

void HDAController::freePositionBuffer() {
	positionBuffer.free();
}

// инициализация CORB и RIRB
bool HDAController::initCommandIO() {
	IOLog("HDAController[%p]::initCommandIO()\n", this);

	/* Allocate buffers */
	allocateRingBuffers();

	isSingleCommand = false;

	/* CORB set up */
	regsWrite32(HDA_CORBLBASE, corb.phaddr.low32());
	regsWrite32(HDA_CORBUBASE, corb.phaddr.hi32());

	/* set the corb size to 256 entries (ULI requires explicitly) */
	regsWrite8(HDA_CORBSIZE, 0x02);
	/* set the corb write pointer to 0 */
	regsWrite16(HDA_CORBWP, 0);
	/* reset the corb hw read pointer */
	regsWrite16(HDA_CORBRP, RBRWP_CLR);
	/* enable corb dma */
	regsWrite8(HDA_CORBCTL, RBCTL_DMA_EN);

	/* RIRB set up */
	regsWrite32(HDA_RIRBLBASE, rirb.phaddr.low32());
	regsWrite32(HDA_RIRBUBASE, rirb.phaddr.hi32());

	/* set the rirb size to 256 entries (ULI requires explicitly) */
	regsWrite8(HDA_RIRBSIZE, 0x02);
	/* reset the rirb hw write pointer */
	regsWrite16(HDA_RIRBWP, RBRWP_CLR);
	/* set N=1, get RIRB response interrupt for new entry */
	regsWrite16(HDA_RINTCNT, 1);
	/* enable rirb dma and response irq */
	regsWrite8(HDA_RIRBCTL, RBCTL_DMA_EN | RBCTL_IRQ_EN);
	rirb.rp = rirb.cmds = 0;

	return true;
}

void HDAController::disableRingBuffers() {
	/* disable ringbuffer DMAs */
	regsWrite8(HDA_RIRBCTL, 0);
	regsWrite8(HDA_CORBCTL, 0);
}

/* send a command */
bool HDAController::corbSendCommand(UInt32 val) {
	unsigned int wp;
	
	/* add command to corb */
	wp = regsRead8(HDA_CORBWP);
	wp++;
	wp %= MAX_CORB_ENTRIES;
	
	IOInterruptState state = IOSimpleLockLockDisableInterrupt(regsSpinLock);
	rirb.cmds++;
//	IOLog("buf=%p,wp=%u\n", corb.buf, wp);
	OSWriteLittleInt32(corb.buf, wp * sizeof(UInt32), val);
	regsWrite32(HDA_CORBWP, wp);
	IOSimpleLockUnlockEnableInterrupt(regsSpinLock, state);
	
	return true;
}

/* retreive RIRB entry - called from interrupt handler */
void HDAController::updateRirb() {
	unsigned int rp, wp;
	UInt32 res, res_ex;

	wp = regsRead8(HDA_RIRBWP);
	if (wp == rirb.wp)
		return;
	rirb.wp = wp;
	
	while (rirb.rp != wp) {
		rirb.rp++;
		rirb.rp %= MAX_RIRB_ENTRIES;

		rp = rirb.rp << 1; /* an RIRB entry is 8-bytes */
		res_ex = OSReadLittleInt32(rirb.buf, (rp + 1) * sizeof(UInt32));
		res = OSReadLittleInt32(rirb.buf, rp * sizeof(UInt32));
		if (res_ex & RIRB_EX_UNSOL_EV) {
			// TODO: process unsolicited event
			//snd_hda_queue_unsol_event(chip->bus, res, res_ex);
			++unsolicited;
		}
		else if (rirb.cmds) {
			rirb.cmds--;
			rirb.res = res;
		}
	}
}

/* receive a response */
UInt32 HDAController::rirbGetResponse() {
	IOInterruptState interruptState;
	static bool pollingMode = false;
	int timeout = 100;

//	IOLog("HDAController[%p]::rirbGetResponse()\n", this);

	for ( ; ; ) {
		while (timeout--) {
			if (pollingMode) {
				interruptState = IOSimpleLockLockDisableInterrupt(regsSpinLock);
				updateRirb();
				IOSimpleLockUnlockEnableInterrupt(regsSpinLock, interruptState);
			}
			if (!rirb.cmds)
				return rirb.res;	/* the last value */
			IOSleep(2);
		}
		if (!pollingMode) {
			IOLog("HDAController[%p]::rirbGetResponse() timeout. switching to polling mode\n", this);
			pollingMode = true;
			continue;				// try again
		}
		break;
	}

	IOLog("HDAController[%p]::rirbGetResponse() timeout switching to singleCommand mode\n", this);
	
	rirb.rp = regsRead8(HDA_RIRBWP);
	rirb.cmds = 0;
	isSingleCommand = true;

	return (UInt32)-1;
}

/*
 * Use the single immediate command instead of CORB/RIRB for simplicity
 *
 * Note: according to Intel, this is not preferred use.  The command was
 *       intended for the BIOS only, and may get confused with unsolicited
 *       responses.  So, we shouldn't use it for normal operation from the
 *       driver.
 *       I left the codes, however, for debugging/testing purposes.
 */

/* send a command */
bool HDAController::singleSendCommand(UInt32 val) {
	int timeout = 50;
	
//	IOLog("HDAController[%p]::singleSendCommand(%x)\n", this, (unsigned int)val);

	while (timeout--) {
		/* check ICB busy bit */
		if (!(regsRead16(HDA_IRS) & IRS_BUSY)) {
			/* Clear IRV valid bit */
			regsWrite16(HDA_IRS, regsRead16(HDA_IRS) | IRS_VALID);
			regsWrite32(HDA_IC, val);
			regsWrite16(HDA_IRS, regsRead16(HDA_IRS) | IRS_BUSY);
			return true;
		}
		IODelay(1);
	}
	IOLog("HDAController[%p]::singleSendCommand(%x) timeout", this, (unsigned int)val);

	return false;
}

/* receive a response */
UInt32 HDAController::singleGetResponse() {
	int timeout = 50;

//	IOLog("HDAController[%p]::singleGetResponse()\n", this);
	while (timeout--) {
		/* check IRV busy bit */
		if (regsRead8(HDA_IRS) & IRS_VALID)
			return regsRead32(HDA_IR);
		IODelay(1);
	}
	IOLog("HDAController[%p]::singleGetResponse() failed\n", this);

	return (UInt32)-1;
}

/*
 * The below are the main callbacks from hda_codec.
 *
 * They are just the skeleton to call sub-callbacks according to the
 * current setting of chip->single_cmd.
 */

/* send a command */
bool HDAController::sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para) {
	UInt32 val;

	val = (UInt32)(addr & 0x0f) << 28;
	val |= (UInt32)direct << 27;
	val |= (UInt32)nid << 20;
	val |= verb << 8;
	val |= para & 0xffff;

	if (isSingleCommand)
		return singleSendCommand(val);
	return corbSendCommand(val);
}

/* get a response */
UInt32 HDAController::getResponse() {
	if (isSingleCommand)
		return singleGetResponse();
	return rirbGetResponse();
}

/* Смотрим Lowlevel Interface */

bool HDAController::initHardware(IOService *provider)
{
    bool result = false;
    
    IOLog("HDAController[%p]::initHardware(%p)\n", this, provider);
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // Get the PCI device provider
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if (!pciDevice) {
        goto Done;
    }
    
    // Config a map for the PCI config base registers
    // We need to keep this map around until we're done accessing the registers
    deviceMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
    if (!deviceMap) {
        goto Done;
    }

    IOLog("map: Phys:%08x Virt:%08x len:%u\n",
		(unsigned) deviceMap->getPhysicalAddress(),
		(unsigned) deviceMap->getVirtualAddress(),
		(unsigned) deviceMap->getLength());

    // Get the virtual address for the registers - mapped in the kernel address space
    deviceRegisters = (Registers)deviceMap->getVirtualAddress();
    if (!deviceRegisters) {
        goto Done;
    }
    
    // Enable the PCI memory access - the kernel will panic if this isn't done before accessing the 
    // mapped registers
    pciDevice->setMemoryEnable(true);

    // add the hardware init code here
	initPCIConfigSpace(pciDevice);
    
    setDeviceName("Sample PCI Audio Device");
    setDeviceShortName("PCIAudio");
    setManufacturerName("My Company");

//#error Put your own hardware initialization code here...and in other routines!!

	if (!initHDA()) {
		goto Done;
	}
	
    if (!createAudioEngine()) {
        goto Done;
    }

	audioEngine->setFormat(0, AUDIO_PLAY, HDA_SAMPR48000, AUDIO_CHANNELS_STEREO, AUDIO_PRECISION_16, AUDIO_ENCODING_LINEAR);

    result = true;
    
Done:

    if (!result) {
        if (deviceMap) {
            deviceMap->release();
            deviceMap = NULL;
        }
    }

    return result;
}

bool HDAController::allocateRegsSpinLock() {

	regsSpinLock = IOSimpleLockAlloc();
	if (!regsSpinLock)
		return false;
		
	IOSimpleLockInit(regsSpinLock);

	return true;
}

bool HDAController::freeRegsSpinLock() {

	if (regsSpinLock) {
		IOSimpleLockFree(regsSpinLock);
		regsSpinLock = NULL;
	}

	return true;
}

bool HDAController::allocateMutex() {
	mutex = IOLockAlloc();
	return (mutex != NULL);
}

bool HDAController::freeMutex() {
	if (mutex) {
		IOLockFree(mutex);
		mutex = NULL;
	}
	return true;
}

bool HDAController::allocateCommandMutex() {
	commandMutex = IOLockAlloc();
	if (!commandMutex)
		return false;
	return true;
}

bool HDAController::freeCommandMutex() {
	if (commandMutex) {
		IOLockFree(commandMutex);
		commandMutex = NULL;
	}
	return true;
}

void HDAController::commandLock() {
	IOLockLock(commandMutex);
}

void HDAController::commandUnlock() {
	IOLockUnlock(commandMutex);
}

bool HDAController::initPCIConfigSpace(IOPCIDevice *provider)
{
	UInt16	reg16;
        
    reg16	= provider->configRead16( kIOPCIConfigCommand );
    reg16  &= ~kIOPCICommandIOSpace;

    reg16	|= ( kIOPCICommandBusMaster
			|    kIOPCICommandMemorySpace
			|	 kIOPCICommandMemWrInvalidate );

	provider->configWrite16( kIOPCIConfigCommand, reg16 );

	provider->configWrite8( kIOPCIConfigCacheLineSize, 64 / sizeof( UInt32 ) );
	provider->configWrite8( kIOPCIConfigLatencyTimer, 0xF8 );// max timer - low 3 bits ignored

    return true;
}

// Калька с oss hdaudio.c/reset_contoller
bool HDAController::resetController() {
	UInt32 gctl, tmout;

    IOLog("HDAController[%p]::resetController()\n", this);
	assert(deviceRegisters);
	
	gctl = regsRead32(HDA_GCTL);
	gctl &= ~CRST;
	regsWrite32(HDA_GCTL, gctl);

	tmout = 50;
	while ((regsRead32(HDA_GCTL) & CRST) && --tmout)
		IODelay(1000);
	IODelay(1000);
	
	gctl = regsRead32(HDA_GCTL);
	gctl |= CRST;
	regsWrite32(HDA_GCTL, gctl);

	tmout = 50;
	while (!(regsRead32(HDA_GCTL) & CRST) && --tmout)
		IODelay(1000);
	IODelay(1000);

	if (!regsRead32(HDA_GCTL)) {
		IOLog("HDAController[%p]::resetController() - controller is not ready!\n", this);
		return false;
	}
	
	codecMask = regsRead16(HDA_STATESTS);
	
	return true;
}

void HDAController::enableInterrupts() {
	interruptEventSource->enable();
	/* enable controller CIE and GIE */
	regsWrite32(HDA_INTCTL, regsRead32(HDA_INTCTL) | INT_CTRL_EN | INT_GLOBAL_EN | INT_ALL_STREAM);
}

void HDAController::disableInterrupts() {
	int i;
	unsigned int base;

	/* disable interrupts in stream descriptor */
	base = HDA_SD_BASE;
	for (i = 0; i < numStreams; i++) {
		regsWrite8(base + HDA_SD_CTL, regsRead8(base + HDA_SD_CTL) & ~SD_INT_MASK);
		base += HDA_SD_LEN;
	}

	/* disable SIE for all streams */
	regsWrite8(HDA_INTCTL, 0);

	/* disable controller CIE and GIE */
	regsWrite32(HDA_INTCTL, regsRead32(HDA_INTCTL) &
		   ~(INT_CTRL_EN | INT_GLOBAL_EN));

//	interruptEventsource->disable();   //??????????
}

void HDAController::clearInterrupts() {
	int i;
	unsigned int base;

	/* clear stream status */
	base = HDA_SD_BASE;
	for (i = 0; i < numStreams; i++) {
		regsWrite8(base + HDA_SD_STS, SD_INT_MASK);
		base += HDA_SD_LEN;
	}

	/* clear STATESTS */
	regsWrite8(HDA_STATESTS, STATESTS_INT_MASK);

	/* clear rirb status */
	regsWrite8(HDA_RIRBSTS, RIRB_INT_MASK);

	/* clear int status */
	regsWrite32(HDA_INTSTS, INT_CTRL_EN | INT_ALL_STREAM);
}

void HDAController::stopAllDMA() {
	int i;
	unsigned int base;
	UInt8 tmp;

	regsWrite8(HDA_CORBCTL, 0);
	regsWrite8(HDA_RIRBCTL, 0);

	base = HDA_SD_BASE;
	for (i = 0; i < numStreams; i++) {
		tmp = regsRead8(base + HDA_SD_CTL);
		
		/* for input/output stream, it is the same */
		tmp &= RBCTL_DMA_EN;
		
		regsWrite8(base + HDA_SD_CTL, tmp);
		base += HDA_SD_LEN;
	}
	
	/* wait (40us??) for stream DMA to stop */
	IODelay(40);
}

void HDAController::softInit() {
	const int pints = 32;					/* in solaris >=32 but <= 1500 */
	const int rints = 32;					/* in solaris >=32 but <= 1500 */
	
	flags = 0;
	playbackInterruptFrequence = pints;
	recordInterruptFrequence = rints;
	playbackBufferSize = (HDA_SAMPLER_MAX * HDA_MAX_CHANNELS * HDA_MAX_PRECISION / 8) / pints;
	playbackBufferSize = (playbackBufferSize + HDA_BDLE_BUF_ALIGN - 1) & ~(HDA_BDLE_BUF_ALIGN - 1);
	recordBufferSize = (HDA_SAMPLER_MAX * HDA_MAX_CHANNELS * HDA_MAX_PRECISION / 8) / rints;
	recordBufferSize = (recordBufferSize + HDA_BDLE_BUF_ALIGN - 1) & ~(HDA_BDLE_BUF_ALIGN - 1);

	outputsMuted = false;
}

/*
 * reset and start the controller registers
 */
bool HDAController::initController()
{
	softInit();

	stopAllDMA();

	/* Reset controller */
	if (!resetController())
		return false;

	/* Intr enable */
	enableInterrupts();

	/* allocate DMA playback and record buffers */
	allocateRecordBuffers();
	allocatePlaybackBuffers();

	allocatePositionBuffer();

	/* initialize the codec command I/O */
	/* setup the CORB/RIRB structs */
	if (!initCommandIO())
		return false;

	/* program the position buffer */
	regsWrite32(HDA_DPLBASE, positionBuffer.getPhysicalAddress().low32());
	regsWrite32(HDA_DPUBASE, positionBuffer.getPhysicalAddress().hi32());

	return true;
}

bool HDAController::initHDA() {
	int gcap, vmin, vmaj;
	unsigned int busNumber, deviceNumber, functionNumber;
	unsigned short vendor, device, subvendor, subdevice;
	unsigned char irq_line;

	IOLog("HDAController[%p]::initHDA()\n", this);

	gcap = regsRead32(HDA_GCAP);
	
	dma64ok = gcap & 0x1;
	inputStreams = (gcap >> 8) & 0xf;
	outputStreams = (gcap >> 12) & 0xf;
	numStreams = inputStreams + outputStreams;
	
	/* due to limitations of our driver, lets do things pretty simple */
	recordStreamTag = 1;
	playbackStreamTag = inputStreams + 1;
	recordRegistryBase = HDA_SD_BASE;
	playbackRegistryBase = HDA_SD_BASE + HDA_SD_LEN * inputStreams;
	
	IOLog("gcap=%d\n", gcap);
	IOLog("number of output streams supported = %d\n", outputStreams);
	IOLog("number of input streams supported = %d\n", inputStreams);
	IOLog("number of bidirectional streams supported = %d\n", (gcap >> 3) & 0xf);
	IOLog("number of serial data out signals = %d\n", (gcap >> 1) & 0x3);
	IOLog("64 bit address supported = %d\n", dma64ok);
	
	vmin = regsRead8(HDA_VMIN);
	vmaj = regsRead8(HDA_VMAJ);
	IOLog("version number %d.%d\n", vmaj, vmin);

	busNumber = pciDevice->getBusNumber();
	deviceNumber = pciDevice->getDeviceNumber();
	functionNumber = pciDevice->getFunctionNumber();
	IOLog("busNumber=%u, deviceNumber=%u, functionNumber=%u\n", busNumber, deviceNumber, functionNumber);
	
	vendor = pciDevice->configRead16(PCI_VENDOR_ID);
	device = pciDevice->configRead16(PCI_DEVICE_ID);
	subvendor = pciDevice->configRead16(PCI_SUBSYSTEM_VENDOR_ID);
	subdevice = pciDevice->configRead16(PCI_SUBSYSTEM_ID);
	irq_line = pciDevice->configRead8(PCI_INTERRUPT_LINE);
	
	IOLog("vendor=%x, device=%x\n", vendor, device);
	IOLog("subvendor=%x, subdevice=%x\n", subvendor, subdevice);
	IOLog("irq=%d\n", (int)irq_line);


	/* allocate spinlocks */
	allocateRegsSpinLock();
	/* allocate command locking mutex */
	allocateCommandMutex();
	allocateMutex();

	
	/* setup interrupt handlers */
	IOWorkLoop *workLoop;
	workLoop = getWorkLoop();

	interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
								OSMemberFunctionCast(IOInterruptEventAction,
												this,
												&HDAController::handleInterrupt),
								OSMemberFunctionCast(IOFilterInterruptAction,
												this,
												&HDAController::filterInterrupt),
								pciDevice, 1);
	if (!interruptEventSource) {
		IOLog("HDAController[%p]::initHDA() cannot create interruptEventSource\n", this);
		return false;
	}
	
	if (workLoop->addEventSource(interruptEventSource) != kIOReturnSuccess) {
		IOLog("HDAController[%p]::initHDA() failed to add interrupt event source\n", this);
		return false;
	}
	
	interruptsHandled = 0;
	interruptsAquired = 0;
	unsolicited = 0;

	if (!initController()) {
		return false;
	}

	/* setup the engine structs */
//	if (!setupEngines())
//		return false;

	if (!codecMask) {
		IOLog("HDAController[%p]::initHDA() - no codecs found after reset\n", this);
		return false;
	}
	
	// TODO: Там дальше еще есть!
	return true;
}

/* determine if rirb interrupt occured */
bool HDAController::filterInterrupt(IOInterruptEventSource *source) {
	UInt32 status;
	int i;
	unsigned int regbase;
	bool needHandler = false;
	
	++interruptsAquired;
	
	IOInterruptState state = IOSimpleLockLockDisableInterrupt(regsSpinLock);

	status = regsRead8(HDA_INTSTS);
	for (i = 0; i < numStreams; i++) {
		if ((status  & (1<<i)) == 0)
			continue;
		
		regbase = HDA_SD_BASE + HDA_SD_LEN * i;
		regsWrite8(regbase + HDA_SD_STS, SD_INT_MASK);
		if (i == inputStreams && (flags & PLAY_STARTED)) {
			needHandler = true;
		}
	}

	status = regsRead8(HDA_RIRBSTS);
	if (status & RIRB_INT_MASK) {
		if (!isSingleCommand && (status & RIRB_INT_RESPONSE)) {
			updateRirb();
			++interruptsHandled;
		}
		regsWrite8(HDA_RIRBSTS, RIRB_INT_MASK);
	}
	IOSimpleLockUnlockEnableInterrupt(regsSpinLock, state);

	return needHandler;
}

void HDAController::handleInterrupt(IOInterruptEventSource *source, int count) {
	IOLockLock(mutex);
	if (flags & PLAY_STARTED) {
		if (++audioEngine->numberOfPlaybackBDLEPassed == HDA_BDLE_NUMS) {
			audioEngine->takeTimeStamp();
			audioEngine->numberOfPlaybackBDLEPassed = 0;
		}
	}
	IOLockUnlock(mutex);
}

void HDAController::free()
{
    IOLog("HDAController[%p]::free()\n", this);
    
	stopAllDMA();

	audioEngine->stopPlayback(0);
    audioEngine->release();

	if (interruptEventSource) {
		interruptEventSource->disable();
		getWorkLoop()->removeEventSource(interruptEventSource);
		interruptEventSource->release();
		interruptEventSource = NULL;
		IOLog("interruptsHandled = %d\n", interruptsHandled);
		IOLog("interruptsAquired = %d\n", interruptsAquired);
		IOLog("unsolicited = %d\n", unsolicited);
	}
	
    if (deviceMap) {
        deviceMap->release();
        deviceMap = NULL;
    }
	
	freeRingBuffers();
	freePlaybackBuffers();
	freeRecordBuffers();
	
	freePositionBuffer();
	
	freeRegsSpinLock();
	freeCommandMutex();
	freeMutex();
    
    super::free();
}
    
bool HDAController::createAudioEngine()
{
    bool result = false;
    IOAudioControl *control;
    
    IOLog("HDAController[%p]::createAudioEngine()\n", this);
    
    audioEngine = new HDACodec;
    if (!audioEngine) {
        goto Done;
    }
    
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
	// Здесь по идее нужно проанализировать codecmask и создать нужное число кодеков
	// Но это даже не альфа-версия, и я знаю, что кодек один. Так что пусть это будет в
	// TODO
    if (!audioEngine->init(this, 0)) {
        goto Done;
    }
    
    // Create a left & right output volume control with an int range from 0 to 65535
    // and a db range from -22.5 to 0.0
    // Once each control is added to the audio engine, they should be released
    // so that when the audio engine is done with them, they get freed properly
    control = IOAudioLevelControl::createVolumeControl(32767/*65535*/,	// Initial value
                                                        0,		// min value
                                                        65535,	// max value
                                                        (-22 << 16) + (32768),	// -22.5 in IOFixed (16.16)
                                                        0,		// max 0.0 in IOFixed
                                                        kIOAudioControlChannelIDDefaultLeft,
                                                        kIOAudioControlChannelNameLeft,
                                                        0,		// control ID - driver-defined
                                                        kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    control = IOAudioLevelControl::createVolumeControl(32767/*65535*/,	// Initial value
                                                        0,		// min value
                                                        65535,	// max value
                                                        (-22 << 16) + (32768),	// min -22.5 in IOFixed (16.16)
                                                        0,		// max 0.0 in IOFixed
                                                        kIOAudioControlChannelIDDefaultRight,	// Affects right channel
                                                        kIOAudioControlChannelNameRight,
                                                        1,		// control ID - driver-defined
                                                        kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
        
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();

    // Create an output mute control
    control = IOAudioToggleControl::createMuteControl(false,	// initial state - unmuted
                                                        kIOAudioControlChannelIDAll,	// Affects all channels
                                                        kIOAudioControlChannelNameAll,
                                                        0,		// control ID - driver-defined
                                                        kIOAudioControlUsageOutput);
                                
    if (!control) {
        goto Done;
    }
        
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)outputMuteChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();

    // Create a left & right input gain control with an int range from 0 to 65535
    // and a db range from 0 to 22.5
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                        0,		// min value
                                                        65535,	// max value
                                                        0,		// min 0.0 in IOFixed
                                                        (22 << 16) + (32768),	// 22.5 in IOFixed (16.16)
                                                        kIOAudioControlChannelIDDefaultLeft,
                                                        kIOAudioControlChannelNameLeft,
                                                        1,		// control ID - driver-defined
                                                        kIOAudioControlUsageInput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)gainChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                        0,		// min value
                                                        65535,	// max value
                                                        0,		// min 0.0 in IOFixed
                                                        (22 << 16) + (32768),	// max 22.5 in IOFixed (16.16)
                                                        kIOAudioControlChannelIDDefaultRight,	// Affects right channel
                                                        kIOAudioControlChannelNameRight,
                                                        0,		// control ID - driver-defined
                                                        kIOAudioControlUsageInput);
    if (!control) {
        goto Done;
    }
        
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)gainChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();

    // Create an input mute control
    control = IOAudioToggleControl::createMuteControl(false,	// initial state - unmuted
                                                        kIOAudioControlChannelIDAll,	// Affects all channels
                                                        kIOAudioControlChannelNameAll,
                                                        0,		// control ID - driver-defined
                                                        kIOAudioControlUsageInput);
                                
    if (!control) {
        goto Done;
    }
        
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)inputMuteChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();

    // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
    // After this function returns, that audio engine should be ready to begin vending audio services to the system
    activateAudioEngine(audioEngine);
    // Once the audio engine has been activated, release it so that when the driver gets terminated,
    // it gets freed
    
    result = true;
    
Done:

    if (!result && (audioEngine != NULL)) {
        audioEngine->release();
    }

    return result;
}

IOReturn HDAController::volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAController *audioDevice;
    
    audioDevice = (HDAController *)target;
    if (audioDevice) {
        result = audioDevice->volumeChanged(volumeControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAController::volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAController[%p]::volumeChanged(%p, %ld, %ld)\n", this, volumeControl, oldValue, newValue);
    
    if (volumeControl) {
        IOLog("\t-> Channel %ld\n", volumeControl->getChannelID());
    }
    
	unsigned int myvalue = newValue * playbackGainMax / 65536;
	
    // Add hardware volume code change 
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultLeft)
		audioEngine->setGain(AUDIO_PLAY, myvalue, 0);
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultRight)
		audioEngine->setGain(AUDIO_PLAY, myvalue, 1);

    return kIOReturnSuccess;
}
    
IOReturn HDAController::outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAController *audioDevice;
    
    audioDevice = (HDAController *)target;
    if (audioDevice) {
        result = audioDevice->outputMuteChanged(muteControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAController::outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAController[%p]::outputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add output mute code here
	audioEngine->muteOutputs(newValue);
    
    return kIOReturnSuccess;
}

IOReturn HDAController::gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAController *audioDevice;
    
    audioDevice = (HDAController *)target;
    if (audioDevice) {
        result = audioDevice->gainChanged(gainControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAController::gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAController[%p]::gainChanged(%p, %ld, %ld)\n", this, gainControl, oldValue, newValue);
    
    if (gainControl) {
        IOLog("\t-> Channel %ld\n", gainControl->getChannelID());
    }
    
    // Add hardware gain change code here 

    return kIOReturnSuccess;
}
    
IOReturn HDAController::inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    HDAController *audioDevice;
    
    audioDevice = (HDAController *)target;
    if (audioDevice) {
        result = audioDevice->inputMuteChanged(muteControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn HDAController::inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("HDAController[%p]::inputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add input mute change code here
    
    return kIOReturnSuccess;
}
