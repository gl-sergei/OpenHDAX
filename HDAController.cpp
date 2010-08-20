/*
 * Intel High Definition Audio Device Driver for OS X
 *
 * Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 * Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 * was a great sample and probably start point for this driver.
 */

#include "HDAController.h"
#include "HDAIOEngine.h"
#include "HDATestingUserClient.h"

#include <IOKit/audio/IOAudioControl.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioDefines.h>

#include <IOKit/IOLib.h>

#include <IOKit/pci/IOPCIDevice.h>

#define super IOAudioDevice

OSDefineMetaClassAndStructors(HDAController, IOAudioDevice)

bool HDAController::allocatePositionBuffer() {

	IOLog("HDAController::allocatePositionBuffer size = %d\n", numStreams * 8);

	positionBuffer = HDADMABuffer::withSize(numStreams * 8, dma64ok);
	if (!positionBuffer)
		return false;
	return true;
}

void HDAController::freePositionBuffer() {
	if (positionBuffer)
		positionBuffer->release();
}

bool HDAController::init(OSDictionary *dictionary)
{
    IOLog("HDAController[%p]::init(%p)\n", this, dictionary);

	if (!super::init(dictionary))
		return false;
	
	// all objects initialy unitialized
    pciDevice = NULL;
	deviceRegs = NULL;
	commandTransmitter = NULL;
	interruptEventSource = NULL;
	mutex = NULL;

	if (!allocateMutex())
		return false;

	widgets = new HDAAudioWidget[256];

	return true;
}

/* Смотрим Lowlevel Interface */

bool HDAController::initHardware(IOService *provider)
{
    IOLog("HDAController[%p]::initHardware(%p)\n", this, provider);

    if (!super::initHardware(provider))
		return false;
    
    // Get the PCI device provider
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if (!pciDevice)
		return false;
    
	deviceRegs = HDAPCIRegisters::withPCIDevice(pciDevice);
    if (!deviceRegs)
		return false;

    // add the hardware init code here
	deviceRegs->initPCIConfigSpace();
    
    setDeviceName("Intel HDA controller (ESB2 chipset)");
    setDeviceShortName("IntelHDA");
    setManufacturerName("Intel");

//#error Put your own hardware initialization code here...and in other routines!!

	commandTransmitter = HDACommandTransmitter::withPCIRegs(deviceRegs);
	if (!commandTransmitter)
		return false;

	// найти более подходящее место. И вообще контроллер не должен сам выделять буфера для кодеков. А лишь предоставлять
	// необходимый интерфейс для этого. Или вообще не должен.
	positionBuffer = NULL;

	if (!initHDA())
		return false;
	
    if (!createAudioEngine())
		return false;

//	audioEngine->setFormat(0, AUDIO_PLAY, HDA_SAMPR48000, AUDIO_CHANNELS_STEREO, AUDIO_PRECISION_16, AUDIO_ENCODING_LINEAR);

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

// TODO: Ниже есть три полезные в некоторых случаях процедуры
// нужно со временем переписать их в человеческом виде и избавить от
// некоторых ошибок

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

	interruptEventSource->disable();
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

void HDAController::enablePositionBuffer()
{
	/* enable position buffer */
	if (!(regsRead32(HDA_DPLBASE) & DPLBASE_ENABLE))
	{
		regsWrite32(HDA_DPUBASE, positionBuffer->getPhysicalAddress().hi32());
		regsWrite32(HDA_DPLBASE, positionBuffer->getPhysicalAddress().low32() | DPLBASE_ENABLE);
	}
}

void HDAController::disablePositionBuffer()
{
	/* enable position buffer */
	regsWrite32(HDA_DPLBASE, 0);
	regsWrite32(HDA_DPUBASE, 0);
}

void HDAController::stopAllDMA() {
	int i;
	unsigned int base;
	UInt8 tmp;

	disablePositionBuffer();
	
	commandTransmitter->stop();
	
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

/*
 * reset and start the controller registers
 */
bool HDAController::initController()
{

	stopAllDMA();

	/* ICH6/ICH7 book tells that position buffer must be set up before controller reset (we will see...)*/
	if (!allocatePositionBuffer())
		return false;

	/* program the position buffer */
	regsWrite32(HDA_DPLBASE, positionBuffer->getPhysicalAddress().low32());
	regsWrite32(HDA_DPUBASE, positionBuffer->getPhysicalAddress().hi32());

	/* Reset controller */
	if (!resetController())
		return false;

	/* Intr enable */
	enableInterrupts();

	commandTransmitter->initHardware();
	commandTransmitter->start();


	/* program the position buffer */
	regsWrite32(HDA_DPLBASE, positionBuffer->getPhysicalAddress().low32());
	regsWrite32(HDA_DPUBASE, positionBuffer->getPhysicalAddress().hi32());

	enablePositionBuffer();

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
	
	vendor = pciDevice->configRead16(kIOPCIConfigVendorID);
	device = pciDevice->configRead16(kIOPCIConfigVendorID);
	subvendor = pciDevice->configRead16(kIOPCIConfigSubSystemVendorID);
	subdevice = pciDevice->configRead16(kIOPCIConfigSubSystemID);
	irq_line = pciDevice->configRead8(kIOPCIConfigInterruptLine);
	
	IOLog("vendor=%x, device=%x\n", vendor, device);
	IOLog("subvendor=%x, subdevice=%x\n", subvendor, subdevice);
	IOLog("irq=%d\n", (int)irq_line);

	/* setup interrupt handlers */
	IOWorkLoop *workLoop;
	workLoop = getWorkLoop();

	interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this, 
																					OSMemberFunctionCast(IOInterruptEventAction,
																										this,
																										 &HDAController::handleInterrupt),
																					OSMemberFunctionCast(IOFilterInterruptEventSource::Filter,
																										 this,
																										 &HDAController::filterInterrupt),
																					pciDevice, 0);
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

	if (!codecMask) {
		IOLog("HDAController[%p]::initHDA() - no codecs found after reset\n", this);
		return false;
	}
	
	int i;
	for (i = 2; i < 2 + 26; i++)
	{
		widgets[i].init(commandTransmitter, 0, i);
//		widgets[i].print();
	}
	
	// TODO: Там дальше еще есть!
	return true;
}

bool HDAController::filterInterrupt(IOFilterInterruptEventSource *source) {
	UInt32 status;
	
	status = regsRead32(HDA_INTSTS);
	if (status & INTSTS_BIT_GIS) {
		interruptStatus = status;
		return true;
	}

	return false;
}

void HDAController::handleInterrupt(IOFilterInterruptEventSource *source, int count) {

	HDAIOEngine *engine;
	UInt32 status;
	unsigned i, j;
	unsigned regbase;
	
	++interruptsAquired;
	if (count > 1)
		IOLog("HDAController::handleInterrupt count = %d\n", count);
	
	status = interruptStatus;
	for (i = 0; i < numStreams; i++) {
		if ((status  & (1<<i)) == 0)
			continue;
		
		regbase = getStreamBaseRegByTag(i + 1);
		regsWrite8(regbase + HDA_SD_STS, SD_INT_MASK);
		if (audioEngines) {
			for (j = 0; j < audioEngines->getCount(); j++)
			{
				engine = (HDAIOEngine*)audioEngines->getObject(j);
				engine->handleStreamInterrupt(i + 1);
			}
		}
	}

	status = regsRead8(HDA_RIRBSTS);
	if (status & RIRB_INT_MASK) {
		if (/*!isSingleCommand && */(status & RIRB_INT_RESPONSE)) {
			commandTransmitter->updateRirb();
			++interruptsHandled;
		}
		regsWrite8(HDA_RIRBSTS, RIRB_INT_MASK);
	}
}

void HDAController::stop(IOService *provider)
{

	IOLog("HDAController[%p]::stop(%p)\n", this, provider);

	if (commandTransmitter)
	{
		IOLog("HDAController[%p]::stop(%p) - stopAllDMA\n", this, provider);
		stopAllDMA();
	}
	
	if (interruptEventSource) {
		IOLog("HDAController[%p]::stop(%p) - disableInterruptEventSource\n", this, provider);
		interruptEventSource->disable();
		getWorkLoop()->removeEventSource(interruptEventSource);
		IOLog("interruptsHandled = %d\n", interruptsHandled);
		IOLog("interruptsAquired = %d\n", interruptsAquired);
		IOLog("unsolicited = %d\n", unsolicited);
	}
	
	IOLog("HDAController[%p]::stop(%p) - super::stop()\n", this, provider);
	super::stop(provider);
	
}

void HDAController::free()
{
    IOLog("HDAController[%p]::free()\n", this);

	if (widgets)
	{
		IOLog("HDAController::free delete [] widgets\n");
		delete [] widgets;
	}
	
	if (commandTransmitter) {
		IOLog("HDAController::free releasing command transmitter\n");
		commandTransmitter->release();
		commandTransmitter = NULL;
	}

	if (interruptEventSource)
	{
		IOLog("HDAController::free releasing interrupt event source\n");
		interruptEventSource->release();
		interruptEventSource = NULL;
	}

	IOLog("HDAController::free position buffer\n");
	freePositionBuffer();
	
	IOLog("HDAController::free mutex\n");
	freeMutex();
	
    if (deviceRegs) {
		IOLog("HDAController::free releasing device registers\n");
        deviceRegs->release();
        deviceRegs = NULL;
    }

	IOLog("HDAController::free super::free\n");
    super::free();
}
    
bool HDAController::createAudioEngine()
{
    bool result = false;
    IOLog("HDAController[%p]::createAudioEngine()\n", this);
    
	HDAIOEngine *outputEngine;
	
	outputEngine = new HDAIOEngine;
    
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
	// Здесь по идее нужно проанализировать codecmask и создать нужное число кодеков
	// Но это даже не альфа-версия, и я знаю, что кодек один. Так что пусть это будет в
	// TODO
	if (!outputEngine->init(kIOAudioStreamDirectionOutput, this, inputStreams + 1, 0))
	{
		IOLog("failed to initialize outputEngine\n");
		return false;
	}

	HDAAudioWidget *oconv[1];
	oconv[0] = &widgets[0x2];
	outputEngine->setConvertors(oconv, 1);

	HDAAudioWidget *omix[2];
	omix[0] = &widgets[0x8]; omix[1] = &widgets[0x9];
	outputEngine->setMixers(omix, 2);

	HDAAudioWidget *opin[2];
	opin[0] = &widgets[0x10]; opin[1] = &widgets[0xf];
	outputEngine->setPins(opin, 2);

    // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
    // After this function returns, that audio engine should be ready to begin vending audio services to the system
    activateAudioEngine(outputEngine);
	outputEngine->release();
    // Once the audio engine has been activated, release it so that when the driver gets terminated,
    // it gets freed


	HDAIOEngine *inputEngine;
	inputEngine = new HDAIOEngine;

	if (!inputEngine->init(kIOAudioStreamDirectionInput, this, 1, 0))
	{
		IOLog("failed to initialize inputEngine\n");
		return false;
	}

	HDAAudioWidget *iconv[1];
	iconv[0] = &widgets[0x4];
	inputEngine->setConvertors(iconv, 1);

	HDAAudioWidget *ipin[1];
	ipin[0] = &widgets[0x12];
	inputEngine->setPins(ipin, 1);
    
	activateAudioEngine(inputEngine);
	inputEngine->release();
	
    result = true;
    
    return result;
}

HDACommandTransmitter *HDAController::getCommandTransmitter() {
	return commandTransmitter;
}

HDAPCIRegisters *HDAController::getPCIRegisters()
{
	return deviceRegs;
}

HDADMABuffer *HDAController::getPositionBuffer()
{
	return positionBuffer;
}

unsigned int HDAController::getStreamBaseRegByTag(int stream) {
	return HDA_SD_BASE + HDA_SD_LEN * (stream - 1);
}

IOReturn HDAController::newUserClient( task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{

	IOReturn ior = kIOReturnSuccess;

	IOLog("HDAController[%p]::newUserClient()\n", this);
	
	HDATestingUserClient *client;
	
	client = new HDATestingUserClient;
	if (!client) {
		IOLog("HDAController[%p]::newUserClient() new failed\n", this);
		return kIOReturnError;
	}
	
	if ( !client->initWithTask( owningTask, securityID, type ) ) {
		IOLog("HDAController[%p]::newUserClient() initWithTask failed\n", this);
		ior = kIOReturnError;
	}

	if ( ior == kIOReturnSuccess )
	{		// Attach ourself to the client so that this client instance can call us.
		if ( client->attach( this ) == false )
		{
			IOLog("HDAController[%p]::newUserClient() attach failed\n", this);
			ior = kIOReturnError;
		}
	}

	if ( ior == kIOReturnSuccess )
	{		// Start the client so it can accept requests.
		if ( client->start( this ) == false )
		{
			IOLog("HDAController[%p]::newUserClient() start failed\n", this);
			ior = kIOReturnError;
		}
	}

	if ( client && (ior != kIOReturnSuccess) )
	{
		client->detach( this );
		client->release();
		client = 0;
	}

	*handler = client;
	return ior;
}
