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

bool HDAController::allocatePlaybackBuffers() {
	IOLog("allocatePlaybackBuffers: size = %d, dma64ok=%d\n", (int)playbackBufferSize * HDA_BDLE_NUMS, (int)dma64ok);

	/* allocate DMA for data buffer of playback stream */
	playbackBuffer = HDADMABuffer::withSize(playbackBufferSize * HDA_BDLE_NUMS, dma64ok);
	if (!playbackBuffer)
		return false;

//	for (int i = 0; i < playbackBufferSize * HDA_BDLE_NUMS; i++) {
//		((char*)playbackBuffer.buf)[i] = (char)i;
//	}

	/* allocate DMA for buffer descriptor list of playback stream */
	playbackBufferDescriptor = HDADMABuffer::withSize(sizeof(BDLEntry) * HDA_BDLE_NUMS);
	if (!playbackBufferDescriptor)
		return false;

	return true;
}

bool HDAController::allocateRecordBuffers() {
	/* allocate DMA for data buffer of record stream */
	recordBuffer = HDADMABuffer::withSize(recordBufferSize * HDA_BDLE_NUMS, dma64ok);
	if (!recordBuffer)
		return false;

	/* allocate DMA for buffer descriptor list of record stream */
	recordBufferDescriptor = HDADMABuffer::withSize(sizeof(BDLEntry) * HDA_BDLE_NUMS);
	if (!recordBufferDescriptor)
		return false;

	return true;
}

void HDAController::freePlaybackBuffers() {
	if (playbackBuffer)
		playbackBuffer->release();
	if (playbackBufferDescriptor)
		playbackBufferDescriptor->release();
}

void HDAController::freeRecordBuffers() {
	if (recordBuffer)
		recordBuffer->release();
	if (recordBufferDescriptor)
		recordBufferDescriptor->release();
}

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

/* Смотрим Lowlevel Interface */

bool HDAController::initHardware(IOService *provider)
{
	// all objects initialy unitialized
    pciDevice = NULL;
    outputEngine = NULL;
	deviceRegs = NULL;
	commandTransmitter = NULL;
	interruptEventSource = NULL;
	mutex = NULL;
	playbackBufferDescriptor = NULL;
	playbackBuffer = NULL;
	recordBufferDescriptor = NULL;
	recordBuffer = NULL;
	positionBuffer = NULL;
    
    IOLog("HDAController[%p]::initHardware(%p)\n", this, provider);
    
	widgets = new HDAAudioWidget[256];

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
	playbackBufferDescriptor = playbackBuffer = recordBufferDescriptor = recordBuffer = positionBuffer = NULL;

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

void HDAController::enablePositionBuffer()
{
	/* enable position buffer */
	if (!(regsRead32(HDA_DPLBASE) & DPLBASE_ENABLE))
	{
		regsWrite32(HDA_DPUBASE, positionBuffer->getPhysicalAddress().hi32());
		regsWrite32(HDA_DPLBASE, positionBuffer->getPhysicalAddress().low32() | DPLBASE_ENABLE);
	}
}

void HDAController::stopAllDMA() {
	int i;
	unsigned int base;
	UInt8 tmp;

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

void HDAController::softInit() {
//	const int pints = 32;					/* in solaris >=32 but <= 1500 */
//	const int rints = 32;					/* in solaris >=32 but <= 1500 */
	
	flags = 0;
//	playbackInterruptFrequence = pints;
//	recordInterruptFrequence = rints;
	playbackBufferSize = 1024 * 8;//(HDA_SAMPLER_MAX * HDA_MAX_CHANNELS * HDA_MAX_PRECISION / 8) / pints;
	playbackBufferSize = (playbackBufferSize + HDA_BDLE_BUF_ALIGN - 1) & ~(HDA_BDLE_BUF_ALIGN - 1);
	recordBufferSize = 1024 * 8;//(HDA_SAMPLER_MAX * HDA_MAX_CHANNELS * HDA_MAX_PRECISION / 8) / rints;
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

	if (!allocatePlaybackBuffers())
		return false;
	/* allocate DMA playback and record buffers */
	if (!allocateRecordBuffers())
		return false;
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
	
	/* TODO:
	 * streamTag - логический номер потока, передающегося через линк
	 *    (0 - зарезервирован и не может использоваться. Максимальный - 15)
	 * registryBase - базовый адрес группы регистров управления потоком
	 *    всего таких регистров по идее inputStreams + outputStreams + bidirectionalStreams
	 * bufferDescriptor - описание потока
	 * buffer - буфер потока, непрерывен в памяти и состоит BDLE_NUMS частей
	 * НУЖНО СДЕЛАТЬ ИНТЕРФЕЙС ДЛЯ КОДЕКА, ПОЗВОЛЯЮЩИЙ ВЫДЕЛЯТЬ ВСЕ ЭТО В СВЯЗКЕ ПО ЗАПРОСУ КОДЕКА
	 */
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
	
	vendor = pciDevice->configRead16(kIOPCIConfigVendorID);
	device = pciDevice->configRead16(kIOPCIConfigVendorID);
	subvendor = pciDevice->configRead16(kIOPCIConfigSubSystemVendorID);
	subdevice = pciDevice->configRead16(kIOPCIConfigSubSystemID);
	irq_line = pciDevice->configRead8(kIOPCIConfigInterruptLine);
	
	IOLog("vendor=%x, device=%x\n", vendor, device);
	IOLog("subvendor=%x, subdevice=%x\n", subvendor, subdevice);
	IOLog("irq=%d\n", (int)irq_line);


	if (!allocateMutex())
		return false;

	
	/* setup interrupt handlers */
	IOWorkLoop *workLoop;
	workLoop = getWorkLoop();

	interruptEventSource = IOInterruptEventSource::interruptEventSource(this,
								OSMemberFunctionCast(IOInterruptEventAction,
													this,
													&HDAController::handleInterrupt),
								pciDevice,
								1 /* from Apple mailing list: "Historically 0 is the normal interrupt, and 1 is for MSI" */);
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
	
	int i;
	for (i = 2; i < 2 + 26; i++)
	{
		widgets[i].init(commandTransmitter, 0, i);
//		widgets[i].print();
	}
	
	// TODO: Там дальше еще есть!
	return true;
}

void HDAController::handleInterrupt(IOInterruptEventSource *source, int count) {

	HDAIOEngine *engine;
	UInt32 status;
	unsigned i, j;
	unsigned regbase;
	
	++interruptsAquired;
	if (count > 1)
		IOLog("HDAController::handleInterrupt count = %d\n", count);
	
	IOInterruptState state = deviceRegs->lock();

	status = regsRead8(HDA_INTSTS);
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
	deviceRegs->unlock(state);
}

void HDAController::free()
{
    IOLog("HDAController[%p]::free()\n", this);

	/* disable position buffer */
	regsWrite32(HDA_DPLBASE, 0);
	regsWrite32(HDA_DPUBASE, 0);

	delete [] widgets;
	
	if (commandTransmitter) {
		IOLog("HDAController::free stopping all DMA\n");
		stopAllDMA();
	}

	/* stop the position buffer */
	IOLog("HDAController::free stopping position buffer\n");
	regsWrite32(HDA_DPLBASE, 0);
	regsWrite32(HDA_DPUBASE, 0);

//	if (audioEngine) {
//		IOLog("HDAController::free stopping audio engine\n");
//		audioEngine->stopPlayback(0);
//		audioEngine->release();
//	}

	if (interruptEventSource) {
		IOLog("HDAController::free distabling interrupt event source\n");
		interruptEventSource->disable();
		getWorkLoop()->removeEventSource(interruptEventSource);
		interruptEventSource->release();
		interruptEventSource = NULL;
		IOLog("interruptsHandled = %d\n", interruptsHandled);
		IOLog("interruptsAquired = %d\n", interruptsAquired);
		IOLog("unsolicited = %d\n", unsolicited);
	}

	if (commandTransmitter) {
		IOLog("HDAController::free releasing command transmitter\n");
		commandTransmitter->release();
		commandTransmitter = NULL;
	}

	freePlaybackBuffers();
	freeRecordBuffers();
	
	freePositionBuffer();
	
	freeMutex();
    
	
    if (deviceRegs) {
		IOLog("HDAController::free releasing device registers\n");
        deviceRegs->release();
        deviceRegs = NULL;
    }

    super::free();
}
    
bool HDAController::createAudioEngine()
{
    bool result = false;
    IOAudioControl *control;
    
    IOLog("HDAController[%p]::createAudioEngine()\n", this);
    
//    audioEngine = new HDACodec;
//    if (!audioEngine) {
//        goto Done;
//    }

	outputEngine = new HDAIOEngine;
    
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
	// Здесь по идее нужно проанализировать codecmask и создать нужное число кодеков
	// Но это даже не альфа-версия, и я знаю, что кодек один. Так что пусть это будет в
	// TODO
//    if (!audioEngine->init(this, 0)) {
//        goto Done;
//    }
	if (!outputEngine->init(kIOAudioStreamDirectionOutput, this, inputStreams + 1, 0))
	{
		IOLog("failed to initialize outputEngine\n");
		return false;
	}

	HDAAudioWidget *conv[1];
	conv[0] = &widgets[0x2];
	outputEngine->setConvertors(conv, 1);

	HDAAudioWidget *mix[2];
	mix[0] = &widgets[0x8]; mix[1] = &widgets[0x9];
	outputEngine->setMixers(mix, 2);

	HDAAudioWidget *pin[2];
	pin[0] = &widgets[0x10]; pin[1] = &widgets[0xf];
	outputEngine->setPins(pin, 2);

    // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
    // After this function returns, that audio engine should be ready to begin vending audio services to the system
    activateAudioEngine(outputEngine);
	outputEngine->release();
    // Once the audio engine has been activated, release it so that when the driver gets terminated,
    // it gets freed
    
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

/*
 * ВАЖНОЕ ЗАМЕЧАНИЕ! Тэги начинаются с нуля (0)
 */
unsigned int HDAController::getStreamBaseRegByTag(int stream) {
	return HDA_SD_BASE + HDA_SD_LEN * (stream - 1);
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
    
#if 0
    if (volumeControl) {
        IOLog("\t-> Channel %ld\n", volumeControl->getChannelID());
    }
    
	unsigned int myvalue = newValue * playbackGainMax / 65536;
	
    // Add hardware volume code change 
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultLeft)
		audioEngine->setGain(AUDIO_PLAY, myvalue, 0);
	if (volumeControl->getChannelID() == kIOAudioControlChannelIDDefaultRight)
		audioEngine->setGain(AUDIO_PLAY, myvalue, 1);
#endif
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
//	audioEngine->muteOutputs(newValue);
    
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
