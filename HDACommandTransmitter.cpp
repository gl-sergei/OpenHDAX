/*
 *  HDACommandTransmitter.cpp
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 12/14/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDACommandTransmitter.h"

#include <IOKit/IOLib.h>

/*
 * HDACommandResponse
 */

OSDefineMetaClassAndStructors(HDACommandResponse, IOCommand)

bool HDACommandResponse::init(UInt32 val, UInt32 val_ex) {

	if (!IOCommand::init())
		return false;

	value = val; value_ex = val_ex;

	return true;
}

void HDACommandResponse::free() {

	IOCommand::free();
}

HDACommandResponse *HDACommandResponse::withValue(UInt32 val, UInt32 val_ex) {

	HDACommandResponse *commandResponse = new HDACommandResponse;

	if (!commandResponse->init(val, val_ex)) {
		commandResponse->release();
		commandResponse = NULL;
	}

	return commandResponse;
}

UInt32 HDACommandResponse::getValue() {

	return value;
}

UInt32 HDACommandResponse::getValueEx() {

	return value_ex;
}


/*
 * HDACommandTransmitter
 */

#define super OSObject

OSDefineMetaClassAndStructors(HDACommandTransmitter, OSObject)

bool HDACommandTransmitter::init(HDAPCIRegisters *registers) {

	if (!super::init())
		return false;

	commandBuffer = NULL; regs = NULL;
//	unsolicitedResponses = NULL;
	singleMode = false;

	regs = registers;

/*	if (regs)
		pciDevice = regs->getDevice();*/

	workLoop = IOWorkLoop::workLoop();
	if (workLoop)
		commandGate = IOCommandGate::commandGate(this);

	if (commandGate)
		workLoop->addEventSource(commandGate);

	commandBuffer = HDADMABuffer::withSize(4069, true);
	if (!commandBuffer)
		return false;

/*	unsolicitedResponses = IOCommandPool::withWorkLoop(pciDevice->getWorkLoop());
	if (!unsolicitedResponses)
		return false;*/

	corb.phaddr = commandBuffer->getPhysicalAddress();
	corb.vaddr = (UInt32*)commandBuffer->getVirtualAddress();
	rirb.phaddr = corb.phaddr.offset(2048);
	rirb.vaddr = corb.vaddr + 512;

	IOLog("HDACommandTransmitter[%p]::init() corb: phaddr=%08qx, buf=%p\n", this, corb.phaddr.whole64(), corb.vaddr);
	IOLog("HDACommandTransmitter[%p]::init() rirb: phaddr=%08qx, buf=%p\n", this, rirb.phaddr.whole64(), rirb.vaddr);
	
	return true;
}

void HDACommandTransmitter::free() {
	
	if (workLoop) {
		if (commandGate) {
			workLoop->removeEventSource(commandGate);
			commandGate->release();
			commandGate = NULL;
		}
		workLoop->release();
		workLoop = NULL;
	}
	
	if (commandBuffer) {
		commandBuffer->release();
		commandBuffer = NULL;
	}

/*	if (unsolicitedResponses) {
		unsolicitedResponses->release();
		unsolicitedResponses = NULL;
	}*/

	super::free();
}

HDACommandTransmitter *HDACommandTransmitter::withPCIRegs(HDAPCIRegisters *registers) {

	HDACommandTransmitter *command;

	command = new HDACommandTransmitter;
	if (!command->init(registers))
	{
		command->release();
		command = NULL;
	}

	return command;
}

void HDACommandTransmitter::initHardware() {

	/* CORB set up */
	regs->write32(HDA_CORBLBASE, corb.phaddr.low32());
	regs->write32(HDA_CORBUBASE, corb.phaddr.hi32());

	/* set the corb size to 256 entries (ULI requires explicitly) */
	regs->write8(HDA_CORBSIZE, 0x02);
	/* set the corb write pointer to 0 */
	regs->write16(HDA_CORBWP, 0);
	/* reset the corb hw read pointer */
	regs->write16(HDA_CORBRP, RBRWP_CLR);

	/* RIRB set up */
	regs->write32(HDA_RIRBLBASE, rirb.phaddr.low32());
	regs->write32(HDA_RIRBUBASE, rirb.phaddr.hi32());

	/* set the rirb size to 256 entries (ULI requires explicitly) */
	regs->write8(HDA_RIRBSIZE, 0x02);
	/* reset the rirb hw write pointer */
	regs->write16(HDA_RIRBWP, RBRWP_CLR);
	/* set N=1, get RIRB response interrupt for new entry */
	regs->write16(HDA_RINTCNT, 1);
}

void HDACommandTransmitter::start() {

	/* enable corb dma */
	regs->write8(HDA_CORBCTL, RBCTL_DMA_EN);
	/* enable rirb dma and response irq */
	regs->write8(HDA_RIRBCTL, RBCTL_DMA_EN | RBCTL_IRQ_EN);
	rirb.rp = rirb.cmds = 0;
}

void HDACommandTransmitter::stop() {

	/* disable ringbuffer DMAs */
	regs->write8(HDA_RIRBCTL, 0);
	regs->write8(HDA_CORBCTL, 0);
}

bool HDACommandTransmitter::corbSendCommand(UInt32 command) {

	/* send a command */
	unsigned int wp;
	
	/* add command to corb */
	regs->lock();
	wp = regs->read8(HDA_CORBWP);
	wp++;
	wp %= MAX_CORB_ENTRIES;
	
	rirb.cmds++;
	OSWriteLittleInt32(corb.vaddr, wp * sizeof(UInt32), command);
	regs->write32(HDA_CORBWP, wp);
	regs->unlock();
	
	return true;
}

void HDACommandTransmitter::updateRirb() {

	/* retreive RIRB entry - called from interrupt handler */
	unsigned int rp, wp;
	UInt32 res, res_ex;

	regs->lock();
	wp = regs->read8(HDA_RIRBWP);
	if (wp == rirb.wp) {
		regs->unlock();
		return;
	}
	rirb.wp = wp;
	
	while (rirb.rp != wp) {
		rirb.rp++;
		rirb.rp %= MAX_RIRB_ENTRIES;

		rp = rirb.rp << 1; /* an RIRB entry is 8-bytes */
		res_ex = OSReadLittleInt32(rirb.vaddr, (rp + 1) * sizeof(UInt32));
		res = OSReadLittleInt32(rirb.vaddr, rp * sizeof(UInt32));
		if (res_ex & RIRB_EX_UNSOL_EV) {
			// TODO: process unsolicited event
			//snd_hda_queue_unsol_event(chip->bus, res, res_ex);
			// I hope there are not a lot of them, really
			//HDACommandResponse *unsolicitedResponse = HDACommandResponse::withValue(res, res_ex);
			//unsolicitedResponses->returnCommand(unsolicitedResponse);
			// WHOW!!! That is great news. All needed information contains in res_ex! Including codec address and much more!!!
			++unsolicited;
		}
		else if (rirb.cmds) {
			rirb.res = res;
			rirb.cmds--;
		}
	}
	regs->unlock();
}

UInt32 HDACommandTransmitter::rirbGetResponse() {

	/* receive a response */
	bool pollingMode = false;

	for ( ; ; ) {
		int timeout = 100;
		while (timeout--) {
			if (pollingMode) {
				updateRirb();
			}
			if (!rirb.cmds)
				return rirb.res;	/* the last value */
			IOSleep(2);
		}
		if (!pollingMode) {
			IOLog("HDACommandTransmitter[%p]::rirbGetResponse() timeout. switching to polling mode\n", this);
			pollingMode = true;
			continue;				// try again
		}
		break;
	}

	IOLog("HDACommandTransmitter[%p]::rirbGetResponse() timeout switching to singleCommand mode\n", this);
	
	rirb.rp = regs->read8(HDA_RIRBWP);
	rirb.cmds = 0;

//	singleMode = true;

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

bool HDACommandTransmitter::singleSendCommand(UInt32 command) {

	/* send a command */
	int timeout = 50;
	
	while (timeout--) {
		/* check ICB busy bit */
		if (!(regs->read16(HDA_IRS) & IRS_BUSY)) {
			/* Clear IRV valid bit */
			regs->write16(HDA_IRS, regs->read16(HDA_IRS) | IRS_VALID);
			regs->write32(HDA_IC, command);
			regs->write16(HDA_IRS, regs->read16(HDA_IRS) | IRS_BUSY);
			return true;
		}
		IODelay(1);
	}
	IOLog("HDACommandTransmitter[%p]::singleSendCommand(%x) timeout\n", this, (unsigned int)command);

	return false;
}

UInt32 HDACommandTransmitter::singleGetResponse() {

	/* receive a response */
	int timeout = 50;

	while (timeout--) {
		/* check IRV busy bit */
		if (regs->read8(HDA_IRS) & IRS_VALID)
			return regs->read32(HDA_IR);
		IODelay(1);
	}
	IOLog("HDACommandTransmitter[%p]::singleGetResponse() failed\n", this);

	return (UInt32)-1;
}

bool HDACommandTransmitter::sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para) {

	/* send a command */
/*	UInt32 val;

	val = (UInt32)(addr & 0x0f) << 28;
	val |= (UInt32)direct << 27;
	val |= (UInt32)nid << 20;
	val |= verb << 8;
	val |= para & 0xffff;

	if (singleMode)
		return singleSendCommand(val);
	return corbSendCommand(val);*/

	HDACommand command;
	command.addr = addr;
	command.nid = nid;
	command.direct = direct;
	command.verb = verb;
	command.para = para;
	
	return commandGate->runAction(sendCommandAction, &command, NULL, NULL, NULL) == kIOReturnSuccess;
	
}

bool HDACommandTransmitter::sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para, unsigned *result) {

/*	if (!sendCommand(addr, nid, direct, verb, para))
		return false;
	
	*result = getResponse();

	return true;*/

	HDACommand command;
	command.addr = addr;
	command.nid = nid;
	command.direct = direct;
	command.verb = verb;
	command.para = para;
	
	if (commandGate->runAction(sendCommandAndGetResponseAction, &command, NULL, NULL, NULL) == kIOReturnSuccess)
	{
		*result = command.result;
		return true;
	}
	return false;
}

UInt32 HDACommandTransmitter::getResponse() {

	/* get a response */
	if (singleMode)
		return singleGetResponse();
	return rirbGetResponse();
}


IOReturn HDACommandTransmitter::sendCommandAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
	HDACommandTransmitter *commandTransmitter = OSDynamicCast(HDACommandTransmitter, owner);

	if (!commandTransmitter)
		return kIOReturnBadArgument;
	HDACommand *command = (HDACommand*)arg0;
	
	return commandTransmitter->performSendCommand(command) ? kIOReturnSuccess : kIOReturnError;
}

IOReturn HDACommandTransmitter::sendCommandAndGetResponseAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
	HDACommandTransmitter *commandTransmitter = OSDynamicCast(HDACommandTransmitter, owner);
	
	if (!commandTransmitter)
		return kIOReturnBadArgument;
	HDACommand *command = (HDACommand*)arg0;
	
	if (!commandTransmitter->performSendCommand(command))
		return kIOReturnError;

	command->result = commandTransmitter->performGetResponse();
	return kIOReturnSuccess;
}

bool HDACommandTransmitter::performSendCommand(HDACommand *command) {
	
	/* send a command */
	UInt32 val;
	
	val = (UInt32)(command->addr & 0x0f) << 28;
	val |= (UInt32)command->direct << 27;
	val |= (UInt32)command->nid << 20;
	val |= command->verb << 8;
	val |= command->para & 0xffff;
	
	if (singleMode)
		return singleSendCommand(val);
	return corbSendCommand(val);
}

UInt32 HDACommandTransmitter::performGetResponse() {
	
	/* get a response */
	if (singleMode)
		return singleGetResponse();
	return rirbGetResponse();
}
