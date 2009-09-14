/*
 *  HDACommandTransmitter.h
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 12/14/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include <IOKit/IOTypes.h>
#include <IOKit/IOCommand.h>
#include <IOKit/IOCommandPool.h>
#include <IOKit/IOCommandGate.h>

#include "HDAPCIRegisters.h"
#include "dma.h"

#ifndef __HDA_COMMAND_TRANSMITTER__
#define __HDA_COMMAND_TRANSMITTER__

/*
 * Thats it! We'll create IOCommandPool, for queuing unsolicited responces. We'll use returnCommand from updateRirb and will
 * fetch and dispatch them by calling getCommand(true) from separate thread. We'll create thread by calling IOCreateThread.
 * That is simple!
 * Нужно подумать над реорганизацией обработчика прерываний прежде чем обрабатывать неожиданные ответы кодеков.
 *
 * Идея трансформируется в следующую:
 * На unsolicited response создаем поток, в котором выполняем вызов обработчика через IOCommandGate. Порядок в общем случае
 * это нам не обеспечит, но возиться с очередями придется меньше. Пример есть в ethernet-драйвере в модуле управления питанием.
 */



/*
 * 1. Нужен механизм создания команд разных видов (пока 2, но кто знает сколько их?). Выяснить что такое direct.
 *    сейчас direct передается всегда 0, а в спецификации ACL260 бит 27 это часть nid, а не отдельная смысловая единица.
 * 2. Нужен механизм распознавания ответов. Во-первых нужно отделять ожиданные ответы от неожиданных, во-вторых механизм
 *    фильтрации и поиска совпадений ответов для вызова вызова зарегистрированных обработчиков неожиданных ответов, в третьих
 *    нужно как-то связывать команду с ответом чтобы представлять его в удобном для анализа виде.
 * 3. Нужен механизм маршрутизации команд и ответов. (Очереди, регистрация обработчиков, etc.) ОТВЕТЫ ОТ ?ОТДЕЛЬНОГО КОДЕКА? ПРИХОДЯТ
 *    В ТОМ ЖЕ ПОРЯДКЕ ЧТО И КОМАНДЫ. Нет способа отличить от какого кодека пришел ответ.
 *    Отправка команды должна возвращаться сразу же, оставив ссылку на обработчик ответа. Часто, однако,
 *    ответ нужен клиенту немедленно, поэтому необходим простой интерфейс вызова синхронного вызова команды. (Реализовать это через
 *    асинхронный вызов или сделать специальный синхронный интерфейс?).
 *    По спецификации кодек должен в любом случае вернуть ответ на команду. В реальности же нужно вводить TTL для команд.
 */

class HDACommandResponse : public IOCommand {

	OSDeclareDefaultStructors(HDACommandResponse)

private:
	UInt32 value;
	UInt32 value_ex;

public:

	virtual bool init(UInt32 val, UInt32 val_ex);
	static HDACommandResponse *withValue(UInt32 val, UInt32 val_ex);
	virtual void free();

	virtual UInt32 getValue();
	virtual UInt32 getValueEx();

};

struct RINGBUFFER {
	PH64 phaddr;			// physical address
	UInt32 *vaddr;			// virtual address
	UInt16 rp, wp;			// read/write pointers
	int cmds;				// number of pending requests
	UInt32 res;				// last read value
};

class HDACommandTransmitter : public OSObject {

	OSDeclareDefaultStructors(HDACommandTransmitter);

	HDADMABuffer *commandBuffer;			// DMA buffer for CORB and RIRB operations
	HDAPCIRegisters *regs;					// PCI registers interface
	IOWorkLoop *workLoop;					// our own work loop
	IOCommandGate *commandGate;				// Command Gate for sync
	RINGBUFFER rirb, corb;					// our view of corb/rirb
	int unsolicited;						// unsolicited responce counter
	bool singleMode;						// indicates BIOS legacy mode
//	IOCommandPool *unsolicitedResponses;	// command pool to manage unsolicited responses

	/* here we need response queue here */
	/* and something in workLoop to process unsolicited responses */

	/* low level CORB/RIRB handling */
	virtual bool corbSendCommand(UInt32 command);				// actual send command
	virtual UInt32 rirbGetResponse();							// actual receive response
	virtual UInt32 getResponse();
	
	/* BIOS legacy mode */
	virtual bool singleSendCommand(UInt32 command);
	virtual UInt32 singleGetResponse();

	struct HDACommand {
		unsigned int addr;
		UInt16 nid;
		int direct;
		unsigned int verb;
		unsigned int para;
		unsigned result;
	};
	
	virtual bool performSendCommand(HDACommand *command);
	virtual UInt32 performGetResponse();
	
public:

	/* construction and destruction */
	virtual bool init(HDAPCIRegisters *registers);
	static HDACommandTransmitter *withPCIRegs(HDAPCIRegisters *registers);
	virtual void free();

	virtual void initHardware();										// must call this before start
	virtual void start();												// start corb/rirb dma (NOT STARTED BY DEFAULT!)
	virtual void stop();												// stop corb/rirb dma

	/* recommended command send interface */
	virtual bool sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para);
	virtual bool sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para, unsigned *result);

	// should be called by interrupt handler only
	virtual void updateRirb();											// update RIRB responce when interrupt happend
	
	static IOReturn sendCommandAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
	static IOReturn sendCommandAndGetResponseAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
	
//	void lock();
//	void unlock();

};

#endif
