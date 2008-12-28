/*
 *  HDACommandTransmitter.h
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 12/14/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include <IOKit/IOTypes.h>

#include "HDAPCIRegisters.h"
#include "dma.h"

#ifndef __HDA_COMMAND_TRANSMITTER__
#define __HDA_COMMAND_TRANSMITTER__


/*
 * 1. Нужен механизм содания команд разных видов (пока 2, но кто знает сколько их?). Выяснить что такое direct.
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

/*
 * General HDA Command class
 */
class HDACommand {

private:
	UInt32 command32;

public:
	HDACommand(UInt32 command) {
		command32 = command;
	}

	virtual UInt32 getCommand32() const {
		return command32;
	}
};

class HDACommandResponse {

private:
	UInt32 response32;

public:
	HDACommandResponse(UInt32 response) {
		response32 = response;
	}

	UInt32 getResponse32() {
		return response32;
	}
};

class HDASolicitedResponse : public HDACommandResponse {
};

class HDAUnsolicitedResponse : public HDACommandResponse {
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
	IOLock *mutex;							// mutex
	HDAPCIRegisters *regs;					// PCI registers interface
	RINGBUFFER rirb, corb;					// our view of corb/rirb
	int unsolicited;						// unsolicited responce counter
	bool singleMode;						// indicates BIOS legacy mode

	/* here we need response queue here */
	/* and something in workLoop to process unsolicited responses */

public:

	/* construction and destruction */
	virtual bool init(HDAPCIRegisters *registers);
	static HDACommandTransmitter *withPCIRegs(HDAPCIRegisters *registers);
	virtual void free();

	void initHardware();										// must call this before start
	void start();												// start corb/rirb dma (NOT STARTED BY DEFAULT!)
	void stop();												// stop corb/rirb dma

	/* low level CORB/RIRB handling */
	virtual bool corbSendCommand(UInt32 command);				// actual send command
	virtual void updateRirb();									// update RIRB responce when interrupt happend
	virtual UInt32 rirbGetResponse();							// actual receive response

	/* BIOS legacy mode */
	bool singleSendCommand(UInt32 command);
	UInt32 singleGetResponse();

	/* recommended command send interface */
	bool sendCommand(unsigned int addr, UInt16 nid, int direct, unsigned int verb, unsigned int para);
	UInt32 getResponse();

	void lock();
	void unlock();

};

#endif
