/*
 *  HDAAudioStream.cpp
 *  OpenHDAX
 *
 *  Created by Юрий Гагарин on 3/29/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDAAudioStream.h"

#define super IOAudioStrem

OSDefineMetaClassAndStructors(HDAAudioStream, IOAudioStream)

/*
 * Нужно
 * 1. Выделить ресурсы контроллера (память, tag)
 * 2. Опросить и настроить nid
 * 3. Подготовить stream к работе
 */
bool HDAAudioStream::init(HDAController *controller, HDACodec *codec, unsigned nid) {

	this->codec = codec;
	this->controller = controller;
	this->nid = nid;

	return true;

}

void HDAAudioStream::free() {
}
