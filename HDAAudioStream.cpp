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

bool HDAAudioStream::init(HDAController *controller, HDACodec *codec, unsigned nid) {

	return true;

}

void HDAAudioStream::free() {
}
