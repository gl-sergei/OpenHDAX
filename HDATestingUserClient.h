/*
 *  HDATestingUserClient.h
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 1/6/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HDA_TESTING_USER_CLIENT__
#define __HDA_TESTING_USER_CLIENT__

#include "HDAController.h"
#include "HDACodecCommon.h"

#include <IOKit/IOUserClient.h>

class HDATestingUserClient : public IOUserClient
{
    /*
     * Declare the metaclass information that is used for runtime
     * typechecking of IOKit objects.
     */

    OSDeclareDefaultStructors( HDATestingUserClient );

private:
    HDAController *		fDriver;
    task_t			fTask;
    SInt32			fOpenCount;

public:
    /* IOService overrides */
    virtual bool start( IOService * provider );
    virtual void stop( IOService * provider );

    /* IOUserClient overrides */
    virtual bool initWithTask( task_t owningTask, void * securityID,
                                                UInt32 type );
    virtual IOReturn clientClose( void );

    virtual IOExternalMethod * getTargetAndMethodForIndex(
                                            IOService ** targetP, UInt32 index );

    virtual IOReturn externalMethod( uint32_t selector, IOExternalMethodArguments * arguments,
					IOExternalMethodDispatch * dispatch = 0, OSObject * target = 0, void * reference = 0 );

    /* External methods */
    virtual IOReturn sendCommand( HDACommandInput * dataIn, HDACommandOutput * dataOut,
                                                IOByteCount inputCount, IOByteCount * outputCount );
};

#endif
