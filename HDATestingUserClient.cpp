/*
 *  HDATestingUserClient.cpp
 *  OpenHDAX
 *
 *  Created by юрий гагарин on 1/6/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDATestingUserClient.h"

#include <IOKit/IOLib.h>
#include <IOKit/assert.h>

/* 
 * Define the metaclass information that is used for runtime
 * typechecking of IOKit objects. We're a subclass of IOUserClient.
 */

#define super IOUserClient
OSDefineMetaClassAndStructors( HDATestingUserClient, IOUserClient );

/* 
 * Since this sample uses the IOUserClientClass property, the AppleSamplePCIUserClient
 * is created automatically in response to IOServiceOpen(). More complex applications
 * might have several kinds of clients each with a different IOUserClient subclass,
 * with different enumerated types. In that case the AppleSamplePCI class must implement
 * the newUserClient() method (see IOService.h headerdoc).
 */

bool HDATestingUserClient::initWithTask( task_t owningTask, void * securityID,
                                             UInt32 type )
{
    IOLog("HDATestingUserClient::initWithTask(type %ld)\n", type);
    
    fTask = owningTask;

	if ( super::initWithTask( owningTask, securityID, type ) )
		return true;

	IOLog("super::initWithTask failed\n");

    return false;
}

bool HDATestingUserClient::start( IOService * provider )
{
    IOLog("HDATestingUserClient::start\n");

    if( !super::start( provider )) {
		IOLog("super::start failed\n");
        return( false );
	}

    /*
     * Our provider is the AppleSamplePCI object.
     */

    assert( OSDynamicCast( HDAController, provider ));
    fDriver = (HDAController *) provider;
    fOpenCount = 1;

    return( true );
}


/*
 * Kill ourselves off if the client closes its connection or the client dies.
 */

IOReturn HDATestingUserClient::clientClose( void )
{

	IOLog("clientClose\n");

    if( !isInactive())
        terminate();

    return( kIOReturnSuccess );
}

/* 
 * stop will be called during the termination process, and should free all resources
 * associated with this client.
 */
void HDATestingUserClient::stop( IOService * provider )
{
    IOLog("HDATestingUserClient::stop\n");

    super::stop( provider );
}

/*
 * Lookup the external methods - supply a description of the parameters 
 * available to be called 
 */

IOExternalMethod * HDATestingUserClient::getTargetAndMethodForIndex(
                                                    IOService ** targetP, UInt32 index )
{
    static const IOExternalMethod methodDescs[kHDANumMethods] = {

      { NULL, (IOMethod) &HDATestingUserClient::sendCommand,
        kIOUCStructIStructO, kIOUCVariableStructureSize, kIOUCVariableStructureSize },

    };

    *targetP = this;
    if( index < kHDANumMethods)
        return( (IOExternalMethod *)(methodDescs + index) );
    else
        return NULL;
}

IOReturn HDATestingUserClient::externalMethod(
	uint32_t selector, IOExternalMethodArguments * arguments,
	IOExternalMethodDispatch * dispatch, OSObject * target, void * reference )
{

    return (super::externalMethod(selector, arguments, NULL, this, NULL));

    IOReturn err;

    switch (selector)
    {
	case kHDASendCommand:
	    err = sendCommand( (HDACommandInput *) arguments->structureInput, 
			    (HDACommandOutput *)  arguments->structureOutput,
			    arguments->structureInputSize, (IOByteCount *) &arguments->structureOutputSize );
	    break;

	default:
	    err = kIOReturnBadArgument;
	    break;
    }

    IOLog("externalMethod(%d) 0x%x", selector, err);

    return (err);
}

/*
 * Implement each of the external methods described above.
 */

IOReturn HDATestingUserClient::sendCommand(
					   HDACommandInput * dataIn, HDACommandOutput * dataOut,
                                           IOByteCount inputSize, IOByteCount * outputSize )
{
    IOReturn	ret;
    IOItemCount	count;

	if ( inputSize < sizeof(HDACommandInput) )
		return kIOReturnNoSpace;

    IOLog("HDATestingUserClient::sendCommand( caddr=%d, nid=%d, direct=%d, verb=%d, parm=%d )\n", 
		dataIn->caddr, dataIn->nid, dataIn->direct, dataIn->verb, dataIn->parm);

    if( *outputSize < sizeof(int))
        return( kIOReturnNoSpace );

	unsigned result;
	
	HDACommandTransmitter *commandTransmitter = fDriver->getCommandTransmitter();
//	commandTransmitter->lock();
	if (!commandTransmitter->sendCommand(dataIn->caddr, dataIn->nid, dataIn->direct, dataIn->verb, dataIn->parm, &result))
		result = (UInt32)-1;
//	commandTransmitter->unlock();

	dataOut->result = result;

    *outputSize = sizeof( HDACommandOutput );

    return kIOReturnSuccess;
}
