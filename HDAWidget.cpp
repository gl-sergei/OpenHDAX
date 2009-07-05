/*
 *  HDAWidget.cpp
 *  HDATest
 *
 *  Created by юрий гагарин on 2/1/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "HDAWidget.h"

#include "HDACodecCommon.h"

bool HDAAudioWidget::codecWrite(int direct, unsigned verb, unsigned parm) {
	bool result;
	
	commandTransmitter->lock();
	result = commandTransmitter->sendCommand(codecAddress, nid, direct, verb, parm);
	commandTransmitter->unlock();

	return result;
}

unsigned HDAAudioWidget::codecRead(int direct, unsigned verb, unsigned parm) {

	unsigned result;

	commandTransmitter->lock();
	if (commandTransmitter->sendCommand(codecAddress, nid, direct, verb, parm))
		result = commandTransmitter->getResponse();
	else
		result = (UInt32)-1;
	commandTransmitter->unlock();

	return result;
}

unsigned HDAAudioWidget::paramRead(unsigned param) {
	return codecRead(0, AC_VERB_PARAMETERS, param);
}

unsigned HDAAudioWidget::getAudioWidgetCapabilities() {
	return paramRead(AC_PAR_AUDIO_WIDGET_CAP);
}

unsigned HDAAudioWidget::getSupportedPCMSizeAndRates() {
	return paramRead(AC_PAR_PCM);
}

unsigned HDAAudioWidget::getSupportedStreamFormats() {
	return paramRead(AC_PAR_STREAM);
}

unsigned HDAAudioWidget::getPinCapabilities() {
	return paramRead(AC_PAR_PIN_CAP);
}

unsigned HDAAudioWidget::getAmplifierInputCapabilities() {
	return paramRead(AC_PAR_AMP_IN_CAP);
}

unsigned HDAAudioWidget::getAmplifierOutputCapabilities() {
	return paramRead(AC_PAR_AMP_OUT_CAP);
}

unsigned HDAAudioWidget::getConnectionListLen() {
	return paramRead(AC_PAR_CONNLIST_LEN);
}

unsigned HDAAudioWidget::getConnectionListEntry(unsigned offsetIndex) {
	return codecRead(0, AC_VERB_GET_CONNECT_LIST, offsetIndex);
}

unsigned HDAAudioWidget::getPinConfigurationDefault() {
	return codecRead(0, AC_VERB_GET_CONFIG_DEFAULT, 0);
}

unsigned HDAAudioWidget::getPinSense() {
	return codecRead(0, AC_VERB_GET_PIN_SENSE, 0);
}

unsigned HDAAudioWidget::getConnectionSelectControl() {
	return codecRead(0, AC_VERB_GET_CONNECT_SEL, 0);
}

bool HDAAudioWidget::setConverterStreamChannel(unsigned streamId, unsigned channelId)
{
	return codecWrite(0, AC_VERB_SET_CHANNEL_STREAMID, (streamId << 4) | channelId);
}

/*
 * Set amplifier gain/mute
 * params - combination of AC_AMP_SET_RIGHT | AC_AMP_SET_LEFT | AC_AMP_SET_INPUT | AC_AMP_SET_OUTPUT
 * index  - only used when programming the input amplifiers on Selector Widgets and Sum Widgets, 
 *          where each input may have an individual amplifier.  The index corresponds to the input’s offset in 
 *          the Connection List
 * gain   - gain value
 */
bool HDAAudioWidget::setAmplifierGainMute(unsigned params, unsigned index, unsigned gain)
{
	return codecWrite(0, AC_VERB_SET_AMP_GAIN_MUTE, params | (index << AC_AMP_SET_INDEX_SHIFT) | (gain & (AC_GAIN_MAX | AC_AMP_MUTE)));
}

bool HDAAudioWidget::setStreamFormat(unsigned format)
{
	return codecWrite(0, AC_VERB_SET_STREAM_FORMAT, format);
}

unsigned HDAAudioWidget::getPinControl()
{
	return codecRead(0, AC_VERB_GET_PIN_WIDGET_CONTROL, 0);
}

bool HDAAudioWidget::setPinControl(unsigned control)
{
	return codecWrite(0, AC_VERB_SET_PIN_WIDGET_CONTROL, control);
}

HDAAudioWidget::PinSense HDAAudioWidget::measurePinSense() {
	PinSense ps;
	ps.pinSense = getPinSense();
	return ps;
}

bool HDAAudioWidget::isSelector() {
	return fSelector;
}

unsigned HDAAudioWidget::getCurrentSelection() {
	return getConnectionSelectControl();
}

void HDAAudioWidget::init(HDACommandTransmitter *commandTransmitter, unsigned codecAddress, unsigned nid) {

	this->commandTransmitter = commandTransmitter;
	this->codecAddress = codecAddress;
	this->nid = nid;

	commandTransmitter->retain();

	// This is audio widget. Lets determine its capabilities
	wcaps.wcaps = getAudioWidgetCapabilities();

	// Widget specific capabilities
	switch (wcaps.type) {
	case audioOutput:
	case audioInput:
		pcmcaps.pcmcaps = getSupportedPCMSizeAndRates();
		streamFormats.streamFormats = getSupportedStreamFormats();
		break;
	case audioMixer:
		break;
	case audioSelector:
		break;
	case pinComplex:
		pincaps.pincaps = getPinCapabilities();
		configDefault.configDefault = getPinConfigurationDefault();
		break;
	}

	// Amplifier capabilities
	if (wcaps.in_amp_present)
		amp_in_caps.ampcaps = getAmplifierInputCapabilities();
	if (wcaps.out_amp_present)
		amp_out_caps.ampcaps = getAmplifierOutputCapabilities();

	// Construct connection list
	connected_nids = NULL;
	number_of_connected_nids = getConnectionListLen();

	if (number_of_connected_nids > 0)
		connected_nids = new unsigned[number_of_connected_nids];

	int offset = 0;
	for ( offset = 0; connected_nids && offset < number_of_connected_nids; offset += 4) {
		unsigned list = getConnectionListEntry(offset);
		unsigned idx;
		for ( idx = 0; idx < 4 && list; idx++) {
			connected_nids[offset + idx] = list & 0xff;
			list = list >> 8;
		}
	}

	if (!connected_nids) number_of_connected_nids = 0;

	fSelector = false;
	if (number_of_connected_nids > 0)	// possible selector
		fSelector = (wcaps.type == audioSelector) || (wcaps.type == audioInput) || (wcaps.type == pinComplex);

	rootWidget = NULL;
	predcessor = NULL;
	pathLength = 0;
}

HDAAudioWidget::~HDAAudioWidget() {
	if (connected_nids)
		delete [] connected_nids;
	commandTransmitter->release();
}

void HDAAudioWidget::print() {
	const char *typeStr[] = {
		"Audio Output", 
		"Audio Input", 
		"Audio Mixer", 
		"Audio Selector",  
		"Pin Complex", 
		"Power Widget", 
		"Volume Knob Widget", 
		"Beep Generator Widget", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Reserved", 
		"Vendor defined audio widget" 
	};

	IOLog("nid = 0x%x (%s)\n", nid, typeStr[wcaps.type]);

	if (wcaps.stereo) IOLog("\tstereo\n");
	if (wcaps.in_amp_present) {
		IOLog("\tin amp present:");
		if (amp_in_caps.mute_capable) IOLog(" mute,");
		IOLog(" step size = %d, num steps = %d, offset = %d\n", amp_in_caps.step_size, amp_in_caps.num_steps, amp_in_caps.offset);
	}
	if (wcaps.out_amp_present) {
		IOLog("\tout amp present:");
		if (amp_out_caps.mute_capable) IOLog(" mute,");
		IOLog(" step size = %d, num steps = %d, offset = %d\n", amp_out_caps.step_size, amp_out_caps.num_steps, amp_out_caps.offset);
	}
	if (wcaps.amp_param_override) IOLog("\tamp param override\n");
	if (wcaps.format_override) {
		IOLog("\tformat override:");
		if (pcmcaps.sr8000) IOLog(" 8kHz");
		if (pcmcaps.sr11025) IOLog(" 11.025kHz");
		if (pcmcaps.sr16000) IOLog(" 16kHz");
		if (pcmcaps.sr22050) IOLog(" 22.050kHz");
		if (pcmcaps.sr32000) IOLog(" 32kHz");
		if (pcmcaps.sr44100) IOLog(" 44.1kHz");
		if (pcmcaps.sr48000) IOLog(" 48kHz");
		if (pcmcaps.sr88200) IOLog(" 88.2kHz");
		if (pcmcaps.sr96000) IOLog(" 96kHz");
		if (pcmcaps.sr176400) IOLog(" 176.4kHz");
		if (pcmcaps.sr192000) IOLog(" 192kHz");
		if (pcmcaps.sr384000) IOLog(" 384kHz");
		if (pcmcaps.br8) IOLog(" 8bit");
		if (pcmcaps.br16) IOLog(" 16bit");
		if (pcmcaps.br20) IOLog(" 20bit");
		if (pcmcaps.br24) IOLog(" 24bit");
		if (pcmcaps.br32) IOLog(" 32bit");
		if (streamFormats.pcm) IOLog(" PCM");
		if (streamFormats.float32) IOLog(" float32");
		if (streamFormats.ac3) IOLog(" AC3");
		IOLog("\n");
	}
	if (wcaps.stripe) IOLog("\tstripe\n");
	if (wcaps.proc_widget) IOLog("\tproc widget\n");
	if (wcaps.unsol_capable) IOLog("\tunsol capable\n");
	if (wcaps.conn_list) {
		IOLog("\tconn list:");
		for (int i = 0; i < number_of_connected_nids; i++)
			IOLog(" 0x%x", connected_nids[i]);
		IOLog("\n");
	}
	if (wcaps.digital) IOLog("\tdigital\n");
	if (wcaps.power_cntrl) IOLog("\tpower cntrl\n");
	if (wcaps.l_r_swap) IOLog("\tl r swap\n");
	IOLog("\tdelay: 0x%x\n", wcaps.delay);
	if (wcaps.type == pinComplex) {
		const char *portConnectivityStr[] = { "The Port Complex is connected to a jack (1/8\", ATAPI, etc.)",
										"No physical connection for Port",
										"A fixed function device (integrated speaker, integrated mic, etc.) is attached",
										"Both a jack and an internal device are attached" };
		const char * defaultDeviceStr[] = {"Line Out",
									"Speaker",
									"HP Out",
									"CD",
									"SPDIF Out",
									"Digital Other Out",
									"Modem Line Side",
									"Modem Handset Side",
									"Line In",
									"AUX",
									"Mic In",
									"Telephony",
									"SPDIF In",
									"Digital Other In",
									"Reserved",
									"Other"	};
	const char * connectionTypeStr[] = {"Unknown",
									"1/8\" stereo/mono",
									"1/4\" stereo/mono",
									"ATAPI internal",
									"RCA",
									"Optical",
									"Other Digital",
									"Other Analog",
									"Multichannel Analog (DIN)",
									"XLR/Professional",
									"RJ-11 (Modem)",
									"Combination",
									"Other"};
	const char * colorStr[] = {		"Unknown",
									"Black",
									"Grey",
									"Blue",
									"Green",
									"Red",
									"Orange",
									"Yellow",
									"Purple",
									"Pink",
									"Reserved",
									"Reserved",
									"Reserved",
									"Reserved",
									"White",
									"Other"};

		PinSense pinSense;
		if (pincaps.impendance_sense_capable || pincaps.presence_detect_capabe)
			pinSense.pinSense = getPinSense();

		if (pincaps.impendance_sense_capable) IOLog("\timpendance sense capable (%u)\n", pinSense.impendance);
		if (pincaps.trigger_required) IOLog("\ttrigger required\n");
		if (pincaps.presence_detect_capabe) IOLog("\tpresence detect capabe (%s)\n", pinSense.presence ? "present" : "not present");
		if (pincaps.headphone_drive_capable) IOLog("\theadphone drive capable\n");
		if (pincaps.output_capable) IOLog("\toutput capable\n");
		if (pincaps.input_capable) IOLog("\tinput capable\n");
		if (pincaps.balanced_io_pins) IOLog("\tbalanced io pins\n");
		if (pincaps.vref_control) IOLog("\tvref control\n");
		if (pincaps.eapd_capable) IOLog("\teapd capable\n");

		IOLog("\tport connectivity: %s\n", portConnectivityStr[configDefault.portConnectivity]);
		IOLog("\tdefault device: %s\n", defaultDeviceStr[configDefault.defaultDevice]);
		IOLog("\tconnection type: %s\n", connectionTypeStr[configDefault.connectionType]);
		IOLog("\tcolor: %s\n", colorStr[configDefault.color]);
		IOLog("\tdefault association: 0x%x\n", configDefault.defaultAssociation);
		IOLog("\tsequence: 0x%x\n", configDefault.sequence);

	}
}
