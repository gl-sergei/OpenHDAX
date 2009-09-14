/*
 *  HDAWidget.h
 *  HDATest
 *
 *  Created by юрий гагарин on 2/1/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __HDA_WIDGET__
#define __HDA_WIDGET__

#include "HDAController.h"

/*
 * High Definition Audio Widget abstraction
 */

class HDAAudioWidget {

public:

	// Audio widget functionality
	enum Type {
		audioOutput = 0x0,
		audioInput,
		audioMixer,
		audioSelector,
		pinComplex,
		powerWidget,
		volumeKnobWidget,
		beepGeneratorWidget,
		reserved8 = 0x8,
		reserved9,
		reservedA,
		reservedB,
		reservedC,
		reservedD,
		reservedE,
		vendorDefined = 0xf
	};

	// Audio widget capabilities
	union Wcaps {
		struct {
			unsigned stereo:1;
			unsigned in_amp_present:1;
			unsigned out_amp_present:1;
			unsigned amp_param_override:1;
			unsigned format_override:1;
			unsigned stripe:1;
			unsigned proc_widget:1;
			unsigned unsol_capable:1;
			unsigned conn_list:1;
			unsigned digital:1;
			unsigned power_cntrl:1;
			unsigned l_r_swap:1;
			unsigned reserved1:4;
			unsigned delay:4;
			Type type:4;
			unsigned reserved2:8;
		};
		UInt32 wcaps;
	};

	// PCM size, rates
	union PCMCaps {
		struct {
			unsigned sr8000:1;
			unsigned sr11025:1;
			unsigned sr16000:1;
			unsigned sr22050:1;
			unsigned sr32000:1;
			unsigned sr44100:1;
			unsigned sr48000:1;
			unsigned sr88200:1;
			unsigned sr96000:1;
			unsigned sr176400:1;
			unsigned sr192000:1;
			unsigned sr384000:1;
			unsigned reserved1:4;
			unsigned br8:1;
			unsigned br16:1;
			unsigned br20:1;
			unsigned br24:1;
			unsigned br32:1;
			unsigned reserved2:11;
		};
		UInt32 pcmcaps;
	};

	// Supported stream formats
	union StreamFormats {
		struct {
			unsigned pcm:1;
			unsigned float32:1;
			unsigned ac3:1;
			unsigned reserved:29;
		};
		UInt32 streamFormats;
	};

	// Pin capabilities
	union PinCaps {
		struct {
			unsigned impendance_sense_capable:1;
			unsigned trigger_required:1;
			unsigned presence_detect_capabe:1;
			unsigned headphone_drive_capable:1;
			unsigned output_capable:1;
			unsigned input_capable:1;
			unsigned balanced_io_pins:1;
			unsigned reserved1:1;
			unsigned vref_control:8;
			unsigned eapd_capable:1;
			unsigned reserved2:15;
		};
		UInt32 pincaps;
	};

	// Amplifier capabilities
	union AmpCaps {
		struct {
			unsigned offset:7;
			unsigned reserved1:1;
			unsigned num_steps:7;
			unsigned reserved2:1;
			unsigned step_size:7;
			unsigned reserved3:8;
			unsigned mute_capable:1;
		};
		UInt32 ampcaps;
	};

	enum PortConnectivity {
		portcJack = 0,
		portcNone,
		portcFixed,
		portcBoth
	};

	enum DefaultDevice {
		devLine = 0x0,
		devSpeaker,
		devHPOut,
		devCD,
		devSPDIFOut,
		devDigitalOtherOut,
		devModemLineSide,
		devModemHandsetSide,
		devLineIn,
		devAUX,
		devMicIn,
		devTelephony,
		devSPDIFIn,
		devDigitalOtherIn,
		devReserved = 0xe,
		devOther = 0xf
	};

	enum ConnectionType {
		contUnknown = 0x0,
		conHalfQuarterInchStrereoMono,
		conQuarterInchStrereoMono,
		conATAPIInternal,
		conRCA,
		conOptical,
		conOtherDigital,
		conOtherAnalog,
		conMultichannelAnalog,
		conXLRProfessional,
		conRJ11,
		conCombination,
		conOther
	};

	enum Color {
		unknownColor = 0x0,
		black,
		grey,
		blue,
		green,
		red,
		orange,
		yellow,
		purple,
		pink,
		reserved_colorA,
		reserved_colorB,
		reserved_colorC,
		reserved_colorD,
		white,
		otherColor
	};

	// Configuration default
	union ConfigDefault {
		struct {
			unsigned sequence:4;
			unsigned defaultAssociation:4;
			unsigned misc:4;
			Color color:4;
			ConnectionType connectionType:4;
			DefaultDevice defaultDevice:4;
			unsigned location:6;
			PortConnectivity portConnectivity:2;
		};
		UInt32 configDefault;
	};

	union PinSense {
		struct {
			unsigned impendance:31;
			unsigned presence:1;
		};
		UInt32 pinSense;
	};

	unsigned nid;								// nid of widget
	unsigned codecAddress;						// address of codec
	HDACommandTransmitter *commandTransmitter;	// HDA controller

	unsigned *connected_nids;
	unsigned number_of_connected_nids;

	Wcaps wcaps;
	PCMCaps pcmcaps;
	StreamFormats streamFormats;
	PinCaps pincaps;
	AmpCaps amp_in_caps, amp_out_caps;
	ConfigDefault configDefault;
	PinSense pinSense;
	bool fSelector;

	HDAAudioWidget *rootWidget;
	HDAAudioWidget *predcessor;
	int pathLength;

protected:

	virtual bool codecWrite(int direct, unsigned verb, unsigned parm);
	virtual unsigned codecRead(int direct, unsigned verb, unsigned parm);
	virtual unsigned paramRead(unsigned param);

public:
	virtual unsigned getAudioWidgetCapabilities();
	virtual unsigned getSupportedPCMSizeAndRates();
	virtual unsigned getSupportedStreamFormats();
	virtual unsigned getPinCapabilities();
	virtual unsigned getAmplifierInputCapabilities();
	virtual unsigned getAmplifierOutputCapabilities();
	virtual unsigned getConnectionListLen();
	virtual unsigned getConnectionListEntry(unsigned offsetIndex);
	virtual unsigned getPinConfigurationDefault();
	virtual unsigned getPinSense();
	virtual unsigned getConnectionSelectControl();

	virtual bool setConverterStreamChannel(unsigned streamId, unsigned channelId);
	virtual bool setAmplifierGainMute(unsigned params, unsigned index, unsigned gain);
	virtual bool setStreamFormat(unsigned format);
	virtual unsigned getPinControl();
	virtual bool setPinControl(unsigned control);

public:
	virtual void init(HDACommandTransmitter *commandTransmitter, unsigned codecAddress, unsigned nid);
	virtual void print();

	virtual PinSense measurePinSense();
	
	// for Selectors
	virtual bool isSelector();
	virtual unsigned getCurrentSelection();

	virtual ~HDAAudioWidget();
	HDAAudioWidget();
};

#endif
