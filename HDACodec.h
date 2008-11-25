/*
 *  Intel High Definition Audio Device Driver for OS X
 *
 *  Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 *  Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 *  was a great sample and probably start point for this driver.
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __HDACODEC_H__
#define __HDACODEC_H__

#include <IOKit/audio/IOAudioEngine.h>
#include "HDAController.h"


/*
 * nodes
 */
#define	AC_NODE_ROOT		0x00

/*
 * function group types
 */
enum {
	AC_GRP_AUDIO_FUNCTION = 0x01,
	AC_GRP_MODEM_FUNCTION = 0x02,
};
	
/*
 * widget types
 */
enum {
	AC_WID_AUD_OUT,		/* Audio Out */
	AC_WID_AUD_IN,		/* Audio In */
	AC_WID_AUD_MIX,		/* Audio Mixer */
	AC_WID_AUD_SEL,		/* Audio Selector */
	AC_WID_PIN,		/* Pin Complex */
	AC_WID_POWER,		/* Power */
	AC_WID_VOL_KNB,		/* Volume Knob */
	AC_WID_BEEP,		/* Beep Generator */
	AC_WID_VENDOR = 0x0f	/* Vendor specific */
};

/*
 * GET verbs
 */
#define AC_VERB_GET_STREAM_FORMAT		0x0a00
#define AC_VERB_GET_AMP_GAIN_MUTE		0x0b00
#define AC_VERB_GET_PROC_COEF			0x0c00
#define AC_VERB_GET_COEF_INDEX			0x0d00
#define AC_VERB_PARAMETERS			0x0f00
#define AC_VERB_GET_CONNECT_SEL			0x0f01
#define AC_VERB_GET_CONNECT_LIST		0x0f02
#define AC_VERB_GET_PROC_STATE			0x0f03
#define AC_VERB_GET_SDI_SELECT			0x0f04
#define AC_VERB_GET_POWER_STATE			0x0f05
#define AC_VERB_GET_CONV			0x0f06
#define AC_VERB_GET_PIN_WIDGET_CONTROL		0x0f07
#define AC_VERB_GET_UNSOLICITED_RESPONSE	0x0f08
#define AC_VERB_GET_PIN_SENSE			0x0f09
#define AC_VERB_GET_BEEP_CONTROL		0x0f0a
#define AC_VERB_GET_EAPD_BTLENABLE		0x0f0c
#define AC_VERB_GET_DIGI_CONVERT_1		0x0f0d
#define AC_VERB_GET_DIGI_CONVERT_2		0x0f0e
#define AC_VERB_GET_VOLUME_KNOB_CONTROL		0x0f0f
/* f10-f1a: GPIO */
#define AC_VERB_GET_GPIO_DATA			0x0f15
#define AC_VERB_GET_GPIO_MASK			0x0f16
#define AC_VERB_GET_GPIO_DIRECTION		0x0f17
#define AC_VERB_GET_GPIO_WAKE_MASK		0x0f18
#define AC_VERB_GET_GPIO_UNSOLICITED_RSP_MASK	0x0f19
#define AC_VERB_GET_GPIO_STICKY_MASK		0x0f1a
#define AC_VERB_GET_CONFIG_DEFAULT		0x0f1c
/* f20: AFG/MFG */
#define AC_VERB_GET_SUBSYSTEM_ID		0x0f20

/*
 * SET verbs
 */
#define AC_VERB_SET_STREAM_FORMAT		0x200
#define AC_VERB_SET_AMP_GAIN_MUTE		0x300
#define AC_VERB_SET_PROC_COEF			0x400
#define AC_VERB_SET_COEF_INDEX			0x500
#define AC_VERB_SET_CONNECT_SEL			0x701
#define AC_VERB_SET_PROC_STATE			0x703
#define AC_VERB_SET_SDI_SELECT			0x704
#define AC_VERB_SET_POWER_STATE			0x705
#define AC_VERB_SET_CHANNEL_STREAMID		0x706
#define AC_VERB_SET_PIN_WIDGET_CONTROL		0x707
#define AC_VERB_SET_UNSOLICITED_ENABLE		0x708
#define AC_VERB_SET_PIN_SENSE			0x709
#define AC_VERB_SET_BEEP_CONTROL		0x70a
#define AC_VERB_SET_EAPD_BTLENABLE		0x70c
#define AC_VERB_SET_DIGI_CONVERT_1		0x70d
#define AC_VERB_SET_DIGI_CONVERT_2		0x70e
#define AC_VERB_SET_VOLUME_KNOB_CONTROL		0x70f
#define AC_VERB_SET_GPIO_DATA			0x715
#define AC_VERB_SET_GPIO_MASK			0x716
#define AC_VERB_SET_GPIO_DIRECTION		0x717
#define AC_VERB_SET_GPIO_WAKE_MASK		0x718
#define AC_VERB_SET_GPIO_UNSOLICITED_RSP_MASK	0x719
#define AC_VERB_SET_GPIO_STICKY_MASK		0x71a
#define AC_VERB_SET_CONFIG_DEFAULT_BYTES_0	0x71c
#define AC_VERB_SET_CONFIG_DEFAULT_BYTES_1	0x71d
#define AC_VERB_SET_CONFIG_DEFAULT_BYTES_2	0x71e
#define AC_VERB_SET_CONFIG_DEFAULT_BYTES_3	0x71f
#define AC_VERB_SET_CODEC_RESET			0x7ff

/*
 * Parameter IDs
 */
#define AC_PAR_VENDOR_ID		0x00
#define AC_PAR_SUBSYSTEM_ID		0x01
#define AC_PAR_REV_ID			0x02
#define AC_PAR_NODE_COUNT		0x04
#define AC_PAR_FUNCTION_TYPE		0x05
#define AC_PAR_AUDIO_FG_CAP		0x08
#define AC_PAR_AUDIO_WIDGET_CAP		0x09
#define AC_PAR_PCM			0x0a
#define AC_PAR_STREAM			0x0b
#define AC_PAR_PIN_CAP			0x0c
#define AC_PAR_AMP_IN_CAP		0x0d
#define AC_PAR_CONNLIST_LEN		0x0e
#define AC_PAR_POWER_STATE		0x0f
#define AC_PAR_PROC_CAP			0x10
#define AC_PAR_GPIO_CAP			0x11
#define AC_PAR_AMP_OUT_CAP		0x12
#define AC_PAR_VOL_KNB_CAP		0x13

/*
 * AC_VERB_PARAMETERS results (32bit)
 */

/* Function Group Type */
#define AC_FGT_TYPE			(0xff<<0)
#define AC_FGT_TYPE_SHIFT		0
#define AC_FGT_UNSOL_CAP		(1<<8)

/* Audio Function Group Capabilities */
#define AC_AFG_OUT_DELAY		(0xf<<0)
#define AC_AFG_IN_DELAY			(0xf<<8)
#define AC_AFG_BEEP_GEN			(1<<16)

/* Audio Widget Capabilities */
#define AC_WCAP_STEREO			(1<<0)	/* stereo I/O */
#define AC_WCAP_IN_AMP			(1<<1)	/* AMP-in present */
#define AC_WCAP_OUT_AMP			(1<<2)	/* AMP-out present */
#define AC_WCAP_AMP_OVRD		(1<<3)	/* AMP-parameter override */
#define AC_WCAP_FORMAT_OVRD		(1<<4)	/* format override */
#define AC_WCAP_STRIPE			(1<<5)	/* stripe */
#define AC_WCAP_PROC_WID		(1<<6)	/* Proc Widget */
#define AC_WCAP_UNSOL_CAP		(1<<7)	/* Unsol capable */
#define AC_WCAP_CONN_LIST		(1<<8)	/* connection list */
#define AC_WCAP_DIGITAL			(1<<9)	/* digital I/O */
#define AC_WCAP_POWER			(1<<10)	/* power control */
#define AC_WCAP_LR_SWAP			(1<<11)	/* L/R swap */
#define AC_WCAP_DELAY			(0xf<<16)
#define AC_WCAP_DELAY_SHIFT		16
#define AC_WCAP_TYPE			(0xf<<20)
#define AC_WCAP_TYPE_SHIFT		20

/* supported PCM rates and bits */
#define AC_SUPPCM_RATES			(0xfff << 0)
#define AC_SUPPCM_BITS_8		(1<<16)
#define AC_SUPPCM_BITS_16		(1<<17)
#define AC_SUPPCM_BITS_20		(1<<18)
#define AC_SUPPCM_BITS_24		(1<<19)
#define AC_SUPPCM_BITS_32		(1<<20)

/* supported PCM stream format */
#define AC_SUPFMT_PCM			(1<<0)
#define AC_SUPFMT_FLOAT32		(1<<1)
#define AC_SUPFMT_AC3			(1<<2)

/* GP I/O count */
#define AC_GPIO_IO_COUNT		(0xff<<0)
#define AC_GPIO_O_COUNT			(0xff<<8)
#define AC_GPIO_O_COUNT_SHIFT		8
#define AC_GPIO_I_COUNT			(0xff<<16)
#define AC_GPIO_I_COUNT_SHIFT		16
#define AC_GPIO_UNSOLICITED		(1<<30)
#define AC_GPIO_WAKE			(1<<31)

/* Converter stream, channel */
#define AC_CONV_CHANNEL			(0xf<<0)
#define AC_CONV_STREAM			(0xf<<4)
#define AC_CONV_STREAM_SHIFT		4

/* Input converter SDI select */
#define AC_SDI_SELECT			(0xf<<0)

/* Unsolicited response */
#define AC_UNSOL_TAG			(0x3f<<0)
#define AC_UNSOL_ENABLED		(1<<7)

/* Pin widget capabilies */
#define AC_PINCAP_IMP_SENSE		(1<<0)	/* impedance sense capable */
#define AC_PINCAP_TRIG_REQ		(1<<1)	/* trigger required */
#define AC_PINCAP_PRES_DETECT		(1<<2)	/* presence detect capable */
#define AC_PINCAP_HP_DRV		(1<<3)	/* headphone drive capable */
#define AC_PINCAP_OUT			(1<<4)	/* output capable */
#define AC_PINCAP_IN			(1<<5)	/* input capable */
#define AC_PINCAP_BALANCE		(1<<6)	/* balanced I/O capable */
/* Note: This LR_SWAP pincap is defined in the Realtek ALC883 specification,
 *       but is marked reserved in the Intel HDA specification.
 */
#define AC_PINCAP_LR_SWAP		(1<<7)	/* L/R swap */
#define AC_PINCAP_VREF			(0x37<<8)
#define AC_PINCAP_VREF_SHIFT		8
#define AC_PINCAP_EAPD			(1<<16)	/* EAPD capable */
/* Vref status (used in pin cap) */
#define AC_PINCAP_VREF_HIZ		(1<<0)	/* Hi-Z */
#define AC_PINCAP_VREF_50		(1<<1)	/* 50% */
#define AC_PINCAP_VREF_GRD		(1<<2)	/* ground */
#define AC_PINCAP_VREF_80		(1<<4)	/* 80% */
#define AC_PINCAP_VREF_100		(1<<5)	/* 100% */

/* Amplifier capabilities */
#define AC_AMPCAP_OFFSET		(0x7f<<0)  /* 0dB offset */
#define AC_AMPCAP_OFFSET_SHIFT		0
#define AC_AMPCAP_NUM_STEPS		(0x7f<<8)  /* number of steps */
#define AC_AMPCAP_NUM_STEPS_SHIFT	8
#define AC_AMPCAP_STEP_SIZE		(0x7f<<16) /* step size 0-32dB
						    * in 0.25dB
						    */
#define AC_AMPCAP_STEP_SIZE_SHIFT	16
#define AC_AMPCAP_MUTE			(1<<31)    /* mute capable */
#define AC_AMPCAP_MUTE_SHIFT		31

/* Connection list */
#define AC_CLIST_LENGTH			(0x7f<<0)
#define AC_CLIST_LONG			(1<<7)

/* Supported power status */
#define AC_PWRST_D0SUP			(1<<0)
#define AC_PWRST_D1SUP			(1<<1)
#define AC_PWRST_D2SUP			(1<<2)
#define AC_PWRST_D3SUP			(1<<3)

/* Power state values */
#define AC_PWRST_SETTING		(0xf<<0)
#define AC_PWRST_ACTUAL			(0xf<<4)
#define AC_PWRST_ACTUAL_SHIFT		4
#define AC_PWRST_D0			0x00
#define AC_PWRST_D1			0x01
#define AC_PWRST_D2			0x02
#define AC_PWRST_D3			0x03

/* Processing capabilies */
#define AC_PCAP_BENIGN			(1<<0)
#define AC_PCAP_NUM_COEF		(0xff<<8)
#define AC_PCAP_NUM_COEF_SHIFT		8

/* Volume knobs capabilities */
#define AC_KNBCAP_NUM_STEPS		(0x7f<<0)
#define AC_KNBCAP_DELTA			(1<<7)

/*
 * Control Parameters
 */

/* Amp gain/mute */
#define AC_AMP_MUTE			(1<<7)
#define AC_AMP_GAIN			(0x7f)
#define AC_AMP_GET_INDEX		(0xf<<0)

#define AC_AMP_GET_LEFT			(1<<13)
#define AC_AMP_GET_RIGHT		(0<<13)
#define AC_AMP_GET_OUTPUT		(1<<15)
#define AC_AMP_GET_INPUT		(0<<15)

#define AC_AMP_SET_INDEX		(0xf<<8)
#define AC_AMP_SET_INDEX_SHIFT		8
#define AC_AMP_SET_RIGHT		(1<<12)
#define AC_AMP_SET_LEFT			(1<<13)
#define AC_AMP_SET_INPUT		(1<<14)
#define AC_AMP_SET_OUTPUT		(1<<15)

/* from solaris */
#define AC_GAIN_MASK			0x007f
#define AC_GAIN_MAX				0x7f
#define AC_GAIN_BITS			0x7
#define AC_GAIN_DEFAULT			0x0f

/* DIGITAL1 bits */
#define AC_DIG1_ENABLE			(1<<0)
#define AC_DIG1_V			(1<<1)
#define AC_DIG1_VCFG			(1<<2)
#define AC_DIG1_EMPHASIS		(1<<3)
#define AC_DIG1_COPYRIGHT		(1<<4)
#define AC_DIG1_NONAUDIO		(1<<5)
#define AC_DIG1_PROFESSIONAL		(1<<6)
#define AC_DIG1_LEVEL			(1<<7)

/* DIGITAL2 bits */
#define AC_DIG2_CC			(0x7f<<0)

/* Pin widget control - 8bit */
#define AC_PINCTL_VREFEN		(0x7<<0)
#define AC_PINCTL_VREF_HIZ		0	/* Hi-Z */
#define AC_PINCTL_VREF_50		1	/* 50% */
#define AC_PINCTL_VREF_GRD		2	/* ground */
#define AC_PINCTL_VREF_80		4	/* 80% */
#define AC_PINCTL_VREF_100		5	/* 100% */
#define AC_PINCTL_IN_EN			(1<<5)
#define AC_PINCTL_OUT_EN		(1<<6)
#define AC_PINCTL_HP_EN			(1<<7)

/* Unsolicited response - 8bit */
#define AC_USRSP_EN			(1<<7)

/* Pin sense - 32bit */
#define AC_PINSENSE_IMPEDANCE_MASK	(0x7fffffff)
#define AC_PINSENSE_PRESENCE		(1<<31)

/* EAPD/BTL enable - 32bit */
#define AC_EAPDBTL_BALANCED		(1<<0)
#define AC_EAPDBTL_EAPD			(1<<1)
#define AC_EAPDBTL_LR_SWAP		(1<<2)

/* configuration default - 32bit */
#define AC_DEFCFG_SEQUENCE		(0xf<<0)
#define AC_DEFCFG_DEF_ASSOC		(0xf<<4)
#define AC_DEFCFG_ASSOC_SHIFT		4
#define AC_DEFCFG_MISC			(0xf<<8)
#define AC_DEFCFG_MISC_SHIFT		8
#define AC_DEFCFG_MISC_NO_PRESENCE	(1<<0)
#define AC_DEFCFG_COLOR			(0xf<<12)
#define AC_DEFCFG_COLOR_SHIFT		12
#define AC_DEFCFG_CONN_TYPE		(0xf<<16)
#define AC_DEFCFG_CONN_TYPE_SHIFT	16
#define AC_DEFCFG_DEVICE		(0xf<<20)
#define AC_DEFCFG_DEVICE_SHIFT		20
#define AC_DEFCFG_LOCATION		(0x3f<<24)
#define AC_DEFCFG_LOCATION_SHIFT	24
#define AC_DEFCFG_PORT_CONN		(0x3<<30)
#define AC_DEFCFG_PORT_CONN_SHIFT	30

/* device device types (0x0-0xf) */
enum {
	AC_JACK_LINE_OUT,
	AC_JACK_SPEAKER,
	AC_JACK_HP_OUT,
	AC_JACK_CD,
	AC_JACK_SPDIF_OUT,
	AC_JACK_DIG_OTHER_OUT,
	AC_JACK_MODEM_LINE_SIDE,
	AC_JACK_MODEM_HAND_SIDE,
	AC_JACK_LINE_IN,
	AC_JACK_AUX,
	AC_JACK_MIC_IN,
	AC_JACK_TELEPHONY,
	AC_JACK_SPDIF_IN,
	AC_JACK_DIG_OTHER_IN,
	AC_JACK_OTHER = 0xf,
};

/* jack connection types (0x0-0xf) */
enum {
	AC_JACK_CONN_UNKNOWN,
	AC_JACK_CONN_1_8,
	AC_JACK_CONN_1_4,
	AC_JACK_CONN_ATAPI,
	AC_JACK_CONN_RCA,
	AC_JACK_CONN_OPTICAL,
	AC_JACK_CONN_OTHER_DIGITAL,
	AC_JACK_CONN_OTHER_ANALOG,
	AC_JACK_CONN_DIN,
	AC_JACK_CONN_XLR,
	AC_JACK_CONN_RJ11,
	AC_JACK_CONN_COMB,
	AC_JACK_CONN_OTHER = 0xf,
};

/* jack colors (0x0-0xf) */
enum {
	AC_JACK_COLOR_UNKNOWN,
	AC_JACK_COLOR_BLACK,
	AC_JACK_COLOR_GREY,
	AC_JACK_COLOR_BLUE,
	AC_JACK_COLOR_GREEN,
	AC_JACK_COLOR_RED,
	AC_JACK_COLOR_ORANGE,
	AC_JACK_COLOR_YELLOW,
	AC_JACK_COLOR_PURPLE,
	AC_JACK_COLOR_PINK,
	AC_JACK_COLOR_WHITE = 0xe,
	AC_JACK_COLOR_OTHER,
};

/* Jack location (0x0-0x3f) */
/* common case */
enum {
	AC_JACK_LOC_NONE,
	AC_JACK_LOC_REAR,
	AC_JACK_LOC_FRONT,
	AC_JACK_LOC_LEFT,
	AC_JACK_LOC_RIGHT,
	AC_JACK_LOC_TOP,
	AC_JACK_LOC_BOTTOM,
};
/* bits 4-5 */
enum {
	AC_JACK_LOC_EXTERNAL = 0x00,
	AC_JACK_LOC_INTERNAL = 0x10,
	AC_JACK_LOC_SEPARATE = 0x20,
	AC_JACK_LOC_OTHER    = 0x30,
};
enum {
	/* external on primary chasis */
	AC_JACK_LOC_REAR_PANEL = 0x07,
	AC_JACK_LOC_DRIVE_BAY,
	/* internal */
	AC_JACK_LOC_RISER = 0x17,
	AC_JACK_LOC_HDMI,
	AC_JACK_LOC_ATAPI,
	/* others */
	AC_JACK_LOC_MOBILE_IN = 0x37,
	AC_JACK_LOC_MOBILE_OUT,
};

/* Port connectivity (0-3) */
enum {
	AC_JACK_PORT_COMPLEX,
	AC_JACK_PORT_NONE,
	AC_JACK_PORT_FIXED,
	AC_JACK_PORT_BOTH,
};

/* max. connections to a widget */
#define HDA_MAX_CONNECTIONS	32

/* max. codec address */
#define HDA_MAX_CODEC_ADDRESS	0x0f

/*
 * currently, only the format of 48K sample rate, 16-bit
 * 2-channel is supported.
 */
#define	HDA_FMT_PCMOUT	0x0011
#define	HDA_FMT_PCMIN	0x0011


class IOFilterInterruptEventSource;
class IOInterruptEventSource;

class HDACodec : public IOAudioEngine
{
    OSDeclareDefaultStructors(HDACodec)
    
	friend class HDAController;
	
    HDAController					*audioDevice;
	unsigned int					codecAddress;
    
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    
    IOFilterInterruptEventSource	*interruptEventSource;
    
	IOAudioStream					*outputAudioStream;
	
	UInt32							afg, mfg;
	UInt32							vendorId;
	UInt32							subsystemId;
	UInt32							revisionId;
	UInt32							*wcaps;
	int								numNodes;
	UInt32							startNid;
	int								numberOfPlaybackBDLEPassed;
	
	int nitrs;
	
public:

    virtual bool init(HDAController *device, unsigned int addr);
    virtual void free();
    
    virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize);

    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);

    virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    
    static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
    static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
    virtual void filterInterrupt(int index);
	

	virtual void setupStream(UInt32 nid, UInt32 streamTag, int channelId, int format);
	virtual bool initDACNode(UInt32 nid);
	virtual bool initMixerNode(UInt32 nid, int indexNum);
	virtual bool unmuteOutputNode(UInt32 nid);
	virtual bool enablePinOutNode(UInt32 nid);
	virtual bool disablePinOutNode(UInt32 nid);
	virtual bool setPCMFormat(int dir, int format);

	virtual bool startPlayback(int stream);
	virtual bool resetStream(int stream);
	virtual bool fillPlaybackBuffer();
	virtual bool stopPlayback(int stream);
	virtual bool setFormat(int stream, int dir, int sample_rate, int channels, int precision, int encoding);
	virtual void getMaxGain(unsigned int &pgain, unsigned int &rgain);
	virtual bool setGain(int dir, int gain, int channel);
	virtual bool muteOutputs(bool mute);
	virtual unsigned int getCurrentSampleFrame(int stream);

	
	/* lowlevel interface */
	virtual void powerUp();
	virtual void powerDown();
	virtual UInt32 codecRead(UInt16 nid, int direct, unsigned int verb, unsigned int parm);
	virtual bool codecWrite(UInt16 nid, int direct, unsigned int verb, unsigned int parm);
	virtual UInt32 paramRead(UInt16 nid, unsigned int param);
	virtual void setupFunctionGroupNodes();
	virtual int getSubNodes(UInt32 nid, UInt32 &startId);
	virtual bool readWidgetCaps(UInt32 fgNode);
};


#endif /* __HDACODEC_H__ */
