/*
 * Intel High Definition Audio Device Driver for OS X
 *
 * Copyright 2008 Sergei Gluschenko, All Rights Reserved.
 *
 * Big thanks to FreeBSD, OpenSolaris, OSS and ALSA Azalia driver developes teams for their gracefull and clean code, which
 * was a great sample and probably start point for this driver.
 */

#include "xmmintrin.h"

#include "HDACodec.h"




	// our compiler does ALL floating point with SSE
	#define GETCSR()    ({ int _result; asm volatile ("stmxcsr %0" : "=m" (*&_result) ); /*return*/ _result; })
	#define SETCSR( a )    { int _temp = a; asm volatile( "ldmxcsr %0" : : "m" (*&_temp ) ); }

	#define DISABLE_DENORMALS int _savemxcsr = GETCSR(); SETCSR(_savemxcsr | 0x8040);
	#define RESTORE_DENORMALS SETCSR(_savemxcsr);
	
	#define ROUNDMODE_NEG_INF int _savemxcsr = GETCSR(); SETCSR((_savemxcsr & ~0x6000) | 0x2000);
	#define RESTORE_ROUNDMODE SETCSR(_savemxcsr);
	#define SET_ROUNDMODE 		ROUNDMODE_NEG_INF


//
// FloatToInt
// N.B. Functions which use this should invoke SET_ROUNDMODE / RESTORE_ROUNDMODE.
static inline SInt32 FloatToInt(double inf, double min32, double max32)
{
	if (inf >= max32) return 0x7FFFFFFF;
	return (SInt32)inf;
}


void Float32ToNativeInt16_X86(const float *src, SInt16 *dst, unsigned int numToConvert )
{
	const float *src0 = src;
	int16_t *dst0 = dst;
	unsigned int count = numToConvert;
	
	if (count >= 8) {
		// vector -- requires 8+ samples
		ROUNDMODE_NEG_INF
		const __m128 vround = (const __m128) { 0.5f, 0.5f, 0.5f, 0.5f };
		const __m128 vmin = (const __m128) { -32768.0f, -32768.0f, -32768.0f, -32768.0f };
		const __m128 vmax = (const __m128) { 32767.0f, 32767.0f, 32767.0f, 32767.0f  };
		const __m128 vscale = (const __m128) { 32768.0f, 32768.0f, 32768.0f, 32768.0f  };
		__m128 vf0, vf1;
		__m128i vi0, vi1, vpack0;
	
#define F32TOLE16 \
		vf0 = _mm_mul_ps(vf0, vscale);			\
		vf1 = _mm_mul_ps(vf1, vscale);			\
		vf0 = _mm_add_ps(vf0, vround);			\
		vf1 = _mm_add_ps(vf1, vround);			\
		vf0 = _mm_max_ps(vf0, vmin);			\
		vf1 = _mm_max_ps(vf1, vmin);			\
		vf0 = _mm_min_ps(vf0, vmax);			\
		vf1 = _mm_min_ps(vf1, vmax);			\
		vi0 = _mm_cvtps_epi32(vf0);			\
		vi1 = _mm_cvtps_epi32(vf1);			\
		vpack0 = _mm_packs_epi32(vi0, vi1);

		int falign = (uintptr_t)src & 0xF;
		int ialign = (uintptr_t)dst & 0xF;
	
		if (falign != 0 || ialign != 0) {
			// do one unaligned conversion
			vf0 = _mm_loadu_ps(src);
			vf1 = _mm_loadu_ps(src+4);
			F32TOLE16
			_mm_storeu_si128((__m128i *)dst, vpack0);
			
			// advance such that the destination ints are aligned
			unsigned int n = (16 - ialign) / 2;
			src += n;
			dst += n;
			count -= n;

			falign = (uintptr_t)src & 0xF;
			if (falign != 0) {
				// unaligned loads, aligned stores
				while (count >= 8) {
					vf0 = _mm_loadu_ps(src);
					vf1 = _mm_loadu_ps(src+4);
					F32TOLE16
					_mm_store_si128((__m128i *)dst, vpack0);
					src += 8;
					dst += 8;
					count -= 8;
				}
				goto VectorCleanup;
			}
		}
	
		// aligned loads, aligned stores
		while (count >= 8) {
			vf0 = _mm_load_ps(src);
			vf1 = _mm_load_ps(src+4);
			F32TOLE16
			_mm_store_si128((__m128i *)dst, vpack0);
			
			src += 8;
			dst += 8;
			count -= 8;
		}
VectorCleanup:
		if (count > 0) {
			// unaligned cleanup -- just do one unaligned vector at the end
			src = src0 + numToConvert - 8;
			dst = dst0 + numToConvert - 8;
			vf0 = _mm_loadu_ps(src);
			vf1 = _mm_loadu_ps(src+4);
			F32TOLE16
			_mm_storeu_si128((__m128i *)dst, vpack0);
		}
		RESTORE_ROUNDMODE
		return;
	}
	
	// scalar for small numbers of samples
	if (count > 0) {
		double scale = 2147483648.0, round = 32768.0, max32 = 2147483648.0 - 1.0 - 32768.0, min32 = 0.;
		ROUNDMODE_NEG_INF
		
		while (count-- > 0) {
			double f0 = *src++;
			f0 = f0 * scale + round;
			SInt32 i0 = FloatToInt(f0, min32, max32);
			i0 >>= 16;
			*dst++ = i0;
		}
		RESTORE_ROUNDMODE
	}
}
//#endif

// The function clipOutputSamples() is called to clip and convert samples from the float mix buffer into the actual
// hardware sample buffer.  The samples to be clipped, are guaranteed not to wrap from the end of the buffer to the
// beginning.
// This implementation is very inefficient, but illustrates the clip and conversion process that must take place.
// Each floating-point sample must be clipped to a range of -1.0 to 1.0 and then converted to the hardware buffer
// format

// The parameters are as follows:
//		mixBuf - a pointer to the beginning of the float mix buffer - its size is based on the number of sample frames
// 					times the number of channels for the stream
//		sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//					to the IOAudioStream using setSampleBuffer()
//		firstSampleFrame - this is the index of the first sample frame to perform the clipping and conversion on
//		numSampleFrames - the total number of sample frames to clip and convert
//		streamFormat - the current format of the IOAudioStream this function is operating on
//		audioStream - the audio stream this function is operating on
IOReturn HDACodec::clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{

	Float32ToNativeInt16_X86((float*)mixBuf + firstSampleFrame * streamFormat->fNumChannels,
				(SInt16 *)sampleBuf + firstSampleFrame * streamFormat->fNumChannels,
				numSampleFrames * streamFormat->fNumChannels);

#if 0

    UInt32 sampleIndex, maxSampleIndex;
    float *floatMixBuf;
    SInt16 *outputBuf;

    
    // Start by casting the void * mix and sample buffers to the appropriate types - float * for the mix buffer
    // and SInt16 * for the sample buffer (because our sample hardware uses signed 16-bit samples)
    floatMixBuf = (float *)mixBuf;
    outputBuf = (SInt16 *)sampleBuf;
    
    // We calculate the maximum sample index we are going to clip and convert
    // This is an index into the entire sample and mix buffers
    maxSampleIndex = (firstSampleFrame + numSampleFrames) * streamFormat->fNumChannels;
    
    // Loop through the mix/sample buffers one sample at a time and perform the clip and conversion operations
    for (sampleIndex = (firstSampleFrame * streamFormat->fNumChannels); sampleIndex < maxSampleIndex; sampleIndex++) {
        float inSample;
        
        // Fetch the floating point mix sample
        inSample = floatMixBuf[sampleIndex];
        
        // Clip that sample to a range of -1.0 to 1.0
        // A softer clipping operation could be done here
        if (inSample > 1.0) {
            inSample = 1.0;
        } else if (inSample < -1.0) {
            inSample = -1.0;
        }
        
        // Scale the -1.0 to 1.0 range to the appropriate scale for signed 16-bit samples and then
        // convert to SInt16 and store in the hardware sample buffer
        if (inSample >= 0) {
            outputBuf[sampleIndex] = (SInt16) (inSample * 32767.0);
        } else {
            outputBuf[sampleIndex] = (SInt16) (inSample * 32768.0);
        }
    }

#endif
    
    return kIOReturnSuccess;
}

// The function convertInputSamples() is responsible for converting from the hardware format 
// in the input sample buffer to float samples in the destination buffer and scale the samples 
// to a range of -1.0 to 1.0.  This function is guaranteed not to have the samples wrapped
// from the end of the buffer to the beginning.
// This function only needs to be implemented if the device has any input IOAudioStreams

// This implementation is very inefficient, but illustrates the conversion and scaling that needs to take place.

// The parameters are as follows:
//		sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//					to the IOAudioStream using setSampleBuffer()
//		destBuf - a pointer to the float destination buffer - this is the buffer that the CoreAudio.framework uses
//					its size is numSampleFrames * numChannels * sizeof(float)
//		firstSampleFrame - this is the index of the first sample frame to the input conversion on
//		numSampleFrames - the total number of sample frames to convert and scale
//		streamFormat - the current format of the IOAudioStream this function is operating on
//		audioStream - the audio stream this function is operating on
IOReturn HDACodec::convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
    UInt32 numSamplesLeft;
    float *floatDestBuf;
    SInt16 *inputBuf;
    
//	static int counter = 0;
//	
//	++counter;
//	if (counter == 200) {
//		IOLog("HDACodec::convertInputSamples, firstSampleFrame=%u, fNumChannels=%lu\n", (unsigned int)firstSampleFrame, streamFormat->fNumChannels);
//		counter = 0;
//	}

    // Start by casting the destination buffer to a float *
    floatDestBuf = (float *)destBuf;
    // Determine the starting point for our input conversion 
    inputBuf = &(((SInt16 *)sampleBuf)[firstSampleFrame * streamFormat->fNumChannels]);
    
    // Calculate the number of actual samples to convert
    numSamplesLeft = numSampleFrames * streamFormat->fNumChannels;
    
    // Loop through each sample and scale and convert them
    while (numSamplesLeft > 0) {
        SInt16 inputSample;
        
        // Fetch the SInt16 input sample
        inputSample = *inputBuf;
    
        // Scale that sample to a range of -1.0 to 1.0, convert to float and store in the destination buffer
        // at the proper location
        if (inputSample >= 0) {
            *floatDestBuf = inputSample / 32767.0;
        } else {
            *floatDestBuf = inputSample / 32768.0;
        }
        
        // Move on to the next sample
        ++inputBuf;
        ++floatDestBuf;
        --numSamplesLeft;
    }
    
    return kIOReturnSuccess;
}
