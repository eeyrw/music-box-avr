#include "WaveTable.h"
#include "SynthCore.h"
#include "AsmCommon.h"

.section .text 
	SynthAsm: ; Register usage: t0l,t0h:r18,r19 t1l,t1h:r20,r21 zl,zh:r30,r31
	push yl
	push yh
	push loopIndex
	push mixOutl
	push mixOuth
	push mixOute	
	clr loopIndex
	clr mixOutl
	clr mixOuth
	clr mixOute	
	clr zero
	ldi yl,lo8(synthForAsm)
	ldi yh,hi8(synthForAsm)
	loopSynth:
		cpi loopIndex,POLY_NUM
		breq loopSynth_end
		ldd t1h,Y+pEnvelopeLevel ; Load evnvlopelevel to t1h for later use	
		cp t1h,zero
		breq loopSynthContinue	
		ldd zl,Y+pWavetablePos_int_l ; Get a sample by pWavetablePos_int and save to t0l
		ldd zh,Y+pWavetablePos_int_h


		ldd t0l,Y+pIncrement_frac
		ldd r0,Y+pWavetablePos_frac ; Load pWavetablePos_frac to r0 for later use	
		add t0l,r0
		std Y+pWavetablePos_frac,t0l

		ldd t0l,Y+pIncrement_int
		adc t0l,zl

		mov t0h,zh
		adc t0h,zero

		cpi t0l,lo8(WAVETABLE_LEN)
		ldi t1l,hi8(WAVETABLE_LEN)
		cpc t0h,t1l
		brlo branch0_end
		subi t0l,lo8(WAVETABLE_LOOP_LEN)
		ldi t1l,hi8(WAVETABLE_LOOP_LEN)
		sbc t0h,t1l
		branch0_end:
		std Y+pWavetablePos_int_l,t0l
		std Y+pWavetablePos_int_h,t0h



		ldi t2l,hh8(WaveTable)
		ldi t0h,hi8(WaveTable)
		ldi t0l,lo8(WaveTable)
		
		
		add zl,t0l
		adc zh,t0h
		adc t2l,zero
		out RAMPZ,t2l
	#ifdef USE_LINEAR_INTEROP
		elpm t0l,Z+
		elpm t0h,Z
		sub t0h,t0l ;TO BE FIXED: Overflow will happen if t0h=127,t0l=-128
		mov t2l,r0
		mulsu t0h,t2l ;r1:r0=sampleDiff*pWavetablePos_frac

		;https://sestevenson.wordpress.com/2009/08/19/rounding-in-fixed-point-number-conversions/
		;r1 = Round(r1.r0)
		ldi t1l,0x80
		ldi t1h,0x00
		add r0,t1l
		adc r1,t1h

		add t0l,r1
	#else
		elpm t0l,Z
	#endif

	#ifdef RUN_TEST
		std Y+pSampleVal,t0l
	#endif
		mulsu t0l,t1h ;r1:r0=sample*evnvlopelevel
	#ifdef RUN_TEST	
		std Y+pVal,r1
		std Y+(pVal+1),r0
	#endif	
		clr t0l ;(r1:r0)/255
		sbrc r1,7 ;skip if bit 7(sign bit of signed 16) of r1 is 0
		com t0l ;~r0=0xFF	
		add mixOutl,r0 ; mixOut+=(r1:r0)>>8
		adc mixOuth,r1			
		adc mixOute,t0l
		clr r1 ; Always keep r1(zero)=0			


	loopSynthContinue:						
	inc loopIndex
	ldi t0l,SoundUnitSize
	add yl,t0l
	adc yh,zero
	rjmp loopSynth

	loopSynth_end:

	; mixOutHL = Round(mixOutEH.L)
	ldi t0l,0x80
	add mixOutl,t0l
	adc mixOuth,zero
	adc mixOute,zero

	mov mixOutl,mixOuth
	mov mixOuth,mixOute


	#ifdef RUN_TEST
	sts synthForAsm+pMixOut,mixOutl
	sts synthForAsm+pMixOut+1,mixOuth
	#endif

	ldi	t0l, lo8(511)			;Clip it between -512 to 511
	ldi t0h, hi8(511)
	cp mixOutl,t0l
	cpc mixOuth,t0h
	brlt	higherBoundSatisfied			;
	movw	mixOutl, t0l			;
	higherBoundSatisfied:
	ldi	t0l, lo8(-512)			;
	ldi t0h, hi8(-512)
	cp mixOutl,t0l
	cpc mixOuth,t0h	;
	brge	lowerBoundSatisfied			;
	movw	mixOutl, t0l			;/
	lowerBoundSatisfied:

	ldi	t0l, lo8(512)
	ldi t0h, hi8(512)

	add mixOutl,t0l
	adc mixOuth,t0h


	sts TCA0_SPLIT_HCMP0,mixOuth

	; mixOut = mixOut<<6;
	clr	t0l
	lsr	mixOuth
	ror	mixOutl
	ror	t0l
	lsr	mixOuth
	ror	mixOutl
	ror	t0l
	mov	mixOuth, mixOutl
	mov	mixOutl, t0l

	
	sts DAC0_DATAL,mixOutl
	sts DAC0_DATAH,mixOuth

	pop mixOute	
	pop mixOuth
	pop mixOutl
	pop loopIndex
	pop yh
	pop yl