
/*
 * SynthOutput.inc
 *
 * Created: 2024/3/9 21:47:57
 *  Author: yuan
 */ 

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

