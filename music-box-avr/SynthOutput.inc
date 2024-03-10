
/*
 * SynthOutput.inc
 *
 * Created: 2024/3/9 21:47:57
 *  Author: yuan
 */ 

	ldi	t0l, lo8(253)			;Clip it between -255 to 253
	ldi t0h, hi8(253)
	cp mixOutl,t0l
	cpc mixOuth,t0h
	brlt	higherBoundSatisfied			;
	movw	mixOutl, t0l			;
	higherBoundSatisfied:
	ldi	t0l, lo8(-255)			;
	ldi t0h, hi8(-255)
	cp mixOutl,t0l
	cpc mixOuth,t0h	;
	brge	lowerBoundSatisfied			;
	movw	mixOutl, t0l			;/
	lowerBoundSatisfied:
	asr mixOuth ; Set it to PWM modulator ： 把16位的T2带符号位右移一位，除以2，最低位移到Carrier 
	ror mixOutl			
	ror	mixOuth				; 把T2H的高位右移，Carrier进到T2H的第七位
	mov	t0l, mixOutl			; 复制T2L到EL
	subi t0l, 0x80	; EL=EL-0x80
	mov	t0h, t0l			; 
	com	t0h				; EH取反
	sbrc mixOuth, 7		; 如果T2H的第七位是0就跳过下面一行
	inc	t0l				; EL++
	sts OCR1AL,t0l
	sts OCR1BL,t0h
