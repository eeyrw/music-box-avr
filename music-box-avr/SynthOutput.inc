
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
	asr mixOuth ; Set it to PWM modulator �� ��16λ��T2������λ����һλ������2�����λ�Ƶ�Carrier 
	ror mixOutl			
	ror	mixOuth				; ��T2H�ĸ�λ���ƣ�Carrier����T2H�ĵ���λ
	mov	t0l, mixOutl			; ����T2L��EL
	subi t0l, 0x80	; EL=EL-0x80
	mov	t0h, t0l			; 
	com	t0h				; EHȡ��
	sbrc mixOuth, 7		; ���T2H�ĵ���λ��0����������һ��
	inc	t0l				; EL++
	sts OCR1AL,t0l
	sts OCR1BL,t0h