////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
.globl MemReadTestX8
.type MemReadTestX8, %function
.section .tcmtext
MemReadTestX8:

	//r2 = index (in words not bytes)
	mov		r2, #0
	push	{r4}

//16x unroll
readloop_x8:
	ldrb	r3, [r0, #0]
	ldrb	r4, [r0, #1]
	ldrb	r3, [r0, #2]
	ldrb	r4, [r0, #3]
	ldrb	r3, [r0, #4]
	ldrb	r4, [r0, #5]
	ldrb	r3, [r0, #6]
	ldrb	r4, [r0, #7]
	ldrb	r3, [r0, #8]
	ldrb	r4, [r0, #9]
	ldrb	r3, [r0, #10]
	ldrb	r4, [r0, #11]
	ldrb	r3, [r0, #12]
	ldrb	r4, [r0, #13]
	ldrb	r3, [r0, #14]
	ldrb	r4, [r0, #15]

	add		r2, r2, #16
	add		r0, r0, #16

	cmp		r2, r1
	bne		readloop_x8

	pop		{r4}
	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

.globl MemWriteTestX8
.type MemWriteTestX8, %function
.section .tcmtext
MemWriteTestX8:

	//r2 = index (in words not bytes)
	mov		r2, #0

//16x unroll
writeloop_x8:
	strb	r2, [r0, #0]
	strb	r2, [r0, #1]
	strb	r2, [r0, #2]
	strb	r2, [r0, #3]
	strb	r2, [r0, #4]
	strb	r2, [r0, #5]
	strb	r2, [r0, #6]
	strb	r2, [r0, #7]
	strb	r2, [r0, #8]
	strb	r2, [r0, #9]
	strb	r2, [r0, #10]
	strb	r2, [r0, #11]
	strb	r2, [r0, #12]
	strb	r2, [r0, #13]
	strb	r2, [r0, #14]
	strb	r2, [r0, #15]

	add		r2, r2, #16
	add		r0, r0, #16

	cmp		r2, r1
	bne		writeloop_x8

	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
.globl MemReadTestX16
.type MemReadTestX16, %function
.section .tcmtext
MemReadTestX16:

	//r2 = index (in words not bytes)
	mov		r2, #0
	push	{ r4 }

//16x unroll
readloop_x16:
	ldrh	r3, [r0, #0]
	ldrh	r4, [r0, #2]
	ldrh	r3, [r0, #4]
	ldrh	r4, [r0, #6]
	ldrh	r3, [r0, #8]
	ldrh	r4, [r0, #10]
	ldrh	r3, [r0, #12]
	ldrh	r4, [r0, #14]
	ldrh	r3, [r0, #16]
	ldrh	r4, [r0, #18]
	ldrh	r3, [r0, #20]
	ldrh	r4, [r0, #22]
	ldrh	r3, [r0, #24]
	ldrh	r4, [r0, #26]
	ldrh	r3, [r0, #28]
	ldrh	r4, [r0, #30]

	add		r2, r2, #16
	add		r0, r0, #32

	cmp		r2, r1
	bne		readloop_x16

	pop		{r4}
	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

.globl MemWriteTestX16
.type MemWriteTestX16, %function
.section .tcmtext
MemWriteTestX16:

	//r2 = index (in words not bytes)
	mov		r2, #0

//16x unroll
writeloop_x16:
	strh	r2, [r0, #0]
	strh	r2, [r0, #2]
	strh	r2, [r0, #4]
	strh	r2, [r0, #6]
	strh	r2, [r0, #8]
	strh	r2, [r0, #10]
	strh	r2, [r0, #12]
	strh	r2, [r0, #14]
	strh	r2, [r0, #16]
	strh	r2, [r0, #18]
	strh	r2, [r0, #20]
	strh	r2, [r0, #22]
	strh	r2, [r0, #24]
	strh	r2, [r0, #26]
	strh	r2, [r0, #28]
	strh	r2, [r0, #30]

	add		r2, r2, #16
	add		r0, r0, #32

	cmp		r2, r1
	bne		writeloop_x16

	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
.globl MemReadTestX32
.type MemReadTestX32, %function
.section .tcmtext
MemReadTestX32:

	//r2 = index (in words not bytes)
	mov		r2, #0
	push	{r4}

//16x unroll
readloop_x32:
	ldr		r3, [r0, #0]
	ldr		r4, [r0, #4]
	ldr		r3, [r0, #8]
	ldr		r4, [r0, #12]
	ldr		r3, [r0, #16]
	ldr		r4, [r0, #20]
	ldr		r3, [r0, #24]
	ldr		r4, [r0, #28]
	ldr		r3, [r0, #32]
	ldr		r4, [r0, #36]
	ldr		r3, [r0, #40]
	ldr		r4, [r0, #44]
	ldr		r3, [r0, #48]
	ldr		r4, [r0, #52]
	ldr		r3, [r0, #56]
	ldr		r4, [r0, #60]

	add		r2, r2, #16
	add		r0, r0, #64

	cmp		r2, r1
	bne		readloop_x32

	pop		{ r4 }
	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//r0 = ptr
//r1 = len
.globl MemWriteTestX32
.type MemWriteTestX32, %function
.section .tcmtext
MemWriteTestX32:

	//r2 = index (in words not bytes)
	mov		r2, #0

//16x unroll
writeloop_x32:
	str		r2, [r0, #0]
	str		r2, [r0, #4]
	str		r2, [r0, #8]
	str		r2, [r0, #12]
	str		r2, [r0, #16]
	str		r2, [r0, #20]
	str		r2, [r0, #24]
	str		r2, [r0, #28]
	str		r2, [r0, #32]
	str		r2, [r0, #36]
	str		r2, [r0, #40]
	str		r2, [r0, #44]
	str		r2, [r0, #48]
	str		r2, [r0, #52]
	str		r2, [r0, #56]
	str		r2, [r0, #60]

	add		r2, r2, #16
	add		r0, r0, #64

	cmp		r2, r1
	bne		writeloop_x32

	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
.globl MemReadTestX64
.type MemReadTestX64, %function
.section .tcmtext
MemReadTestX64:

	//r2 = index (in words not bytes)
	mov		r2, #0
	push	{ r4 }

//16x unroll
readloop_x64:
	ldrd	r3, r4, [r0, #0]
	ldrd	r3, r4, [r0, #8]
	ldrd	r3, r4, [r0, #16]
	ldrd	r3, r4, [r0, #24]
	ldrd	r3, r4, [r0, #32]
	ldrd	r3, r4, [r0, #40]
	ldrd	r3, r4, [r0, #48]
	ldrd	r3, r4, [r0, #56]
	ldrd	r3, r4, [r0, #64]
	ldrd	r3, r4, [r0, #72]
	ldrd	r3, r4, [r0, #80]
	ldrd	r3, r4, [r0, #88]
	ldrd	r3, r4, [r0, #96]
	ldrd	r3, r4, [r0, #104]
	ldrd	r3, r4, [r0, #112]
	ldrd	r3, r4, [r0, #120]

	add		r2, r2, #16
	add		r0, r0, #128

	cmp		r2, r1
	bne		readloop_x64

	pop		{ r4 }
	bx		lr

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

.globl MemWriteTestX64
.type MemWriteTestX64, %function
.section .tcmtext
MemWriteTestX64:

	//r2 = index (in words not bytes)
	mov		r2, #0

//16x unroll
writeloop_x64:
	strd	r2, r2, [r0, #0]
	strd	r2, r2, [r0, #8]
	strd	r2, r2, [r0, #16]
	strd	r2, r2, [r0, #24]
	strd	r2, r2, [r0, #32]
	strd	r2, r2, [r0, #40]
	strd	r2, r2, [r0, #48]
	strd	r2, r2, [r0, #56]

	strd	r2, r2, [r0, #64]
	strd	r2, r2, [r0, #72]
	strd	r2, r2, [r0, #80]
	strd	r2, r2, [r0, #88]
	strd	r2, r2, [r0, #96]
	strd	r2, r2, [r0, #104]
	strd	r2, r2, [r0, #112]
	strd	r2, r2, [r0, #120]

	add		r2, r2, #16
	add		r0, r0, #128

	cmp		r2, r1
	bne		writeloop_x64

	bx		lr
