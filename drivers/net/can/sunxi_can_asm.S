	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1	@ Tag_ABI_FP_denormal
	.eabi_attribute 21, 1	@ Tag_ABI_FP_exceptions
	.eabi_attribute 23, 3	@ Tag_ABI_FP_number_model
	.eabi_attribute 24, 1	@ Tag_ABI_align8_needed
	.eabi_attribute 25, 1	@ Tag_ABI_align8_preserved
	.eabi_attribute 26, 2	@ Tag_ABI_enum_size
	.eabi_attribute 30, 4	@ Tag_ABI_optimization_goals
	.eabi_attribute 34, 1	@ Tag_CPU_unaligned_access
	.eabi_attribute 18, 2	@ Tag_ABI_PCS_wchar_t
	.file	"sunxi_can_asm.c"

	.syntax divided
	.syntax unified
	.arm
	.syntax unified
	.text
	.align	2
	.global	can_asm_write_cmdreg
	.syntax unified
	.arm
	.type	can_asm_write_cmdreg, %function
can_asm_write_cmdreg:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}	@
	.save {r4, r5, lr}
	mov	r3, #13	@ tmp134,
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r5, r0	@ mod_reg_val, mod_reg_val
	mov	r4, r1	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp, #8]	@ tmp134, mod_reg_val
	mov	r3, #15	@ tmp135,
	str	r3, [sp, #12]	@ tmp135, mod_reg_addr
	ldr	r2, [sp, #8]	@ D.65586, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65586, mod_reg_addr
	cmp	r2, r3	@ D.65586, D.65586
	ldrlt	r3, [sp, #8]	@ D.65586, mod_reg_val
	ldrlt	r2, [sp, #12]	@ D.65586, mod_reg_addr
	addlt	r3, r3, r3, lsl #1	@ D.65586, D.65586, D.65586,
	addlt	r3, r3, r2	@ D.65586, D.65586, D.65586
	strlt	r3, [sp, #12]	@ D.65586, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65586, mod_reg_val
	and	r3, r3, #243	@ D.65586, D.65586,
	str	r3, [sp, #12]	@ D.65586, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65586, mod_reg_val
	.syntax divided
@ 529 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r5, [r4, #4]	@ mod_reg_val, MEM[(volatile u32 *)_6]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #3	@ tmp139,
	str	r3, [sp]	@ tmp139, mod_reg_val
	str	r3, [sp, #4]	@ tmp139, mod_reg_addr
	ldr	r3, [sp]	@ D.65586, mod_reg_val
	add	r3, r3, #1	@ D.65586, D.65586,
	str	r3, [sp]	@ D.65586, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65586, mod_reg_addr
	add	r3, r3, #51	@ D.65586, D.65586,
	str	r3, [sp, #4]	@ D.65586, mod_reg_addr
	ldr	r3, [sp, #4]	@ D.65586, mod_reg_addr
	cmp	r3, #16	@ D.65586,
	ldreq	r3, [sp, #4]	@ D.65586, mod_reg_addr
	ldrne	r3, [sp]	@ D.65586, mod_reg_val
	andne	r3, r3, #243	@ D.65586, D.65586,
	strne	r3, [sp, #4]	@ D.65586, mod_reg_addr
	ldrne	r3, [sp, #4]	@ D.65586, mod_reg_addr
	ldrne	r3, [sp]	@ D.65586, mod_reg_val
.L1:
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
	.fnend
	.size	can_asm_write_cmdreg, .-can_asm_write_cmdreg
	.align	2
	.global	can_asm_fun0
	.syntax unified
	.arm
	.type	can_asm_fun0, %function
can_asm_fun0:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp116,
	add	r2, sp, #24	@ tmp117,,
	stm	sp, {r1, r3}	@,,
	str	r3, [sp, #20]	@ tmp116,
	str	r2, [sp, #16]	@ tmp117,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp116,
	str	r3, [sp, #8]	@ tmp116,
	ldr	r1, .L10	@,
	ldr	r0, .L10+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L11:
	.align	2
.L10:
	.word	-65529
	.word	-1308622748
	.fnend
	.size	can_asm_fun0, .-can_asm_fun0
	.align	2
	.global	can_asm_fun1
	.syntax unified
	.arm
	.type	can_asm_fun1, %function
can_asm_fun1:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp114,
	add	r2, sp, #24	@ tmp115,,
	str	r3, [sp, #20]	@ tmp114,
	str	r2, [sp, #16]	@ tmp115,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp114,
	str	r3, [sp, #8]	@ tmp114,
	str	r3, [sp, #4]	@ tmp114,
	str	r3, [sp]	@ tmp114,
	ldr	r1, .L14	@,
	ldr	r0, .L14+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L15:
	.align	2
.L14:
	.word	-65533
	.word	-1308622748
	.fnend
	.size	can_asm_fun1, .-can_asm_fun1
	.align	2
	.global	can_asm_fun2
	.syntax unified
	.arm
	.type	can_asm_fun2, %function
can_asm_fun2:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp114,
	add	r2, sp, #24	@ tmp115,,
	str	r3, [sp, #20]	@ tmp114,
	str	r2, [sp, #16]	@ tmp115,
	mov	r2, r0	@, num
	str	r3, [sp, #12]	@ tmp114,
	str	r3, [sp, #8]	@ tmp114,
	str	r3, [sp, #4]	@ tmp114,
	str	r3, [sp]	@ tmp114,
	ldr	r1, .L18	@,
	ldr	r0, .L18+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L19:
	.align	2
.L18:
	.word	-65532
	.word	-1308622748
	.fnend
	.size	can_asm_fun2, .-can_asm_fun2
	.align	2
	.global	can_asm_fun3
	.syntax unified
	.arm
	.type	can_asm_fun3, %function
can_asm_fun3:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp113,
	add	r2, sp, #24	@ tmp114,,
	str	r3, [sp, #20]	@ tmp113,
	str	r2, [sp, #16]	@ tmp114,
	mov	r2, r3	@,
	str	r3, [sp, #12]	@ tmp113,
	str	r3, [sp, #8]	@ tmp113,
	str	r3, [sp, #4]	@ tmp113,
	str	r3, [sp]	@ tmp113,
	ldr	r1, .L22	@,
	ldr	r0, .L22+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L23:
	.align	2
.L22:
	.word	-65531
	.word	-1308622748
	.fnend
	.size	can_asm_fun3, .-can_asm_fun3
	.align	2
	.global	can_asm_fun4
	.syntax unified
	.arm
	.type	can_asm_fun4, %function
can_asm_fun4:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp113,
	add	r2, sp, #24	@ tmp114,,
	str	r3, [sp, #20]	@ tmp113,
	str	r2, [sp, #16]	@ tmp114,
	mov	r2, r3	@,
	str	r3, [sp, #12]	@ tmp113,
	str	r3, [sp, #8]	@ tmp113,
	str	r3, [sp, #4]	@ tmp113,
	str	r3, [sp]	@ tmp113,
	ldr	r1, .L26	@,
	ldr	r0, .L26+4	@,
	bl	__arm_smccc_smc	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L27:
	.align	2
.L26:
	.word	-65530
	.word	-1308622748
	.fnend
	.size	can_asm_fun4, .-can_asm_fun4
	.align	2
	.global	can_asm_fun5
	.syntax unified
	.arm
	.type	can_asm_fun5, %function
can_asm_fun5:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!	@,
	.save {lr}
	.pad #44
	sub	sp, sp, #44	@,,
	mov	r3, #0	@ tmp118,
	add	r2, sp, #24	@ tmp119,,
	str	r3, [sp, #20]	@ tmp118,
	str	r3, [sp, #12]	@ tmp118,
	str	r3, [sp, #8]	@ tmp118,
	str	r3, [sp, #4]	@ tmp118,
	str	r3, [sp]	@ tmp118,
	str	r2, [sp, #16]	@ tmp119,
	mov	r2, r0	@, num
	ldr	r1, .L30	@,
	ldr	r0, .L30+4	@,
	bl	__arm_smccc_smc	@
	ldr	r1, [sp, #32]	@, res.a2
	ldr	r0, [sp, #28]	@, res.a1
	bl	ioremap	@
	add	sp, sp, #44	@,,
	@ sp needed	@
	ldr	pc, [sp], #4	@
.L31:
	.align	2
.L30:
	.word	-65528
	.word	-1308622748
	.fnend
	.size	can_asm_fun5, .-can_asm_fun5
	.align	2
	.global	can_asm_start
	.syntax unified
	.arm
	.type	can_asm_start, %function
can_asm_start:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	mov	r3, r1	@ ctrlmode, ctrlmode
	.pad #88
	sub	sp, sp, #88	@,,
	mov	r1, #15	@ tmp168,
	mov	r4, r0	@ mod_reg_addr, mod_reg_addr
	str	r1, [sp, #24]	@ tmp168, mod_reg_val
	mov	r1, #6	@ tmp169,
	str	r1, [sp, #28]	@ tmp169, mod_reg_addr
	ldr	r1, [sp, #24]	@ D.65624, mod_reg_val
	cmp	r1, #8	@ D.65624,
	ldr	r1, [sp, #28]	@ D.65624, mod_reg_addr
	beq	.L34	@,
	ldr	r0, [sp, #24]	@ D.65624, mod_reg_val
	eor	r0, r0, r1	@ D.65624, D.65624, D.65624
	str	r0, [sp, #24]	@ D.65624, mod_reg_val
	ldr	r1, [sp, #24]	@ D.65624, mod_reg_val
	cmp	r1, #16	@ D.65624,
	ldreq	r1, [sp, #28]	@ D.65624, mod_reg_addr
	ldrne	r1, [sp, #24]	@ D.65624, mod_reg_val
.L34:
	mov	r0, #0	@ tmp172,
	add	r1, sp, #72	@ tmp173,,
	str	r0, [sp, #20]	@ tmp172,
	str	r1, [sp, #16]	@ tmp173,
	str	r0, [sp, #12]	@ tmp172,
	str	r0, [sp, #8]	@ tmp172,
	str	r0, [sp, #4]	@ tmp172,
	str	r0, [sp]	@ tmp172,
	ldr	r1, .L48	@,
	ldr	r0, .L48+4	@,
	bl	__arm_smccc_smc	@
	ldr	r3, [sp, #72]	@ res.a0, res.a0
	cmp	r3, #0	@ res.a0,
	bne	.L36	@,
	mov	r3, #13	@ tmp179,
	str	r3, [sp, #40]	@ tmp179, mod_reg_val
	mov	r3, #2	@ tmp180,
	str	r3, [sp, #44]	@ tmp180, mod_reg_addr
	ldr	r2, [sp, #40]	@ D.65624, mod_reg_val
	ldr	r3, [sp, #44]	@ D.65624, mod_reg_addr
	cmp	r2, r3	@ D.65624, D.65624
	ldrlt	r3, [sp, #44]	@ D.65624, mod_reg_addr
	ldrge	r3, [sp, #40]	@ D.65624, mod_reg_val
	mov	r3, #10	@ tmp181,
	str	r3, [sp, #32]	@ tmp181, mod_reg_val
	mov	r3, #6	@ tmp182,
	str	r3, [sp, #36]	@ tmp182, mod_reg_addr
	ldr	r3, [sp, #32]	@ D.65624, mod_reg_val
	and	r3, r3, #9	@ D.65624, D.65624,
	str	r3, [sp, #36]	@ D.65624, mod_reg_addr
	ldr	r2, [sp, #36]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #32]	@ D.65624, mod_reg_val
	eor	r3, r3, r2	@ D.65624, D.65624, D.65624
	str	r3, [sp, #32]	@ D.65624, mod_reg_val
	ldr	r3, [sp, #32]	@ D.65624, mod_reg_val
	and	r3, r3, #9	@ D.65624, D.65624,
	str	r3, [sp, #36]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #36]	@ D.65624, mod_reg_addr
	cmp	r3, #8	@ D.65624,
	ldr	r3, [sp, #32]	@ D.65624, mod_reg_val
	beq	.L32	@,
	add	r2, r3, #1	@ D.65624, D.65624,
	cmp	r3, #7	@ D.65624,
	str	r2, [sp, #32]	@ D.65624, mod_reg_val
	moveq	r3, #51	@ tmp183,
	streq	r3, [sp, #36]	@ tmp183, mod_reg_addr
	ldr	r3, [sp, #32]	@ D.65624, mod_reg_val
	b	.L32	@
.L36:
	mov	r3, #512	@ tmp184,
	str	r3, [sp, #48]	@ tmp184, mod_reg_val
	mov	r3, #50	@ tmp185,
	str	r3, [sp, #52]	@ tmp185, mod_reg_addr
	ldr	r3, [sp, #52]	@ D.65624, mod_reg_addr
	add	r3, r3, #100	@ D.65624, D.65624,
	str	r3, [sp, #52]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #52]	@ D.65624, mod_reg_addr
	sub	r3, r3, #100	@ D.65624, D.65624,
	str	r3, [sp, #52]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #48]	@ D.65624, mod_reg_val
	ldr	r2, [sp, #52]	@ D.65624, mod_reg_addr
	add	r3, r3, r2	@ D.65624, D.65624, D.65624
	cmp	r3, #384	@ D.65624,
	bhi	.L47	@,
	mov	r3, #256	@ tmp188,
	str	r3, [sp, #56]	@ tmp188, mod_reg_val
	mov	r3, #66	@ tmp189,
	str	r3, [sp, #60]	@ tmp189, mod_reg_addr
	ldr	r3, [sp, #56]	@ D.65624, mod_reg_val
	ldr	r2, [sp, #60]	@ D.65624, mod_reg_addr
	add	r3, r3, r2	@ D.65624, D.65624, D.65624
	cmp	r3, #384	@ D.65624,
	bcs	.L43	@,
.L47:
	.syntax divided
@ 598 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	mov	r3, #48	@ tmp191,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r4, #16]	@ tmp191,
@ 0 "" 2
	.arm
	.syntax unified
.L43:
	mov	r3, #4	@ tmp192,
	str	r3, [sp, #64]	@ tmp192, mod_reg_val
	mov	r3, #11	@ tmp193,
	str	r3, [sp, #68]	@ tmp193, mod_reg_addr
	ldr	r3, [sp, #64]	@ D.65624, mod_reg_val
	and	r3, r3, #9	@ D.65624, D.65624,
	str	r3, [sp, #68]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #68]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #64]	@ D.65624, mod_reg_val
	ldr	r2, [sp, #68]	@ D.65624, mod_reg_addr
	add	r3, r3, r2	@ D.65624, D.65624, D.65624
	str	r3, [sp, #68]	@ D.65624, mod_reg_addr
	ldr	r3, [sp, #64]	@ D.65624, mod_reg_val
	add	r2, r3, #1	@ D.65624, D.65624,
	cmp	r3, #7	@ D.65624,
	str	r2, [sp, #64]	@ D.65624, mod_reg_val
	moveq	r3, #51	@ tmp194,
	streq	r3, [sp, #68]	@ tmp194, mod_reg_addr
	movw	r3, #819	@ tmp195,
	ldr	r2, [sp, #64]	@ D.65624, mod_reg_val
	cmp	r2, r3	@ D.65624, tmp195
	ldr	r3, [sp, #68]	@ D.65624, mod_reg_addr
	ldrne	r3, [sp, #64]	@ D.65624, mod_reg_val
	ldrne	r2, [sp, #68]	@ D.65624, mod_reg_addr
	addne	r3, r3, r2	@ D.65624, D.65624, D.65624
	strne	r3, [sp, #68]	@ D.65624, mod_reg_addr
	ldrne	r3, [sp, #64]	@ D.65624, mod_reg_val
.L32:
	add	sp, sp, #88	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L49:
	.align	2
.L48:
	.word	-65535
	.word	-1308622748
	.fnend
	.size	can_asm_start, .-can_asm_start
	.align	2
	.global	can_asm_set_bittiming
	.syntax unified
	.arm
	.type	can_asm_set_bittiming, %function
can_asm_set_bittiming:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}	@
	.save {r4, r5, lr}
	mov	r3, #1	@ tmp135,
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r4, r2	@ cfg, cfg
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp]	@ tmp135, mod_reg_val
	str	r3, [sp, #4]	@ tmp135, mod_reg_addr
	ldr	r2, [sp]	@ D.65636, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65636, mod_reg_addr
	cmp	r2, r3	@ D.65636, D.65636
	ldrlt	r3, [sp]	@ D.65636, mod_reg_val
	ldrlt	r2, [sp, #4]	@ D.65636, mod_reg_addr
	addlt	r3, r3, r3, lsl #1	@ D.65636, D.65636, D.65636,
	addlt	r3, r3, r2	@ D.65636, D.65636, D.65636
	strlt	r3, [sp, #4]	@ D.65636, mod_reg_addr
	tst	r1, #4	@ mod_reg_val,
	ldr	r3, [sp]	@ D.65636, mod_reg_val
	ldrne	r3, [r4]	@ *cfg_6(D), *cfg_6(D)
	orrne	r3, r3, #8388608	@ D.65637, *cfg_6(D),
	strne	r3, [r4]	@ D.65637, *cfg_6(D)
	mov	r3, #16	@ tmp143,
	str	r3, [sp, #8]	@ tmp143, mod_reg_val
	mov	r3, #7	@ tmp144,
	str	r3, [sp, #12]	@ tmp144, mod_reg_addr
	movw	r3, #819	@ tmp145,
	ldr	r2, [sp, #12]	@ D.65636, mod_reg_addr
	mul	r3, r3, r2	@ D.65636, tmp145, D.65636
	str	r3, [sp, #8]	@ D.65636, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65636, mod_reg_val
	add	r3, r3, #1	@ D.65636, D.65636,
	str	r3, [sp, #8]	@ D.65636, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65636, mod_reg_addr
	add	r3, r3, #51	@ D.65636, D.65636,
	str	r3, [sp, #12]	@ D.65636, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65636, mod_reg_addr
	cmp	r3, #16	@ D.65636,
	ldreq	r3, [sp, #12]	@ D.65636, mod_reg_addr
	ldrne	r3, [sp, #8]	@ D.65636, mod_reg_val
	.syntax divided
@ 610 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r4]	@ D.65637, *cfg_6(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #20]	@ D.65637, MEM[(volatile u32 *)_15]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
	.fnend
	.size	can_asm_set_bittiming, .-can_asm_set_bittiming
	.align	2
	.global	can_asm_clean_transfer_err
	.syntax unified
	.arm
	.type	can_asm_clean_transfer_err, %function
can_asm_clean_transfer_err:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #16
	sub	sp, sp, #16	@,,
	mov	r3, #5	@ tmp148,
	str	r3, [sp, #8]	@ tmp148, mod_reg_val
	str	r3, [sp, #12]	@ tmp148, mod_reg_addr
	ldr	ip, [sp, #12]	@ D.65648, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65648, mod_reg_val
	eor	r3, r3, ip	@ D.65648, D.65648, D.65648
	str	r3, [sp, #8]	@ D.65648, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65648, mod_reg_val
	add	r3, r3, #1	@ D.65648, D.65648,
	str	r3, [sp, #8]	@ D.65648, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65648, mod_reg_addr
	add	r3, r3, #51	@ D.65648, D.65648,
	str	r3, [sp, #12]	@ D.65648, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65648, mod_reg_addr
	cmp	r3, #16	@ D.65648,
	beq	.L67	@,
	ldr	ip, [sp, #8]	@ D.65648, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65648, mod_reg_addr
	cmp	ip, r3	@ D.65648, D.65648
	blt	.L67	@,
	ldr	r3, [sp, #8]	@ D.65648, mod_reg_val
	tst	r3, #8	@ D.65648,
	add	ip, r3, #1	@ D.65648, D.65648,
	str	ip, [sp, #8]	@ D.65648, mod_reg_val
	ldreq	ip, [sp, #8]	@ D.65648, mod_reg_val
	ldreq	r3, [sp, #12]	@ D.65648, mod_reg_addr
	muleq	r3, r3, ip	@ D.65648, D.65648, D.65648
	streq	r3, [sp, #12]	@ D.65648, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65648, mod_reg_val
	cmp	r3, #8	@ D.65648,
	ldrne	r3, [sp, #8]	@ D.65648, mod_reg_val
	bne	.L61	@,
.L67:
	ldr	r3, [sp, #12]	@ D.65648, mod_reg_addr
.L61:
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r3, [r0, #28]	@ errors, MEM[(volatile u32 *)_4]
@ 0 "" 2
@ 618 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	mov	r0, #11	@ tmp152,
	str	r0, [sp]	@ tmp152, mod_reg_val
	mov	r0, #5	@ tmp153,
	str	r0, [sp, #4]	@ tmp153, mod_reg_addr
	ldr	r0, [sp]	@ D.65648, mod_reg_val
	and	r0, r0, #243	@ D.65648, D.65648,
	str	r0, [sp, #4]	@ D.65648, mod_reg_addr
	ldr	r0, [sp, #4]	@ D.65648, mod_reg_addr
	ldr	r0, [sp]	@ D.65648, mod_reg_val
	ldr	ip, [sp, #4]	@ D.65648, mod_reg_addr
	add	r0, r0, ip	@ D.65648, D.65648, D.65648
	str	r0, [sp, #4]	@ D.65648, mod_reg_addr
	ldr	r0, [sp]	@ D.65648, mod_reg_val
	uxtb	r0, r3	@ errors, errors
	ubfx	r3, r3, #16, #8	@ D.65650, errors,,
	strh	r0, [r1]	@ movhi	@ errors, *t_err_9(D)
	strh	r3, [r2]	@ movhi	@ D.65650, *r_err_14(D)
	add	sp, sp, #16	@,,
	@ sp needed	@
	bx	lr	@
	.fnend
	.size	can_asm_clean_transfer_err, .-can_asm_clean_transfer_err
	.align	2
	.global	can_asm_start_xmit
	.syntax unified
	.arm
	.type	can_asm_start_xmit, %function
can_asm_start_xmit:
	.fnstart
	@ args = 8, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}	@
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	tst	r2, #1073741824	@ id,
	ldr	r8, [r3]	@ dlc, *flag_6(D)
	mov	r6, r3	@ flag, flag
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	mov	r4, r2	@ id, id
	ldr	r7, [sp, #40]	@ mod_reg_val, mod_reg_val
	orrne	r3, r8, #64	@ D.65670, dlc,
	ldr	r9, [sp, #44]	@ pdata, pdata
	strne	r3, [r6]	@ D.65670, *flag_6(D)
	cmp	r2, #0	@ id,
	bge	.L70	@,
	ldr	r3, [r6]	@ *flag_6(D), *flag_6(D)
	orr	r3, r3, #128	@ D.65670, *flag_6(D),
	str	r3, [r6]	@ D.65670, *flag_6(D)
	mov	r3, #84	@ tmp201,
	str	r3, [r7]	@ tmp201, *mod_reg_val_17(D)
	.syntax divided
@ 640 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #21, #8	@ D.65668, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65668, MEM[(volatile u32 *)_24]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #13	@ tmp203,
	str	r3, [sp]	@ tmp203, mod_reg_val
	mov	r3, #5	@ tmp204,
	str	r3, [sp, #4]	@ tmp204, mod_reg_addr
	ldr	r2, [sp]	@ D.65669, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65669, mod_reg_addr
	cmp	r2, r3	@ D.65669, D.65669
	ldrlt	r3, [sp]	@ D.65669, mod_reg_val
	ldrlt	r2, [sp, #4]	@ D.65669, mod_reg_addr
	addlt	r3, r3, r3, lsl #1	@ D.65669, D.65669, D.65669,
	addlt	r3, r3, r2	@ D.65669, D.65669, D.65669
	strlt	r3, [sp, #4]	@ D.65669, mod_reg_addr
	ldr	r3, [sp]	@ D.65669, mod_reg_val
	add	r2, r3, #1	@ D.65669, D.65669,
	cmp	r3, #7	@ D.65669,
	str	r2, [sp]	@ D.65669, mod_reg_val
	moveq	r3, #51	@ tmp208,
	streq	r3, [sp, #4]	@ tmp208, mod_reg_addr
	ldr	r3, [sp, #4]	@ D.65669, mod_reg_addr
	ldr	r3, [sp]	@ D.65669, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65669, mod_reg_addr
	add	r3, r3, r2	@ D.65669, D.65669, D.65669
	str	r3, [sp, #4]	@ D.65669, mod_reg_addr
	ldr	r3, [sp]	@ D.65669, mod_reg_val
	and	r3, r3, #9	@ D.65669, D.65669,
	str	r3, [sp, #4]	@ D.65669, mod_reg_addr
	ldr	r3, [sp]	@ D.65669, mod_reg_val
	.syntax divided
@ 642 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #13, #8	@ D.65668, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #72]	@ D.65668, MEM[(volatile u32 *)_30]
@ 0 "" 2
@ 643 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #5, #8	@ D.65668, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #76]	@ D.65668, MEM[(volatile u32 *)_35]
@ 0 "" 2
@ 644 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #3	@ D.65668, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65668, D.65668
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #80]	@ D.65668, MEM[(volatile u32 *)_40]
@ 0 "" 2
	.arm
	.syntax unified
	b	.L73	@
.L70:
	mov	r3, #76	@ tmp212,
	str	r3, [r7]	@ tmp212, *mod_reg_val_17(D)
	.syntax divided
@ 647 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #3, #8	@ D.65668, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65668, MEM[(volatile u32 *)_46]
@ 0 "" 2
@ 648 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #5	@ D.65668, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65668, D.65668
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #72]	@ D.65668, MEM[(volatile u32 *)_51]
@ 0 "" 2
	.arm
	.syntax unified
.L73:
	mov	r4, #0	@ i,
.L74:
	cmp	r4, r8	@ i, dlc
	beq	.L80	@,
	.syntax divided
@ 652 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r7]	@ *mod_reg_val_17(D), *mod_reg_val_17(D)
	ldrb	r2, [r9, r4]	@ zero_extendqisi2	@ D.65668, MEM[base: pdata_55(D), index: _97, offset: 0B]
	add	r3, r3, r4, lsl #2	@ D.65668, *mod_reg_val_17(D), i,
	add	r3, r5, r3	@ D.65672, mod_reg_addr, D.65668
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r2, [r3]	@ D.65668, MEM[(volatile u32 *)_63]
@ 0 "" 2
	.arm
	.syntax unified
	add	r4, r4, #1	@ i, i,
	b	.L74	@
.L80:
	.syntax divided
@ 654 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r6]	@ D.65668, *flag_6(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #64]	@ D.65668, MEM[(volatile u32 *)_68]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #12	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, r8, r9, pc}	@
	.fnend
	.size	can_asm_start_xmit, .-can_asm_start_xmit
	.align	2
	.global	can_asm_rx
	.syntax unified
	.arm
	.type	can_asm_rx, %function
can_asm_rx:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, lr}	@
	.save {r4, lr}
	.pad #8
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #64]	@ __v, MEM[(volatile u32 *)_3]
@ 0 "" 2
@ 660 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	tst	lr, #128	@ __v,
	uxtb	ip, lr	@ fi, __v
	beq	.L82	@,
	mov	lr, #84	@ tmp148,
	str	lr, [r2]	@ tmp148, *mod_reg_val_9(D)
	mov	r2, #11	@ tmp149,
	str	r2, [sp]	@ tmp149, mod_reg_val
	mov	r2, #3	@ tmp150,
	str	r2, [sp, #4]	@ tmp150, mod_reg_addr
	ldr	r2, [sp]	@ D.65683, mod_reg_val
	cmp	r2, #16	@ D.65683,
	ldreq	r2, [sp, #4]	@ D.65683, mod_reg_addr
	ldrne	r2, [sp]	@ D.65683, mod_reg_val
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r4, [r0, #68]	@ __v, MEM[(volatile u32 *)_12]
@ 0 "" 2
@ 664 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_16]
@ 0 "" 2
@ 665 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #76]	@ __v, MEM[(volatile u32 *)_21]
@ 0 "" 2
@ 666 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r0, [r0, #80]	@ __v, MEM[(volatile u32 *)_26]
@ 0 "" 2
@ 667 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r2, r2, #13	@ D.65682, __v,
	ubfx	r0, r0, #3, #5	@ D.65682, __v,,
	orr	r2, r2, r4, lsl #21	@ D.65682, D.65682, __v,
	orr	r2, r2, #-2147483648	@ D.65682, D.65682,
	orr	r2, r2, lr, lsl #5	@ D.65682, D.65682, __v,
	orr	r2, r2, r0	@ D.65682, D.65682, D.65682
	b	.L87	@
.L82:
	mov	lr, #76	@ tmp164,
	str	lr, [r2]	@ tmp164, *mod_reg_val_9(D)
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #68]	@ __v, MEM[(volatile u32 *)_37]
@ 0 "" 2
@ 671 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_40]
@ 0 "" 2
@ 672 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	ubfx	r2, r2, #5, #3	@ D.65682, __v,,
	orr	r2, r2, lr, lsl #3	@ D.65682, D.65682, __v,
.L87:
	str	r2, [r3]	@ D.65682, *id_32(D)
	strb	ip, [r1]	@ fi, *r_fi_47(D)
	add	sp, sp, #8	@,,
	@ sp needed	@
	pop	{r4, pc}	@
	.fnend
	.size	can_asm_rx, .-can_asm_rx
	.align	2
	.global	can_asm_probe
	.syntax unified
	.arm
	.type	can_asm_probe, %function
can_asm_probe:
	.fnstart
	@ args = 0, pretend = 0, frame = 112
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}	@
	.save {r4, r5, r6, r7, lr}
	.pad #140
	sub	sp, sp, #140	@,,
	add	r2, sp, #136	@ tmp140,,
	mvn	r3, #0	@ tmp136,
	mov	r7, r0	@ node, node
	mov	r4, r1	@ num, num
	str	r3, [r2, #-88]!	@ tmp136, res.a0
	mov	r3, #0	@ tmp139,
	str	r2, [sp, #16]	@ tmp140,
	mov	r2, r1	@, num
	str	r3, [sp, #20]	@ tmp139,
	str	r3, [sp, #12]	@ tmp139,
	str	r3, [sp, #8]	@ tmp139,
	str	r3, [sp, #4]	@ tmp139,
	str	r3, [sp]	@ tmp139,
	ldr	r1, .L100	@,
	ldr	r0, .L100+4	@,
	bl	__arm_smccc_smc	@
	ldr	r5, [sp, #48]	@ res.a0, res.a0
	cmp	r5, #0	@ res.a0,
	beq	.L89	@,
.L91:
	mvn	r0, #0	@ D.65702,
	b	.L90	@
.L89:
	mov	r2, r5	@, res.a0
	ldr	r1, .L100+8	@,
	mov	r0, r7	@, node
	bl	of_get_property	@
	mov	r6, r0	@ be_addr,
	mov	r0, r7	@, node
	bl	of_irq_find_parent	@
	cmp	r0, #0	@ p
	beq	.L91	@,
	mov	r3, #3	@ tmp147,
	str	r0, [sp, #64]	@ p, oirq.np
	str	r3, [sp, #68]	@ tmp147, oirq.args_count
	mov	r3, #1	@ tmp149,
	str	r3, [sp, #40]	@ tmp149, mod_reg_val
	mov	r3, #16	@ tmp150,
	str	r3, [sp, #44]	@ tmp150, mod_reg_addr
	movw	r3, #819	@ tmp151,
	ldr	r2, [sp, #44]	@ D.65702, mod_reg_addr
	add	r4, r4, #21	@ mod_reg_val, num,
	add	r1, sp, #64	@ tmp163,,
	mov	r0, r6	@, be_addr
	str	r5, [sp, #72]	@ res.a0, oirq.args
	str	r4, [sp, #76]	@ mod_reg_val, oirq.args
	mul	r2, r3, r2	@ D.65702, tmp151, D.65702
	str	r2, [sp, #40]	@ D.65702, mod_reg_val
	ldr	r2, [sp, #44]	@ D.65702, mod_reg_addr
	mul	r3, r3, r2	@ D.65702, tmp151, D.65702
	str	r3, [sp, #40]	@ D.65702, mod_reg_val
	ldr	r2, [sp, #44]	@ D.65702, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65702, mod_reg_val
	eor	r3, r3, r2	@ D.65702, D.65702, D.65702
	str	r3, [sp, #40]	@ D.65702, mod_reg_val
	ldr	r3, [sp, #40]	@ D.65702, mod_reg_val
	mov	r3, #15	@ tmp154,
	str	r3, [sp, #32]	@ tmp154, mod_reg_val
	mov	r3, #12	@ tmp155,
	str	r3, [sp, #36]	@ tmp155, mod_reg_addr
	ldr	r3, [sp, #36]	@ D.65702, mod_reg_addr
	ldr	r3, [sp, #32]	@ D.65702, mod_reg_val
	mov	r3, #4	@ tmp156,
	str	r3, [sp, #80]	@ tmp156, oirq.args
	bl	of_irq_parse_raw	@
	mov	r3, #8	@ tmp158,
	add	r0, sp, #64	@ tmp164,,
	str	r3, [sp, #24]	@ tmp158, mod_reg_val
	mov	r3, #9	@ tmp159,
	str	r3, [sp, #28]	@ tmp159, mod_reg_addr
	ldr	r3, [sp, #24]	@ D.65702, mod_reg_val
	cmp	r3, #8	@ D.65702,
	ldreq	r3, [sp, #28]	@ D.65702, mod_reg_addr
	ldrne	r3, [sp, #24]	@ D.65702, mod_reg_val
	bl	irq_create_of_mapping	@
.L90:
	add	sp, sp, #140	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, pc}	@
.L101:
	.align	2
.L100:
	.word	-65534
	.word	-1308622748
	.word	.LC0
	.fnend
	.size	can_asm_probe, .-can_asm_probe
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"reg\000"
	.ident	"GCC: (Linaro GCC 5.3-2016.05) 5.3.1 20160412"
	.section	.note.GNU-stack,"",%progbits
