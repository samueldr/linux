 /*
*
* This file is provided under a dual BSD/GPL license.  When using or
* redistributing this file, you may do so under either license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#ifndef __SUNXI_CAN_ASM_H__
#define __SUNXI_CAN_ASM_H__

#include <linux/can/dev.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/arm-smccc.h>

#define SUNXI_REG_MSEL_ADDR	0x0000
#define SUNXI_REG_CMD_ADDR	0x0004
#define SUNXI_REG_STA_ADDR	0x0008
#define SUNXI_REG_INT_ADDR	0x000c
#define SUNXI_REG_INTEN_ADDR	0x0010
#define SUNXI_REG_BTIME_ADDR	0x0014
#define SUNXI_REG_TEWL_ADDR	0x0018
#define SUNXI_REG_ERRC_ADDR	0x001c
#define SUNXI_REG_RMCNT_ADDR	0x0020
#define SUNXI_REG_RBUFSA_ADDR	0x0024
#define SUNXI_REG_BUF0_ADDR	0x0040
#define SUNXI_REG_BUF1_ADDR	0x0044
#define SUNXI_REG_BUF2_ADDR	0x0048
#define SUNXI_REG_BUF3_ADDR	0x004c
#define SUNXI_REG_BUF4_ADDR	0x0050
#define SUNXI_REG_BUF5_ADDR	0x0054
#define SUNXI_REG_BUF6_ADDR	0x0058
#define SUNXI_REG_BUF7_ADDR	0x005c
#define SUNXI_REG_BUF8_ADDR	0x0060
#define SUNXI_REG_BUF9_ADDR	0x0064
#define SUNXI_REG_BUF10_ADDR	0x0068
#define SUNXI_REG_BUF11_ADDR	0x006c
#define SUNXI_REG_BUF12_ADDR	0x0070
#define SUNXI_REG_ACPC_ADDR	0x0028
#define SUNXI_REG_ACPM_ADDR	0x002C
#define SUNXI_REG_RBUF_RBACK_START_ADDR	0x0180
#define SUNXI_REG_RBUF_RBACK_END_ADDR	0x01b0


#define SUNXI_MSEL_SLEEP_MODE		(0x01 << 4)
#define SUNXI_MSEL_WAKE_UP		(0x00 << 4)
#define SUNXI_MSEL_SINGLE_FILTER	(0x01 << 3)
#define SUNXI_MSEL_DUAL_FILTERS		(0x00 << 3)
#define SUNXI_MSEL_LOOPBACK_MODE	BIT(2)
#define SUNXI_MSEL_LISTEN_ONLY_MODE	BIT(1)
#define SUNXI_MSEL_RESET_MODE		BIT(0)

#define SUNXI_CMD_BUS_OFF_REQ	BIT(5)
#define SUNXI_CMD_SELF_RCV_REQ	BIT(4)
#define SUNXI_CMD_CLEAR_OR_FLAG	BIT(3)
#define SUNXI_CMD_RELEASE_RBUF	BIT(2)
#define SUNXI_CMD_ABORT_REQ	BIT(1)
#define SUNXI_CMD_TRANS_REQ	BIT(0)


#define SUNXI_STA_BIT_ERR	(0x00 << 22)
#define SUNXI_STA_FORM_ERR	(0x01 << 22)
#define SUNXI_STA_STUFF_ERR	(0x02 << 22)
#define SUNXI_STA_OTHER_ERR	(0x03 << 22)
#define SUNXI_STA_MASK_ERR	(0x03 << 22)
#define SUNXI_STA_ERR_DIR	BIT(21)
#define SUNXI_STA_ERR_SEG_CODE	(0x1f << 16)
#define SUNXI_STA_START		(0x03 << 16)
#define SUNXI_STA_ID28_21	(0x02 << 16)
#define SUNXI_STA_ID20_18	(0x06 << 16)
#define SUNXI_STA_SRTR		(0x04 << 16)
#define SUNXI_STA_IDE		(0x05 << 16)
#define SUNXI_STA_ID17_13	(0x07 << 16)
#define SUNXI_STA_ID12_5	(0x0f << 16)
#define SUNXI_STA_ID4_0		(0x0e << 16)
#define SUNXI_STA_RTR		(0x0c << 16)
#define SUNXI_STA_RB1		(0x0d << 16)
#define SUNXI_STA_RB0		(0x09 << 16)
#define SUNXI_STA_DLEN		(0x0b << 16)
#define SUNXI_STA_DATA_FIELD	(0x0a << 16)
#define SUNXI_STA_CRC_SEQUENCE	(0x08 << 16)
#define SUNXI_STA_CRC_DELIMITER	(0x18 << 16)
#define SUNXI_STA_ACK		(0x19 << 16)
#define SUNXI_STA_ACK_DELIMITER	(0x1b << 16)
#define SUNXI_STA_END		(0x1a << 16)
#define SUNXI_STA_INTERMISSION	(0x12 << 16)
#define SUNXI_STA_ACTIVE_ERROR	(0x11 << 16)
#define SUNXI_STA_PASSIVE_ERROR	(0x16 << 16)
#define SUNXI_STA_TOLERATE_DOMINANT_BITS	(0x13 << 16)
#define SUNXI_STA_ERROR_DELIMITER	(0x17 << 16)
#define SUNXI_STA_OVERLOAD	(0x1c << 16)
#define SUNXI_STA_BUS_OFF	BIT(7)
#define SUNXI_STA_ERR_STA	BIT(6)
#define SUNXI_STA_TRANS_BUSY	BIT(5)
#define SUNXI_STA_RCV_BUSY	BIT(4)
#define SUNXI_STA_TRANS_OVER	BIT(3)
#define SUNXI_STA_TBUF_RDY	BIT(2)
#define SUNXI_STA_DATA_ORUN	BIT(1)
#define SUNXI_STA_RBUF_RDY	BIT(0)

#define SUNXI_INT_BUS_ERR	BIT(7)
#define SUNXI_INT_ARB_LOST	BIT(6)
#define SUNXI_INT_ERR_PASSIVE	BIT(5)
#define SUNXI_INT_WAKEUP	BIT(4)
#define SUNXI_INT_DATA_OR	BIT(3)
#define SUNXI_INT_ERR_WRN	BIT(2)
#define SUNXI_INT_TBUF_VLD	BIT(1)
#define SUNXI_INT_RBUF_VLD	BIT(0)


#define SUNXI_INTEN_BERR	BIT(7)
#define SUNXI_INTEN_ARB_LOST	BIT(6)
#define SUNXI_INTEN_ERR_PASSIVE	BIT(5)
#define SUNXI_INTEN_WAKEUP	BIT(4)
#define SUNXI_INTEN_OR		BIT(3)
#define SUNXI_INTEN_ERR_WRN	BIT(2)
#define SUNXI_INTEN_TX		BIT(1)
#define SUNXI_INTEN_RX		BIT(0)

#define SUNXI_ERR_INRCV		(0x1 << 5)
#define SUNXI_ERR_INTRANS	(0x0 << 5)

#define SUNXI_FILTER_CLOSE	0
#define SUNXI_SINGLE_FLTER_MODE	1
#define SUNXI_DUAL_FILTER_MODE	2

#define SUNXI_MSG_EFF_FLAG	BIT(7)
#define SUNXI_MSG_RTR_FLAG	BIT(6)

#define SUNXI_CAN_MAX_IRQ	20
#define SUNXI_MODE_MAX_RETRIES	100

void can_asm_write_cmdreg(u32 mod_reg_val, volatile void __iomem *mod_reg_addr, volatile void __iomem *reg_addr);
void can_asm_start(volatile void __iomem *mod_reg_addr, u32 ctrlmode, int num);
void can_asm_set_bittiming(volatile void __iomem *mod_reg_addr, u32 mod_reg_val, u32 *cfg, volatile void __iomem *reg_addr);
void can_asm_clean_transfer_err(volatile void __iomem *addr, u16 *t_err, u16 *r_err);
void can_asm_rx(volatile void __iomem *addr, u8 *r_fi, u32 *dreg, u32 *id);

void can_asm_start_xmit(volatile void __iomem *mod_reg_addr, volatile void __iomem *addr, u32 id, u32 *flag, u32 *mod_reg_val, u8 *pdata);
int can_asm_probe(struct device_node *node, int num);
void can_asm_fun0(int num);
void can_asm_fun1(int num);
void can_asm_fun2(int num);
void can_asm_fun3(int num);
void can_asm_fun4(int num);
void __iomem *can_asm_fun5(int num);
#endif
