#include "zte_lcd_common.h"

/*
*echo ff988100 > gwrite (0x13,0x29) or echo 51ff > dwrite (0x15,0x39)
*echo 5401 > gread(0x14,0x24), then cat gread
*dread (0x06) sometimes read nothing,return error
*file path: sys/lcd_reg_debug
*/

struct zte_lcd_reg_debug zte_lcd_reg_debug;
extern struct mdss_dsi_ctrl_pdata *g_zte_ctrl_pdata;
#define SYSFS_FOLDER_NAME "reg_debug"

static void zte_lcd_reg_rw_func(struct mdss_dsi_ctrl_pdata *ctrl, struct zte_lcd_reg_debug *reg_debug)
{
	int read_length = -1;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc write_lcd_cmd;

	if ((!reg_debug) || (!ctrl))
		return;

	write_lcd_cmd.dchdr.dtype = reg_debug->dtype;
	write_lcd_cmd.dchdr.last = 1;
	write_lcd_cmd.dchdr.vc = 0;
	write_lcd_cmd.dchdr.dlen = reg_debug->length;
	write_lcd_cmd.payload = (char *)reg_debug->wbuf;

	/*if debug this func,define ZTE_LCD_REG_DEBUG 1*/
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < reg_debug->length; i++)
		ZTE_LCD_INFO("rwbuf[%d]= %x\n", i, reg_debug->wbuf[i]);
	#endif

	memset(&cmdreq, 0, sizeof(cmdreq));
	if (reg_debug->is_read_mode) {
		read_length = reg_debug->wbuf[1];
		reg_debug->wbuf[1] = 0;
		write_lcd_cmd.dchdr.ack = 1;
		write_lcd_cmd.dchdr.wait = 5; /*5ms*/
		cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
		cmdreq.rbuf = (char *)reg_debug->rbuf;
		cmdreq.rlen = read_length;
	} else {
		write_lcd_cmd.dchdr.ack = 0;
		write_lcd_cmd.dchdr.wait = 5; /*5ms*/
		cmdreq.flags = CMD_REQ_COMMIT; /* CMD_REQ_COMMIT | CMD_CLK_CTRL*/
		cmdreq.rbuf = NULL;
		cmdreq.rlen = 0;
	}
	cmdreq.cmds = &write_lcd_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static void get_user_sapce_data(const char *buf, size_t count)
{
	int i = 0, length = 0;
	char lcd_status[ZTE_REG_LEN*2] = { 0 };

	if (count >= sizeof(lcd_status)) {
		ZTE_LCD_INFO("count=%zu,sizeof(lcd_status)=%zu\n", count, sizeof(lcd_status));
		return;
	}

	strlcpy(lcd_status, buf, count);
	memset(zte_lcd_reg_debug.wbuf, 0, ZTE_REG_LEN);
	memset(zte_lcd_reg_debug.rbuf, 0, ZTE_REG_LEN);

	/*if debug this func,define ZTE_LCD_REG_DEBUG 1*/
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < count; i++)
		ZTE_LCD_INFO("lcd_status[%d]=%c  %d\n", i, lcd_status[i], lcd_status[i]);
	#endif
	for (i = 0; i < count; i++) {
		if (isdigit(lcd_status[i]))
			lcd_status[i] -= '0';
		else if (isalpha(lcd_status[i]))
			lcd_status[i] -= (isupper(lcd_status[i]) ? 'A' - 10 : 'a' - 10);
	}
	for (i = 0, length = 0; i < (count-1); i = i+2, length++) {
		zte_lcd_reg_debug.wbuf[length] = lcd_status[i]*16 + lcd_status[1+i];
	}

	zte_lcd_reg_debug.length = length; /*length is use space write data number*/
}

static ssize_t sysfs_show_read(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.rbuf[i]);
		s += len;
		if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}
static ssize_t sysfs_store_dread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_ERROR("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE;
	zte_lcd_reg_debug.dtype = DTYPE_DCS_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x length = %x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_gread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_INFO("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE; /* if 1 read ,0 write*/

	if (zte_lcd_reg_debug.wbuf[1] >= 3)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ2;
	else if (zte_lcd_reg_debug.wbuf[1] == 2)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_GEN_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x num = %x\n", zte_lcd_reg_debug.dtype, zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_dwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE; /* if 1 read ,0 write*/

	if (length >= 3)
		zte_lcd_reg_debug.dtype = DTYPE_DCS_LWRITE;
	else if (length == 2)
		zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x,write cmd = 0x%02x,length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

static ssize_t sysfs_store_gwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE;

	if (length >= 3)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_LWRITE;
	else if (length == 2)
		zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE1;
	else
		zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x write cmd = 0x%02x length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

extern int mdss_dsi_set_clk_rates(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
static ssize_t sysfs_store_mipiclk(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0, tmp = -1, ret = count;
	u8 lanes = 0;

	tmp = kstrtoint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
		ret = tmp;
	} else {

		if (g_zte_ctrl_pdata->panel_data.panel_info.mipi.data_lane0)
			lanes++;
		if (g_zte_ctrl_pdata->panel_data.panel_info.mipi.data_lane1)
			lanes++;
		if (g_zte_ctrl_pdata->panel_data.panel_info.mipi.data_lane2)
			lanes++;
		if (g_zte_ctrl_pdata->panel_data.panel_info.mipi.data_lane3)
			lanes++;

		ZTE_LCD_INFO("count=%zu value=%d lanes=%d\n", count, value, lanes);

		g_zte_ctrl_pdata->pclk_rate =  value * lanes / 3;
		g_zte_ctrl_pdata->panel_data.panel_info.mipi.dsi_pclk_rate = g_zte_ctrl_pdata->pclk_rate;
		g_zte_ctrl_pdata->pclk_rate_bkp = g_zte_ctrl_pdata->pclk_rate;
		g_zte_ctrl_pdata->panel_data.panel_info.mipi.dsi_pclk_rate = g_zte_ctrl_pdata->pclk_rate;

		g_zte_ctrl_pdata->byte_clk_rate = value;
		g_zte_ctrl_pdata->byte_clk_rate_bkp = value;
		g_zte_ctrl_pdata->panel_data.panel_info.clk_rate = value * 8;

		mdss_dsi_set_clk_rates(g_zte_ctrl_pdata);
	}

	return ret;
}

static ssize_t sysfs_show_reserved(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.wbuf[i]);
		s += len;
	if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}
	len = snprintf(s, 100, "\n%s", zte_lcd_reg_debug.reserved);
	s += len;

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}

static ssize_t sysfs_store_reserved(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int i = 0, value = 0;
	int tmp = -1;

	get_user_sapce_data(buf, count);
	for (i = 0; i < zte_lcd_reg_debug.length; i++)
		ZTE_LCD_INFO("write data [%d]=0x%02x\n", i, zte_lcd_reg_debug.wbuf[i]);

	tmp = kstrtouint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
	} else {
		ZTE_LCD_INFO("count=%zu value=%d\n", count, value);
		snprintf(zte_lcd_reg_debug.reserved, ZTE_REG_LEN, "reserved str=%d", value);
	}
/******************************* add code here ************************************************/
	return count;
}

static ssize_t sysfs_show_initcode(struct device *d, struct device_attribute *attr, char *buf)
{
	const struct dsi_panel_cmds *on_cmds = NULL;
	struct dsi_cmd_desc *cmds = NULL;
	uint32_t cmd_cnt = 0;
	uint32_t count = 0;
	int i = 0;
	int j = 0;

	ZTE_LCD_INFO("%s", __func__);
	if (g_zte_ctrl_pdata == NULL) {
		ZTE_LCD_ERROR("%s ctrl_pdata is invalid param", __func__);
		return -EINVAL;
	}
	on_cmds = &g_zte_ctrl_pdata->on_cmds;
	cmd_cnt = on_cmds->cmd_cnt;

	if (cmd_cnt == 0) {
		ZTE_LCD_ERROR("%s cmd_cnt is invalid param", __func__);
		return -EINVAL;
	}

	for (i = 0; i < cmd_cnt; i++) {
		cmds = &on_cmds->cmds[i];
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.dtype);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.last);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.vc);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.ack);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.wait);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.dlen >> 8);
		count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->dchdr.dlen & 0xff);
		for (j = 0; j < cmds->dchdr.dlen; j++) {
			count += snprintf(buf + count, PAGE_SIZE - count, "%02x ", cmds->payload[j]);
		}
		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	return count;
}

int mdss_dsi_cmds_from_buf(struct dsi_panel_cmds *pcmds, char *buf, int blen)
{
	int len = 0;
	char *bp = NULL;
	struct dsi_ctrl_hdr *dchdr;
	int i = 0;
	int cnt = 0;

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			ZTE_LCD_ERROR("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_failed;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		ZTE_LCD_ERROR("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_failed;
	}

	pcmds->cmds = kcalloc(cnt, sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_failed;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}
	pcmds->link_state = g_zte_ctrl_pdata->on_cmds.link_state;

	ZTE_LCD_INFO("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_failed:
	return -ENOMEM;
}

void mdss_dsi_cmds_release(struct dsi_panel_cmds *pcmds)
{
	kfree(pcmds->cmds);
	kfree(pcmds->buf);
}

#define LCD_INITCODE_FILE_NAME "/sdcard/initcode.bin"
static ssize_t sysfs_store_initcode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct file *pfile = NULL;
	off_t fsize = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;
	char *cmd_buf = NULL;
	char *pb = NULL;
	struct dsi_panel_cmds on_cmds_temp;
	struct mdss_panel_timing *current_timing;
	struct dsi_panel_timing *pt;
	int ret = 0;

	ZTE_LCD_INFO("%s", __func__);

	pfile = filp_open(LCD_INITCODE_FILE_NAME, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		ZTE_LCD_ERROR("error occurred while opening file %s.", LCD_INITCODE_FILE_NAME);
		return -EIO;
	}

	fsize = file_inode(pfile)->i_size;
	cmd_buf = kzalloc(fsize, GFP_KERNEL);
	if (cmd_buf == NULL) {
		ZTE_LCD_ERROR("%s kzalloc failed", __func__);
		filp_close(pfile, NULL);
		return -ENOMEM;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, cmd_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	ret = mdss_dsi_cmds_from_buf(&on_cmds_temp, cmd_buf, fsize);
	if (ret < 0) {
		goto set_dsi_cmds_failed;
	}

	mdss_dsi_cmds_release(&g_zte_ctrl_pdata->on_cmds);

	/* param override */
	g_zte_ctrl_pdata->on_cmds = on_cmds_temp;

	current_timing = g_zte_ctrl_pdata->panel_data.current_timing;
	pt = container_of(current_timing, struct dsi_panel_timing, timing);
	pt->on_cmds = on_cmds_temp;

	return count;

set_dsi_cmds_failed:
	return ret;
}

static DEVICE_ATTR(dread, 0600, sysfs_show_read, sysfs_store_dread);
static DEVICE_ATTR(gread, 0600, sysfs_show_read, sysfs_store_gread);
static DEVICE_ATTR(dwrite, 0600, NULL, sysfs_store_dwrite);
static DEVICE_ATTR(gwrite, 0600, NULL, sysfs_store_gwrite);
static DEVICE_ATTR(mipiclk, 0600, NULL, sysfs_store_mipiclk);
static DEVICE_ATTR(initcode, 0600, sysfs_show_initcode, sysfs_store_initcode);
static DEVICE_ATTR(reserved, 0600, sysfs_show_reserved, sysfs_store_reserved);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_dread.attr,
	&dev_attr_gread.attr,
	&dev_attr_dwrite.attr,
	&dev_attr_gwrite.attr,
	&dev_attr_mipiclk.attr,
	&dev_attr_initcode.attr,
	&dev_attr_reserved.attr,
	NULL,
};

static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};

void zte_lcd_reg_debug_func(void)
{
	int ret = -1;

	struct kobject *vkey_obj = NULL;

	vkey_obj = kobject_create_and_add(SYSFS_FOLDER_NAME, g_zte_ctrl_pdata->zte_lcd_ctrl->kobj);
	if (!vkey_obj) {
		ZTE_LCD_ERROR("%s:unable to create kobject\n", __func__);
		return;
	}

	ret = sysfs_create_group(vkey_obj, &sysfs_attr_group);
	if (ret) {
		ZTE_LCD_ERROR("%s:failed to create attributes\n", __func__);
	}
}
