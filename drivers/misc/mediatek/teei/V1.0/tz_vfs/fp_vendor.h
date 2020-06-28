#ifndef __FP_VENDOR_H__
#define __FP_VENDOR_H__

enum {
    FP_VENDOR_INVALID = 0,
    FPC_VENDOR,
    LEADCORE_VENDOR,
    SUNWAVE_VENDOR,
    GOODIX_VENDOR,
	MICROARRAY_VENDOR, //zhangkaiyuan@wind-mobi.com 20171009
};

int get_fp_vendor(void);

#endif  /*__FP_VENDOR_H__*/
