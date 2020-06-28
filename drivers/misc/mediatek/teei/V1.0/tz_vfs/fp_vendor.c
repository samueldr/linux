#include "fp_vendor.h"
#include<linux/types.h>
#include "../tz_driver/include/nt_smc_call.h"
#include<linux/kernel.h>
#include <linux/mutex.h>


#define FPC_VENDOR_ID       0x01
#define SUNWAVE_VENDOR_ID    0x02
#define GOODIX_VENDOR_ID    0x03
#define MICROARRAY_ID  0x04  //zhangkaiyuan@wind-mobi.com 20171009
#define LEADCORE_VENDOR_ID    0xff


int fp_vendor_active = 0;
int fp_vendor = FP_VENDOR_INVALID;
static DEFINE_MUTEX(fp_vendor_lock);

int get_fp_vendor(void)
{
    uint64_t fp_vendor_id_64 = 0;
    uint32_t *p_temp = NULL;
    uint32_t fp_vendor_id_32 = 0;


    mutex_lock(&fp_vendor_lock);
    if(fp_vendor_active) {
        mutex_unlock(&fp_vendor_lock);
        return fp_vendor;
    }

    get_t_device_id(&fp_vendor_id_64);
    printk("%s:%d lichen--->fp_vendor_id_64 = 0x%llx\n", __func__, __LINE__,fp_vendor_id_64);
    p_temp = (uint32_t *)&fp_vendor_id_64;
    fp_vendor_id_32 = *p_temp;
//     fp_vendor_id_32 = (fp_vendor_id_32 >> 8) & 0xff;
    printk("%s:%d lichen--->fp_vendor_id_32 = 0x%x\n", __func__, __LINE__, fp_vendor_id_32);

    switch(fp_vendor_id_32) {
        case FPC_VENDOR_ID:
            fp_vendor = FPC_VENDOR;
            break;
	case SUNWAVE_VENDOR_ID:
            fp_vendor = SUNWAVE_VENDOR;
	    printk("lichen--->it's sunwave chip");
            break;
	case GOODIX_VENDOR_ID:
            fp_vendor = GOODIX_VENDOR;
	   
	    printk("lichen--->it's goodix chip");
            break;

        case LEADCORE_VENDOR_ID:
            fp_vendor = LEADCORE_VENDOR;
	 	
	    printk("lichen--->it's leadcore chip");
            break;
//zhangkaiyuan@wind-mobi.com 20171009 begin			
		case MICROARRAY_ID:
            fp_vendor = MICROARRAY_VENDOR;
	 	
	    printk("lichen--->it's microarray chip");
            break;
//zhangkaiyuan@wind-mobi.com 20171009 end
        default:
            fp_vendor = FP_VENDOR_INVALID;
            break;
    }
    fp_vendor_active = 1;

    //fp_vendor = FPC_VENDOR;
  //  fp_vendor = GOODIX_VENDOR;
    printk("%s:%d lichen-->fp_vendor = 0x%x\n", __func__, __LINE__, fp_vendor);
    mutex_unlock(&fp_vendor_lock);
    return fp_vendor;
}
//liukangping 20170112 end
