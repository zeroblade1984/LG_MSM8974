#ifndef __BROADCAST_TCC3530_H__
#define __BROADCAST_TCC3530_H__

#include "../broadcast_dmb_drv_ifdef.h"

int tcc353x_power_on(void);
int tcc353x_power_off(void);
int tcc353x_is_power_on(void);
int tcc353x_select_antenna(unsigned int sel);

extern int broadcast_tcc353x_drv_if_power_on(void);
extern int broadcast_tcc353x_drv_if_power_off(void);
extern int broadcast_tcc353x_drv_if_open(void);
extern int broadcast_tcc353x_drv_if_close(void);
extern int broadcast_tcc353x_drv_if_set_channel(struct broadcast_dmb_set_ch_info *udata);
extern int broadcast_tcc353x_drv_if_resync(void);
extern int broadcast_tcc353x_drv_if_detect_sync(struct broadcast_dmb_sync_info *udata);
extern int broadcast_tcc353x_drv_if_get_sig_info(struct broadcast_dmb_control_info *bb_info);
extern int broadcast_tcc353x_drv_if_get_ch_info(struct broadcast_dmb_ch_info *ch_info);
extern int broadcast_tcc353x_drv_if_get_dmb_data(struct broadcast_dmb_data_info *pdmb_data);
extern int broadcast_tcc353x_drv_if_reset_ch(void);
extern int broadcast_tcc353x_drv_if_user_stop(int mode);
extern int broadcast_tcc353x_drv_if_select_antenna(unsigned int sel);
extern int broadcast_tcc353x_drv_if_isr(void);
extern int broadcast_tcc353x_drv_if_read_control(char *buf, unsigned int size);
extern int broadcast_tcc353x_drv_if_get_mode(unsigned short *mode);
#endif /*__BROADCAST_TCC3530_H__*/
