
#ifndef _OPT_FL_LAND_H
#define _OPT_FL_LAND_H

#include "math/pprz_algebra_int.h"

// Settable by pluging
extern unsigned int imgWidth, imgHeight;
extern unsigned int tcp_port;
extern unsigned int adjust_factor;
extern unsigned int verbose;
extern float opt_trans_x;
extern float opt_trans_y;
extern float Velx, Vely;
extern float opt_angle_x_raw;
extern float opt_angle_y_raw;

extern float divergence;

extern bool_t snapshot;

extern int32_t fast_tune;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);

// Called by ahrs

//void opticFlowPropagateRates(struct Int32Rates* body_rate);
//void visionInputPropagate(struct Int32Rates body_rate, struct Int32Vect3 body_accel);
void start_timer_rates(void);
long end_timer_rates(void);


#endif
