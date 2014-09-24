
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

extern unsigned int stay_waypoint_3D;
extern unsigned int land_distribution;

extern bool_t train_dictionary;
extern bool_t extract_distribution;
extern bool_t snapshot;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);

// Called by ahrs

//void opticFlowPropagateRates(struct Int32Rates* body_rate);
//void visionInputPropagate(struct Int32Rates body_rate, struct Int32Vect3 body_accel);
void start_timer_rates(void);
long end_timer_rates(void);


#endif
