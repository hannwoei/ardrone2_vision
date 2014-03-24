
#ifndef _OPT_FL_LAND_H
#define _OPT_FL_LAND_H

// Settable by pluging
extern unsigned int imgWidth, imgHeight;
extern unsigned int tcp_port;
extern unsigned int adjust_factor;
extern unsigned int verbose;
extern int opt_trans_x;
extern int opt_trans_y;
extern int opt_angle_x_raw;
extern int opt_angle_y_raw;

extern float divergence;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);

#endif
