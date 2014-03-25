
#ifndef _OPENCV_H
#define _OPENCV_H

// Settable by pluging
extern unsigned int imgWidth, imgHeight;
extern unsigned int tcp_port;
extern unsigned int adjust_factor;
extern unsigned int verbose;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);

#endif
