#ifndef FAST_H
#define FAST_H

typedef struct { int x, y; } xyFAST; 
typedef unsigned char byte;

int fast12_corner_score(const byte* p, const int pixel[], int bstart);

xyFAST* fast12_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

int* fast12_score(const byte* i, int stride, xyFAST* corners, int num_corners, int b);

xyFAST* fast12_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

xyFAST* nonmax_suppression(const xyFAST* corners, const int* scores, int num_corners, int* ret_num_nonmax);


#endif
