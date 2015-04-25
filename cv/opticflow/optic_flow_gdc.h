/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file paparazzi/sw/ext/ardrone2_vision/cv/opticflow/optic_flow_gdc.h
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef OPTIC
#define OPTIC
void multiplyImages(int* ImA, int* ImB, int* ImC, int width, int height);
void getImageDifference(int* ImA, int* ImB, int* ImC, int width, int height);
void getSubPixel_gray(int* Patch, unsigned char* frame_buf, int center_x, int center_y, int half_window_size, int subpixel_factor);
void getGradientPatch(int* Patch, int* DX, int* DY, int half_window_size);
int getSumPatch(int* Patch, int size);
int calculateG(int* G, int* DX, int* DY, int half_window_size);
int calculateError(int* ImC, int width, int height);
int opticFlowLK(unsigned char * new_image_buf, unsigned char * old_image_buf, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int* status, int half_window_size, int max_iterations);
void quick_sort (float *a, int n);
void quick_sort_int (int *a, int n);
void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH);
void OFfilter(float *OFx, float *OFy, float dx, float dy, int count, int OF_FilterType);
void MatVVMul(float* MVec, float** Mat, float* Vec, int MatW, int MatH);
void ScaleAdd(float* Mat3, float* Mat1, float Scale, float* Mat2, int MatW, int MatH);
int dsvd(float **a, int m, int n, float *w, float **v);
void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x);
void svdSolve(float *x_svd, float **u, int m, int n, float *b);
void fitLinearFlowField(float* pu, float* pv, float* divergence_error, int *x, int *y, int *dx, int *dy, int count, int n_samples, float* min_error_u, float* min_error_v, int n_iterations, float error_threshold, int *n_inlier_minu, int *n_inlier_minv);
void extractInformationFromLinearFlowField(float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float* pu, float* pv, int imgWidth, int imgHeight, int *DIV_FILTER);
void slopeEstimation(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v);
void analyseTTI(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float *divergence_error, int *x, int *y, int *dx, int *dy, int *n_inlier_minu, int *n_inlier_minv, int count, int imW, int imH, int *DIV_FILTER);
void saveSingleImageDataFile(unsigned char *frame_buf, int width, int height, char filename[100]);
void DictionaryTrainingYUV(float ****color_words, unsigned char *frame, int n_words, int patch_size, int *learned_samples, int n_samples_image, float alpha, int Width, int Height, int *filled);
void DistributionExtraction(float ****color_words, unsigned char *frame, float* word_distribution, int n_words, int patch_size, int n_samples_image, int RANDOM_SAMPLES, int Width, int Height, int border_width, int border_height, float *error_appearance);
void subimage_extraction(unsigned char *frame, unsigned char *sub_frame, int imgW, int imgH, int type, int n_reg, int in_reg_h, int in_reg_w);
#endif
