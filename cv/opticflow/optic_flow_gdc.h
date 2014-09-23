#ifndef OPTIC
#define OPTIC

typedef struct flowPoint
{
	int x;
	int y;
	int prev_x;
	int prev_y;
	int dx;
	int dy;
	int new_dx;
	int new_dy;
//	double P[16]; // represents a diagonal 4x4 matrix
//	double Q[16]; // represents a diagonal 4x4 matrix
//	double R[16]; // represents a diagonal 4x4 matrix
//	double K[16]; // represents a diagonal 4x4 matrix
//	int n_observations;
} flowPoint;

typedef struct detectedPoint
{
	int x;
	int y;
} detectedPoint;

int getMaximum(int * Im);
int getMinimum(int * Im);
void getGradientPixelWH(unsigned char *frame_buf, int x, int y, int* dx, int* dy);
void getSimpleGradient(unsigned char* frame_buf, int* DX, int* DY);
void multiplyImages(int* ImA, int* ImB, int* ImC, int width, int height);
void getImageDifference(int* ImA, int* ImB, int* ImC, int width, int height);
int calculateError(int* ImC, int width, int height);
void printIntMatrix(int* Matrix, int width, int height);
void printIntMatrixPart(int* Matrix, int width, int height, int n_cols, int n_rows, int x_center, int y_center);
void smoothGaussian(int* Src, int* Dst);
void getHarris(int* DXX, int* DXY, int* DYY, int* Harris);
int findLocalMaxima(int* Harris, int max_val, int MAX_POINTS, int* p_x, int* p_y, int suppression_distance_squared, int* n_found_points);
void excludeArea(unsigned int* Mask, int x, int y, int suppression_distance_squared);
void thresholdImage(int* Harris, int max_val, int max_factor);
int findCorners(unsigned char *frame_buf, int MAX_POINTS, int *x, int *y, int suppression_distance_squared, int* n_found_points, int mark_points, int imW, int imH);
int findActiveCorners(unsigned char *frame_buf, unsigned int GRID_ROWS, int ONLY_STOPPED, int *x, int *y, int* active, int* n_found_points, int mark_points, int imW, int imH);
void getSubPixel(int* Patch, unsigned char* buf, int center_x, int center_y, int half_window_size, int subpixel_factor);
int calculateG(int* G, int* DX, int* DY, int half_window_size);
void getGradientPatch(int* Patch, int* DX, int* DY, int half_window_size);
int getSumPatch(int* Patch, int size);
void showFlow(unsigned char * frame_buf, int* x, int* y, int* status, int n_found_points, int* new_x, int* new_y, int imgW, int imgH);
int opticFlowLK(unsigned char * new_image_buf, unsigned char * old_image_buf, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int* status, int half_window_size, int max_iterations);
void MatMul(float* Mat1, float* Mat2, float* Mat3, int MatW, int MatH);
void MatVVMul(float* MVec, float** Mat, float* Vec, int MatW, int MatH);
void ScaleAdd(float* Mat3, float* Mat1, float Scale, float* Mat2, int MatW, int MatH);
int dsvd(float **a, int m, int n, float *w, float **v);
void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x);
void svdSolve(float *x_svd, float **u, int m, int n, float *b);
void fitLinearFlowField(float* pu, float* pv, float* divergence_error, int *x, int *y, int *dx, int *dy, int count, int n_samples, float* min_error_u, float* min_error_v, int n_iterations, float error_threshold, int *n_inlier_minu, int *n_inlier_minv);
void extractInformationFromLinearFlowField(float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float* pu, float* pv, int imgWidth, int imgHeight, int* DIV_FILTER);
void slopeEstimation(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v);
void quick_sort (float *a, int n);
void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH);
void yuyv_to_rgb24 (int width, int height, unsigned char *src, unsigned char *dst);
void uyvy_to_rgb24 (int width, int height, unsigned char *src, unsigned char *dst);
void setPointsToFlowPoints(struct flowPoint flow_points[], struct detectedPoint detected_points[], int *flow_point_size, int *count, int MAX_COUNT);
void findPoints(unsigned char *gray_frame, unsigned char *frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[]);
void trackPoints(unsigned char *frame, unsigned char *prev_frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points0[],struct detectedPoint detected_points1[], int *x, int *y, int *new_x, int *new_y, int *dx, int *dy, int *status);
void analyseTTI(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float *divergence_error, int *x, int *y, int *dx, int *dy, int *n_inlier_minu, int *n_inlier_minv, int count, int imW, int imH, int* DIV_FILTER);
void lineDivergence(float *divergence, int *x, int *y, int *new_x, int *new_y, int count);
void subimage(unsigned char *gray_frame, unsigned char *subframe, int subimH, int subimW, int wInit, int hInit);
void findDistributedPoints(unsigned char *gray_frame, unsigned char *frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[], int *status);
void trackDistributedPoints(unsigned char *frame, unsigned char *prev_frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[], int *x, int *y, int *new_x, int *new_y, int *dx, int *dy, int *status);
void OFfilter(float *opt_angle_x_raw, float *opt_angle_y_raw, struct flowPoint flow_points[], int count, int OF_FilterType);
void YUV422TORGB(unsigned char *YUV, unsigned char *RGB, unsigned char *GRAY, int Width, int Height);
void saveSingleImageDataFile(unsigned char *frame_buf, int width, int height);
void DictionaryTrainingYUV(float ****color_words, unsigned char *frame, int n_words, int patch_size, int *learned_samples, int n_samples_image, float alpha, int Width, int Height, int *filled);
//void DistributionExtraction(float ****color_words, unsigned char *frame, float* word_distribution, int n_words, int patch_size, int n_samples_image, int RANDOM_SAMPLES, int Width, int Height);
#endif
