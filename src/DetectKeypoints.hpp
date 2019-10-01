#pragma once

#include "SIFTParameters.hpp"

#define FILTER_SIZE1 5
#define FILTER_OFFS1 FILTER_SIZE1/2


#define FILTER_SIZE2 11
#define FILTER_OFFS2 FILTER_SIZE2/2


#define FILTER_SIZE3 19
#define FILTER_OFFS3 FILTER_SIZE3/2

#define CONTRAST_TH 20

#define EIGEN_RATIO_TH 12


#define SUPPRESS_NONMAX_SIZE 3
#define SNMX_OFFSET SUPPRESS_NONMAX_SIZE/2

#define CULL_SIZE 7
#define CULL_OFFSET CULL_SIZE/2

#pragma SDS data access_pattern(pDOG1:SEQUENTIAL,  pDOG2:SEQUENTIAL, pDOG3:SEQUENTIAL)
#pragma SDS data copy( pDOG1[0:WIDTH*HEIGHT], pDOG2[0:WIDTH*HEIGHT], pDOG3[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pKeypointMap1:SEQUENTIAL, pKeypointMap2:SEQUENTIAL)
#pragma SDS data copy(pKeypointMap1[0:WIDTH*HEIGHT], pKeypointMap2[0:WIDTH*HEIGHT])

void detect_extrema(	ap_int<16>* pDOG1,
						ap_int<16>* pDOG2,
						ap_int<16>* pDOG3,
						ap_uint<8>* pKeypointMap1,
						ap_uint<8>* pKeypointMap2
						);


bool IsLocalExtremum(ap_int<16> window[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE]);



Fixed_t IsOnEdge(ap_int<16> window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE]);




//============================= HLS Functions ========================================== //



void detect_extrema_hls(	hls::stream< ap_int<16> > &pDOG1,
							hls::stream< ap_int<16> > &pDOG2,
							hls::stream< ap_int<16> > &pDOG3,
							hls::stream< ap_uint<16> > &pKeypointMap1,
							hls::stream< Fixed_t > &pEigenRatio,
							int contrastTh
						);


void combine_keypoints_hls(	hls::stream< ap_uint<16> > &pKeypointImage,
							hls::stream< Keypoint_t > &pKeypoints );


void build_DOG_hls(	hls::stream< Pixel_t > &pSrcImage,
					hls::stream< Pixel_t > &pGaussian1,
					hls::stream< Pixel_t > &pGaussian2,
					hls::stream< Pixel_t > &pGaussian3,
					hls::stream< ap_int<16> > &pDOG1,
					hls::stream< ap_int<16> > &pDOG2,
					hls::stream< ap_int<16> > &pDOG3);




void culling_hls(	hls::stream< ap_uint<16> > &pKeypointMap1,
					hls::stream< Fixed_t > &pEigenRatio,
					hls::stream< ap_uint<16> > &pKeypointOut
						);

bool IsBestKeypoint(Fixed_t EigWindow[CULL_SIZE][CULL_SIZE], ap_uint<16> KPWindow[CULL_SIZE][CULL_SIZE]);



#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL, pGradients:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT], pGradients[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pKeypoints:SEQUENTIAL)
#pragma SDS data copy( pKeypoints[0:MAX_KEYPOINTS])

#pragma SDS data data_mover(pKeypoints:AXIDMA_SIMPLE)
//#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)


void sift_detect(	ap_uint<8>* pSrcImage,
					Grad_t* pGradients,
					Keypoint_t* pKeypoints,
					int contrastTh
						);



int RoundToInt(Fixed_t a);

// HLS function for gaussian blur



// Kernel
Pixel_t FilterKernelOperator1(Pixel_t window[FILTER_SIZE1][FILTER_SIZE1]);



// Kernel
Pixel_t FilterKernelOperator2(Pixel_t window[FILTER_SIZE2][FILTER_SIZE2]);



// Kernel
Pixel_t FilterKernelOperator3(Pixel_t window[FILTER_SIZE3][FILTER_SIZE3]);



// ======================================//

void GaussianBlurHLS(	hls::stream< Pixel_t > &SrcImageStream,
						hls::stream< Pixel_t > &Gaussian1,
						hls::stream< Pixel_t > &Gaussian2,
						hls::stream< Pixel_t > &Gaussian3,
						hls::stream< Pixel_t > &DupGaussian2,
						hls::stream< Pixel_t > &DupImageStream);




// ====================== COMPUTE ORIENTATION ==================================//


// Signum function
ap_int<2> sign(ap_int<16> a);


ap_int<16> ABS(ap_int<16> a);


Grad_t ComputeGradientKernel(Pixel_t Window[CO_SIZE][CO_SIZE]);



void ComputeGradientsHLS(	hls::stream< ap_uint<8> > &pGaussian2,
							hls::stream<Grad_t> &pGrad
							);


