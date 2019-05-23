#pragma once




#include "SIFTParameters.hpp"


#define FILTER_SIZE1 5
#define FILTER_OFFS1 FILTER_SIZE1/2


#define FILTER_SIZE2 11
#define FILTER_OFFS2 FILTER_SIZE2/2


#define FILTER_SIZE3 19
#define FILTER_OFFS3 FILTER_SIZE3/2

//typedef ap_fixed<16,1,AP_RND_ZERO> Mask_t ;

typedef Fixed_t Mask_t ;

int RoundToInt(Fixed_t a);



Pixel_t FilterKernelOperator1(Pixel_t window[FILTER_SIZE1][FILTER_SIZE1]);

Pixel_t FilterKernelOperator2(Pixel_t window[FILTER_SIZE2][FILTER_SIZE2]);

Pixel_t FilterKernelOperator3(Pixel_t window[FILTER_SIZE3][FILTER_SIZE3]);




// ======================================//

void GaussianBlurHLS(	hls::stream< Pixel_t > &SrcImageStream,
						hls::stream< Pixel_t > &Gaussian1,
						hls::stream< Pixel_t > &Gaussian2,
						hls::stream< Pixel_t > &Gaussian3,
						hls::stream< Pixel_t > &DupGaussian2,
						hls::stream< Pixel_t > &DupImageStream);



// Wrapper

#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL,  pGaussian1:SEQUENTIAL,  pGaussian2:SEQUENTIAL,  pGaussian3:SEQUENTIAL)
#pragma SDS data copy( pSrcImage[0:WIDTH*HEIGHT], pGaussian1[0:WIDTH*HEIGHT], pGaussian2[0:WIDTH*HEIGHT], pGaussian3[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pDupGaussian2:SEQUENTIAL,  pDupSrcImage:SEQUENTIAL)
#pragma SDS data copy( pDupGaussian2[0:WIDTH*HEIGHT], pDupSrcImage[0:WIDTH*HEIGHT])

#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)


void gaussian_blur(	ap_uint<16>* pSrcImage,
					ap_uint<16>* pGaussian1,
					ap_uint<16>* pGaussian2,
					ap_uint<16>* pGaussian3,
					ap_uint<16>* pDupGaussian2,
					ap_uint<16>* pDupSrcImage);
