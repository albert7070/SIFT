#pragma once




#include "SIFTParameters.hpp"


#define FILTER_SIZE1 5
#define FILTER_OFFS1 FILTER_SIZE1/2


#define FILTER_SIZE2 11
#define FILTER_OFFS2 FILTER_SIZE2/2


#define FILTER_SIZE3 19
#define FILTER_OFFS3 FILTER_SIZE3/2

typedef ap_fixed<16,9,AP_RND_ZERO> Mask_t ;

//typedef Fixed_t Mask_t ;

int RoundToInt(Fixed_t a);



Pixel_t FilterKernelOperator1(Pixel_t window[FILTER_SIZE1][FILTER_SIZE1]);

Pixel_t FilterKernelOperator2(Pixel_t window[FILTER_SIZE2][FILTER_SIZE2]);

Pixel_t FilterKernelOperator3(Pixel_t window[FILTER_SIZE3][FILTER_SIZE3]);




// ======================================//

void GaussianBlurHLS(	hls::stream< Pixel_t > &SrcImageStream,
						hls::stream< Pixel_t > &Gaussian1,
						hls::stream< Pixel_t > &Gaussian2,
						hls::stream< Pixel_t > &Gaussian3,

						hls::stream< Pixel_t > &DupImageStream);




