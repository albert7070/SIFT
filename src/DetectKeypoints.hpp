#pragma once

#include "SIFTParameters.hpp"



#define CONTRAST_TH 20

#define EIGEN_RATIO_TH 12


#define SUPPRESS_NONMAX_SIZE 3
#define SNMX_OFFSET SUPPRESS_NONMAX_SIZE/2

#define CULL_SIZE 7
#define CULL_OFFSET CULL_SIZE/2


ap_int<16> ABS(ap_int<16> a);





#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL,  pKeypointOut:SEQUENTIAL, pEigenValuesOut:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT],  pKeypointOut[0:WIDTH*HEIGHT], pEigenValuesOut[0:WIDTH*HEIGHT])

#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE, pKeypointOut:AXIDMA_SIMPLE)

void sift_detect(	ap_uint<8>* pSrcImage,
					ap_uint<8>* pKeypointOut,
					Fixed_t* 	pEigenValuesOut

					);


void build_DOG_hls(	hls::stream< Pixel_t > &pSrcImage,
					hls::stream< Pixel_t > &pGaussian1,
					hls::stream< Pixel_t > &pGaussian2,
					hls::stream< Pixel_t > &pGaussian3,
					hls::stream< ap_int<16> > &pDOG1,
					hls::stream< ap_int<16> > &pDOG2,
					hls::stream< ap_int<16> > &pDOG3);


void detect_extrema_hls(	hls::stream< ap_int<16> > &pDOG1,
							hls::stream< ap_int<16> > &pDOG2,
							hls::stream< ap_int<16> > &pDOG3,
							hls::stream< ap_uint<8> > &pKeypointMap1,
							hls::stream< Fixed_t > &pEigenRatio,
							int contrastTh
						);


Fixed_t IsOnEdge(ap_int<16> window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE]);

bool IsLocalExtremum(ap_int<16> window[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE]);
