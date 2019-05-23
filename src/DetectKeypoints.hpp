#pragma once

#include "SIFTParameters.hpp"



#define CONTRAST_TH 20

#define EIGEN_RATIO_TH 12


#define SUPPRESS_NONMAX_SIZE 3
#define SNMX_OFFSET SUPPRESS_NONMAX_SIZE/2

#define CULL_SIZE 7
#define CULL_OFFSET CULL_SIZE/2




#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL, pDupGaussian2:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT], pDupGaussian2[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pGaussian1:SEQUENTIAL, pGaussian2:SEQUENTIAL,  pGaussian3:SEQUENTIAL)
#pragma SDS data copy(pGaussian1[0:WIDTH*HEIGHT], pGaussian2[0:WIDTH*HEIGHT], pGaussian3[0:WIDTH*HEIGHT])

#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)
#pragma SDS data data_mover(pGaussian1:AXIDMA_SIMPLE, pGaussian2:AXIDMA_SIMPLE, pGaussian3:AXIDMA_SIMPLE, pDupGaussian2:AXIDMA_SIMPLE)

void sift_detect(	ap_uint<8>* pSrcImage,
					ap_uint<8>* pGaussian1,
					ap_uint<8>* pGaussian2,
					ap_uint<8>* pGaussian3,
					ap_uint<8>* pDupGaussian2
						);

