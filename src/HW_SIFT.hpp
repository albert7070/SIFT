#pragma once


#include "SIFTParameters.hpp"






void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								ap_uint<8> *pKeypointImage,
								Fixed_t *pEigenImage
								);
