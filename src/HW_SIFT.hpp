#pragma once


#include "SIFTParameters.hpp"





void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								Grad_t *pGradImage,
								Keypoint_t *pKeypoint,
								int contrastTh
								);
