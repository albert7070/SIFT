#pragma once


#include "SIFTParameters.hpp"






void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								Keypoint_t *pKeypoints,
								Desc_t *pDesc,
								int contrastTh
								);
