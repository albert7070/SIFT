
#include "HW_SIFT.hpp"
#include "DetectKeypoints.hpp"


void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								Grad_t *pGradImage,
								Keypoint_t *pKeypoints,
								int contrastTh
								)
{



	sift_detect(pSrcImageCam0, pGradImage, pKeypoints, contrastTh);


}





