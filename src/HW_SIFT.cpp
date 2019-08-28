
#include "HW_SIFT.hpp"
#include "GaussianBlur.hpp"
#include "DetectKeypoints.hpp"


void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								ap_uint<8> *pKeypointImage,
								Fixed_t *pEigenImage
								)
{


	sift_detect(pSrcImageCam0, pKeypointImage, pEigenImage);


}





