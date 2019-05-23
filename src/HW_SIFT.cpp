
#include "HW_SIFT.hpp"
#include "GaussianBlur.hpp"
#include "DetectKeypoints.hpp"


void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0, ap_uint<8> *pGaussian1, ap_uint<8> *pGaussian2, ap_uint<8> *pGaussian3, ap_uint<8> *pDupGaussian2)
{





#pragma SDS async(1)
	sift_detect(pSrcImageCam0, pGaussian1, pGaussian2, pGaussian3, pDupGaussian2);



	#pragma SDS wait(1)



}





