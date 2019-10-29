
#include "HW_SIFT.hpp"
#include "DetectKeypoints.hpp"
#include "SIFT_Descriptor.hpp"



//
//bool CompareContrast(Keypoint_t lhs, Keypoint_t rhs)
//{
//	return lhs.contrast > rhs.contrast;
//};



void sift_detect_and_compute(	ap_uint<8> *pSrcImageCam0,
								Keypoint_t *pKeypoints,
								Desc_t *pDesc,
								int contrastTh
								)
{

	Grad_t* pGradient = (Grad_t*)sds_alloc(HEIGHT*WIDTH*sizeof(Grad_t));
	ap_uint<8>* pMainOrientation = (ap_uint<8>*)sds_alloc_non_cacheable(MAX_KEYPOINTS*sizeof(ap_uint<8>)) ;

//	Keypoint_t* pPreKeypointsCam0 = (Keypoint_t*)sds_alloc(MAX_KEYPOINTS*sizeof(Keypoint_t)) ;


	sift_detect(pSrcImageCam0, pGradient, pKeypoints, contrastTh);

//
//	std::sort(pPreKeypointsCam0, pPreKeypointsCam0 + MAX_KEYPOINTS, CompareContrast);
//
//
//
//	memcpy(pKeypoints, pPreKeypointsCam0, MAX_FINAL_KEYPOINTS*sizeof(Keypoint_t) ) ;



	compute_main_orientation(pGradient, pKeypoints,pMainOrientation);
	compute_descriptor(pGradient, pKeypoints, pMainOrientation, pDesc);


	sds_free(pGradient);
	sds_free(pMainOrientation);
//	sds_free(pPreKeypointsCam0);



}





