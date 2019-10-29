#include "xf_headers.h"
#include"SIFTParameters.hpp"
#include "HW_SIFT.hpp"
#include "string.h"



bool CompareContrast(Keypoint_t lhs, Keypoint_t rhs)
{
	return lhs.contrast > rhs.contrast;
};



int main(int argc, char** argv)
{

	ap_uint<8> *pSrcImageCam0 	= (ap_uint<8>*)sds_alloc(HEIGHT*WIDTH*sizeof(ap_uint<8>) ) ;

//	ap_int<16>  *pDOGImage = (ap_int<16>*)sds_alloc(HEIGHT*WIDTH*sizeof(ap_int<16>) ) ;

//	Grad_t 	*pGradientImage = (Grad_t*)sds_alloc(HEIGHT*WIDTH*sizeof(Grad_t) ) ;

	Keypoint_t 	*pKeypoints = (Keypoint_t*)sds_alloc(MAX_KEYPOINTS*sizeof(Keypoint_t) ) ;

	Keypoint_t 	*pFinalKeypoints = (Keypoint_t*)sds_alloc(MAX_FINAL_KEYPOINTS*sizeof(Keypoint_t) ) ;

//	ap_uint<8>  *pMainOrientation = (ap_uint<8>*)sds_alloc(MAX_KEYPOINTS*sizeof(ap_uint<8>) ) ;

//	Fixed_t  *pEigenImage = (Fixed_t*)sds_alloc(HEIGHT*WIDTH*sizeof(Fixed_t) ) ;

	Desc_t* pDesc = (Desc_t*)sds_alloc(DESC_SIZE*MAX_KEYPOINTS*sizeof(Desc_t));

int ImgID = 0 ;
int nTotalImages = 40;

	while(ImgID<nTotalImages)
	{


		std::string strImageID = std::to_string(ImgID)	;

		std::string strImagePathCam0 = "Data/Bot/";

//		std::string strImagePathCam0 = "Meas2/";
		std::string strFullImagePathCam0 = strImagePathCam0 + "bot_" + strImageID + ".bmp";					// full path of image from cam0


//		std::string strFullImagePathCam0 = "Top_60.bmp";
		printf("\n\nReading image %s \n", strFullImagePathCam0.c_str());
		cv::Mat mSrcImage0 = cv::imread(strFullImagePathCam0, CV_LOAD_IMAGE_GRAYSCALE);					// Read cam0 and cam1 images from file using opencv



		for(int i = 0 ; i < WIDTH*HEIGHT; i++)															// cv::Mat to arrays so that we can feed them to hardware function
		{
			pSrcImageCam0[i] = *(mSrcImage0.data+i);
		}


		printf("Detecting Keypoints in Hardware ...\n");

		unsigned long clock_start, clock_end;															// counter to profile the hardware function
		clock_start = sds_clock_counter();


		// SIFT DETECT AND COMPUTE //

//		sift_detect_and_compute(pSrcImageCam0, pKeypointImage, pEigenImage);

	//	sift_detect_and_compute(pSrcImageCam0, pGradientImage, pKeypoints, 20);

		sift_detect_and_compute(pSrcImageCam0, pKeypoints,pDesc, 15);

		clock_end = sds_clock_counter();

		printf("Detect and Compute Time = %f ms \n", (1000.0/sds_clock_frequency())*(double)(clock_end-clock_start));
		printf("Done Processing  .. \n");


// SORT keypoints
		std::sort(pKeypoints, pKeypoints + MAX_KEYPOINTS, CompareContrast);

		memcpy(pFinalKeypoints, pKeypoints, MAX_FINAL_KEYPOINTS*sizeof(Keypoint_t) ) ;

		printf("Writing results to file .. \n");

		cv::Mat mDOGImage, mAngle, mMagn, mKeypointImage ;

		std::vector<cv::KeyPoint> vKeypointsOut;

		mAngle.create(HEIGHT, WIDTH, CV_32SC1);
		mMagn.create(HEIGHT, WIDTH, CV_32SC1);

//		mDOGImage.create(HEIGHT, WIDTH, CV_32SC1);
		mKeypointImage.create(HEIGHT, WIDTH, CV_32SC1);

//		for(int iRow =  0; iRow < HEIGHT ; iRow++)
//		{
//			for(int iCol = 0 ; iCol < WIDTH; iCol++ )
//			{
//
////				mKeypointImage.at<uchar>(iRow, iCol) = pKeypointImage[iRow*WIDTH + iCol];
//
//				//mDOGImage.at<int>(iRow, iCol) = (int)pDOGImage[iRow*WIDTH + iCol];
////				mAngle.at<int>(iRow, iCol) = (int)pGradientImage[iRow*WIDTH + iCol].angGrad;
////				mMagn.at<int>(iRow, iCol) = (int)pGradientImage[iRow*WIDTH+ iCol].magGrad;
//			}
//		}


		int nKp =0 ;
		for(int i = 0 ; i< MAX_FINAL_KEYPOINTS; i++)
		{
			cv::KeyPoint kp0;

			kp0.pt.x = (int)pFinalKeypoints[i].x;
			kp0.pt.y = (int)pFinalKeypoints[i].y;
			kp0.response = (int)pFinalKeypoints[i].contrast;

	//		kp0.angle = pMainOrientation[i];

			vKeypointsOut.push_back(kp0);

			if (kp0.pt.x > 0 && kp0.pt.y > 0)
				nKp++;

			mKeypointImage.at<int>(kp0.pt.y, kp0.pt.x) = (int)kp0.response;
		}
		printf("/nnumber of keypoints = %d\n", nKp	);



		cv::Mat  mDesc;
		mDesc.create(MAX_KEYPOINTS, DESC_SIZE, CV_32S);


	// Save Descriptors to File
		int id = 0 ;
		for(int i = 0; i < MAX_KEYPOINTS; i++)
		{
			for(int j = 0 ; j < DESC_SIZE ; j++)
			{

				mDesc.at<int>(i,j)= (int)pDesc[ id] ;
			id++;
			}
		}




		cv::FileStorage fs;

		std::string strKeypointPath = "SIFT_Test/FinalKeypoints/bot_Keypoints_";
		std::string strKeypointFullPath = strKeypointPath + strImageID + ".yml";

		std::string strKeypointImageFullPath = strKeypointPath + strImageID + ".bmp";

//		fs.open(strKeypointFullPath, cv::FileStorage::WRITE);
//		write(fs, "keypoints", vKeypointsOut);

		cv::imwrite(strKeypointImageFullPath, mKeypointImage);

		std::string strDescPath = "SIFT_Test/Descriptors/bot_Desc_";
		std::string strDesc = strDescPath + strImageID + ".yml";

		fs.open(strDesc,  cv::FileStorage::WRITE);
		fs<<"Descriptor" << mDesc;


//		std::string strDOGPath = "SIFT_Test/DOG2/bot_DOG2_";
//		std::string strDOG = strDOGPath + strImageID + ".yml";

//		fs.open(strDOG,  cv::FileStorage::WRITE);
//		fs<<"DOG2" << mDOGImage;

		fs.release();


		std::string strExtremaPath = "SIFT_Test/FilteredKeypoints_/Bot_Keypoints_";
		std::string strExtrema = strExtremaPath + strImageID + ".bmp";
//		std::string strExtrema = "Top_60_Keypoint.bmp";
		cv::imwrite(strExtrema, mKeypointImage);


//		cv::FileStorage fs;
//
//		std::string strAnglePath = "SIFT_Test/Gradients/bot_Angle_";
//		std::string strAngle = strAnglePath + strImageID + ".yml";
//
//		fs.open(strAngle,  cv::FileStorage::WRITE);
//		fs<<"Angle" << mAngle;
//
//
//
//		std::string strMagPath = "SIFT_Test/Gradients/bot_Mag_";
//		std::string strMag = strMagPath + strImageID + ".yml";
//
//		fs.open(strMag,  cv::FileStorage::WRITE);
//		fs<<"Mag" << mMagn;

//		fs.release();





		ImgID++;
	}



	sds_free(pSrcImageCam0) ;
	sds_free(pKeypoints) ;
	sds_free(pFinalKeypoints) ;
	sds_free(pDesc);
//	sds_free(pGradientImage) ;
//	sds_free(pKeypointImage);

//	sds_free(pDOGImage);
return 0;

}
























// =============================================================================================================//



// ========= DEBUG FUNCTIONS=========== //



////	 Write Gaussians to file:
//	cv::Mat mGaussian1, mGaussian2, mGaussian3;
//	mGaussian1.create(1080, 1920, CV_8UC1);
//	mGaussian2.create(1080, 1920, CV_8UC1);
//	mGaussian3.create(1080, 1920, CV_8UC1);
//	for(int iRow =  0; iRow < HEIGHT ; iRow++)
//	{
//		for(int iCol = 0 ; iCol < WIDTH; iCol++ )
//		{
//			mGaussian1.at<uchar>(iRow, iCol) = pGaussian1[iRow*WIDTH + iCol];
//			mGaussian2.at<uchar>(iRow, iCol) = pGaussian2[iRow*WIDTH + iCol];
//			mGaussian3.at<uchar>(iRow, iCol) = pGaussian3[iRow*WIDTH + iCol];
//		}
//	}
//
//	cv::imwrite("Gausian1.bmp", mGaussian1);
//	cv::imwrite("Gausian2.bmp", mGaussian2);
//	cv::imwrite("Gausian3.bmp", mGaussian3);
//
//
//
//
//
////	 Write Difference of Gaussians to file:
//	cv::Mat  mDOG1, mDOG2, mDOG3;
//	mDOG1.create(1080, 1920, CV_32S);
//	mDOG2.create(1080, 1920, CV_32S);
//	mDOG3.create(1080, 1920, CV_32S);
//
//	for(int iRow =  0; iRow < HEIGHT ; iRow++)
//	{
//		for(int iCol = 0 ; iCol < WIDTH; iCol++ )
//		{
//			mDOG1.at<int>(iRow, iCol) =  (int)(pDOG1[iRow*WIDTH + iCol]);
//			mDOG2.at<int>(iRow, iCol) =  (int)(pDOG2[iRow*WIDTH + iCol]);
//			mDOG3.at<int>(iRow, iCol) =  (int)(pDOG3[iRow*WIDTH + iCol]);
//		}
//	}
//
//
//	fs.open("DOG1.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG1;
//
//	fs.open("DOG2.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG2;
//
//	fs.open("DOG3.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG3;
//
//
////	Write Keypoint Images to file
//		cv::Mat mKeypointImage1;
//		mKeypointImage1.create(HEIGHT, WIDTH, CV_8U );
//
//		cv::Mat mKeypointImage2;
//		mKeypointImage2.create(HEIGHT, WIDTH, CV_8U );
//
//		cv::Mat mFinalKeypointImage;
//		mFinalKeypointImage.create(HEIGHT, WIDTH, CV_8U );
//
//		for(int iRow =  0; iRow < HEIGHT ; iRow++)
//		{
//			for(int iCol = 0 ; iCol < WIDTH; iCol++ )
//			{
//				mKeypointImage1.at<uchar>(iRow, iCol) = pKeypointImage1[iRow*WIDTH + iCol];
//				mKeypointImage2.at<uchar>(iRow, iCol) = pKeypointImage2[iRow*WIDTH + iCol];
//
//			}
//		}
//
//		cv::imwrite("SIFTKeypointImage1.bmp", mKeypointImage1);
//		cv::imwrite("SIFTKeypointImage2.bmp", mKeypointImage2);
//

//// Print Descriptor
//	int id = 0 ;
//	for(int i = 0; i < MAX_KEYPOINTS; i++)
//	{
//		printf("\n\nDescriptor keypoint_%d: ", i);
//		for(int j = 0 ; j < NCIRCLES; j++)
//		{
//			printf("\nSubregion_%d : ", j);
//
//			for(int k = 0 ; k < 8; k++)
//			{
//				printf("%d, ", (int)pPrevDesc[id] );
//				id++;
//			}
//
//		}
//	}




//	// Save Descriptors to File
//		int id = 0 ;
//		for(int i = 0; i < MAX_KEYPOINTS; i++)
//		{
//			for(int j = 0 ; j < DESC_SIZE ; j++)
//			{
//				mDesc.at<int>(i,j)= (int)pPrevDesc[id] ;
//			id++;
//			}
//		}
//
//
//		cv::FileStorage fs;
//		fs.open("Descriptor.yml",  cv::FileStorage::WRITE);
//		fs<<"Descriptor" << mDesc;


//std::vector<cv::KeyPoint> vKeypointsOutCam0, vKeypointsOutCam1;
//
//		for(int i = 0 ; i< MAX_KEYPOINTS; i++)
//		{
//			cv::KeyPoint kp0, kp1 ;
//
//			kp0.pt.x = (int)pCurrKeypointsCam0[i].x;
//			kp0.pt.y = (int)pCurrKeypointsCam0[i].y;
//
//			kp1.pt.x = (int)pCurrKeypointsCam1[i].x;
//			kp1.pt.y = (int)pCurrKeypointsCam1[i].y;
//
//			vKeypointsOutCam0.push_back(kp0);
//			vKeypointsOutCam1.push_back(kp1);
//		}
//
//
//		std::string strOutKeypointPathCam0 = "keypoints_Meas2/bot_Keypoints"   + strImageID + ".yml";
//		std::string strOutKeypointPathCam1 = "keypoints_Meas2/top_Keypoints"   + strImageID + ".yml";
//
//
//		fs.open(strOutKeypointPathCam0, cv::FileStorage::WRITE);
//		write(fs, "keypoints", vKeypointsOutCam0);
//



