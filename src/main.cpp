#include "xf_headers.h"
#include"SIFTParameters.hpp"
#include "HW_SIFT.hpp"
#include "string.h"




int main(int argc, char** argv)
{

	ap_uint<8> *pSrcImage 		= (ap_uint<8>*)sds_alloc(HEIGHT*WIDTH*sizeof(ap_uint<8>) ) ;

	ap_uint<8>  *pKeypointImage = (ap_uint<8>*)sds_alloc(HEIGHT*WIDTH*sizeof(ap_uint<8>) ) ;

	Fixed_t 	*pEigenImage = (Fixed_t*)sds_alloc(HEIGHT*WIDTH*sizeof(Fixed_t) ) ;

	printf("Entering main().. \n");

	cv::Mat mSrcImage = cv::imread("wheat.bmp", CV_LOAD_IMAGE_GRAYSCALE);

	for(int i = 0 ; i < WIDTH*HEIGHT; i++)
	{
		pSrcImage[i] = *(mSrcImage.data+i);

	}

	printf("Test Image Read.. \n");

	unsigned long clock_start, clock_end;
	clock_start = sds_clock_counter();

	sift_detect_and_compute(pSrcImage, pKeypointImage, pEigenImage);

	clock_end = sds_clock_counter();

	printf("Eigens and Extremas Computed in %f ms \n", (1000.0/sds_clock_frequency())*(double)(clock_end-clock_start));

		cv::Mat mKeypointImage, mEigenImage ;

		mEigenImage.create(HEIGHT, WIDTH, CV_32F);
		mKeypointImage.create(HEIGHT, WIDTH, CV_8UC1);

		for(int iRow =  0; iRow < HEIGHT ; iRow++)
		{
			for(int iCol = 0 ; iCol < WIDTH; iCol++ )
			{

				mKeypointImage.at<uchar>(iRow, iCol) = pKeypointImage[iRow*WIDTH + iCol];
				mEigenImage.at<float>(iRow, iCol) = (float)pEigenImage[iRow*WIDTH + iCol];
			}
		}


		cv::imwrite("KeypointImageHWSIFT_Tocompare.bmp", mKeypointImage);


		cv::FileStorage fs;

		fs.open("EigenImage_Tocompare.yml",  cv::FileStorage::WRITE);
		fs<<"Eigen" << mEigenImage;

		fs.release();

		sds_free(pSrcImage);

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




