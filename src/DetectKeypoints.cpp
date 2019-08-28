#include "xf_headers.h"
#include "DetectKeypoints.hpp"

#include "GaussianBlur.hpp"


//#include "xf_headers.h"



ap_int<16> ABS(ap_int<16> a)
{

#pragma HLS INLINE

	if(a<0)
		return -1*(a);
	else
		return a; 		// never reaches here

}


// ===================================== Wrapper for SDSoC ==================================== //


#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL,  pKeypointOut:SEQUENTIAL, pEigenValuesOut:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT],  pKeypointOut[0:WIDTH*HEIGHT], pEigenValuesOut[0:WIDTH*HEIGHT])

#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE, pKeypointOut:AXIDMA_SIMPLE)



void sift_detect(	ap_uint<8>* pSrcImage,
					ap_uint<8>* pKeypointOut,
					Fixed_t* 	pEigenValuesOut

					)
{

#pragma HLS INLINE OFF
#pragma HLS DATAFLOW


	hls::stream< Pixel_t > srcImageStream;
	hls::stream< Pixel_t > DupSrcImageStream;
	hls::stream< Pixel_t > GaussianStream1;
	hls::stream< Pixel_t > GaussianStream2;
	hls::stream< Pixel_t > GaussianStream3;



	hls::stream< ap_int<16> > DOGStream1;
	hls::stream< ap_int<16> > DOGStream2;
	hls::stream< ap_int<16> > DOGStream3;

	hls::stream< ap_uint<8> > KeypointMapStream1;
	hls::stream< Fixed_t > pEigenRatioStream2;



// populate input image stream
	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Pixel_t PixelIn = pSrcImage[i];

		srcImageStream.write( PixelIn );

	}



	int contrastTh = 10;
// Calling HLS functions

	GaussianBlurHLS(srcImageStream, GaussianStream1, GaussianStream2, GaussianStream3,  DupSrcImageStream);



		cv::Mat mGaussian1;
		mGaussian1.create(HEIGHT, WIDTH, CV_8UC1);
		for(int iRow =  0; iRow < HEIGHT ; iRow++)
		{
			for(int iCol = 0 ; iCol < WIDTH; iCol++ )
			{
				mGaussian1.at<uchar>(iRow, iCol) = GaussianStream1.read();
			}
		}

		cv::imwrite("Gausian1_Arm.bmp", mGaussian1);

	build_DOG_hls(DupSrcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DOGStream1, DOGStream2, DOGStream3);

	detect_extrema_hls(DOGStream1, DOGStream2, DOGStream3, KeypointMapStream1, pEigenRatioStream2, contrastTh);

	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE


		ap_uint<8> kpOut = KeypointMapStream1.read();

		Fixed_t EigOut = pEigenRatioStream2.read();

		pKeypointOut[i] = kpOut;
		pEigenValuesOut[i] = EigOut;

	}


}








void build_DOG_hls(	hls::stream< Pixel_t > &pSrcImage,
					hls::stream< Pixel_t > &pGaussian1,
					hls::stream< Pixel_t > &pGaussian2,
					hls::stream< Pixel_t > &pGaussian3,
					hls::stream< ap_int<16> > &pDOG1,
					hls::stream< ap_int<16> > &pDOG2,
					hls::stream< ap_int<16> > &pDOG3)
{




	Pixel_t value1, value2, value3, value4;

	ap_int<16> Diff1, Diff2, Diff3;

//	cv::Mat mDOG1, mDOG2, mDOG3 ;
//
//	mDOG1.create(HEIGHT, WIDTH, CV_32S);
//	mDOG2.create(HEIGHT, WIDTH, CV_32S);
//	mDOG3.create(HEIGHT, WIDTH, CV_32S);


	for(int iRow = 0 ; iRow < HEIGHT; iRow++)
	{

		for(int iCol = 0; iCol < WIDTH; iCol++)
		{

#pragma HLS PIPELINE II = 1

			value1 = pSrcImage.read();				// Incoming pixels are read into 16 bit ints to avoid data overflows.
			value2 = pGaussian1.read();
			value3 = pGaussian2.read();
			value4 = pGaussian3.read();




			Diff1 = value2 - value1 ;
			Diff2 = value3 - value2 ;
			Diff3 = value4 - value3 ;


//			mDOG1.at<int>(iRow, iCol) =  (int)Diff1;
//			mDOG2.at<int>(iRow, iCol) =  (int)Diff2;
//			mDOG3.at<int>(iRow, iCol) =  (int)Diff3;

			pDOG1.write(Diff1);
			pDOG2.write(Diff2);
			pDOG3.write(Diff3);


		}

	}



//	cv::FileStorage fs;
//
//	fs.open("DOG1_ARM.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG1;
//
//	fs.open("DOG2_ARM.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG2;
//
//	fs.open("DOG3_ARM.yml",  cv::FileStorage::WRITE);
//	fs<<"DOG" << mDOG3;

}





void detect_extrema_hls(	hls::stream< ap_int<16> > &pDOG1,
							hls::stream< ap_int<16> > &pDOG2,
							hls::stream< ap_int<16> > &pDOG3,
							hls::stream< ap_uint<8> > &pKeypointMap1,
							hls::stream< Fixed_t > &pEigenRatio,
							int contrastTh
						)
{


	ap_int<16> WindowSrc[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];				// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0

	ap_int<16> SrcImageBuf1[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf1 complete dim=1

	ap_int<16> SrcImageBuf2[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf2 complete dim=1

	ap_int<16> SrcImageBuf3[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf3 complete dim=1

	ap_int<16> SrcRightCol1[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol1 dim=0

	ap_int<16> SrcRightCol2[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol2 dim=0

	ap_int<16> SrcRightCol3[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol3 dim=0



	// The following buffers are used to create contrast and edge detection window

	ap_int<16> Window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];							// Window to do contrast and edge detection on
#pragma HLS ARRAY_PARTITION variable=Window complete dim=0

	ap_int<16> SrcImageBuf[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	ap_int<16> SrcRightCol[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0




int ContrastThreshold = contrastTh;


int nKeypointIdx =  0;

L1:	for (int iRow = 0 ; iRow < HEIGHT + SNMX_OFFSET; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + SNMX_OFFSET; iCol++ )
		{
	#pragma HLS PIPELINE

			ap_int<16> PixelIn1;
			ap_int<16> PixelIn2;
			ap_int<16> PixelIn3;


			if(iRow<HEIGHT && iCol < WIDTH)
			{
				PixelIn1 = pDOG1.read();
				PixelIn2 = pDOG2.read();
				PixelIn3 = pDOG3.read();
			}
			else
			{
				PixelIn1 = 0 ;
				PixelIn2 = 0 ;
				PixelIn3 = 0 ;
			}


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < SUPPRESS_NONMAX_SIZE - 1; ii++)
				{
					SrcRightCol1[ii] = SrcImageBuf1[ii][iCol]=SrcImageBuf1[ii+1][iCol];
					SrcRightCol2[ii] = SrcImageBuf2[ii][iCol]=SrcImageBuf2[ii+1][iCol];
					SrcRightCol3[ii] = SrcImageBuf3[ii][iCol]=SrcImageBuf3[ii+1][iCol];

					SrcRightCol[ii] = SrcImageBuf[ii][iCol] = SrcImageBuf[ii+1][iCol];
				}
				SrcRightCol1[SUPPRESS_NONMAX_SIZE-1] = SrcImageBuf1[SUPPRESS_NONMAX_SIZE-1][iCol] = PixelIn1;
				SrcRightCol2[SUPPRESS_NONMAX_SIZE-1] = SrcImageBuf2[SUPPRESS_NONMAX_SIZE-1][iCol] = PixelIn2;
				SrcRightCol3[SUPPRESS_NONMAX_SIZE-1] = SrcImageBuf3[SUPPRESS_NONMAX_SIZE-1][iCol] = PixelIn3;

				SrcRightCol[SUPPRESS_NONMAX_SIZE-1] = SrcImageBuf[SUPPRESS_NONMAX_SIZE-1][iCol] = PixelIn2;
			}
			else
			{
				for(unsigned char ii = 0; ii < SUPPRESS_NONMAX_SIZE; ii++)
				{
					SrcRightCol1[ii] = 0 ;
					SrcRightCol2[ii] = 0 ;
					SrcRightCol3[ii] = 0 ;

					SrcRightCol[ii] = 0 ;
				}
			}


			//3D sliding window:
			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < SUPPRESS_NONMAX_SIZE; ii++)
			{
				for(unsigned char jj = 0; jj < SUPPRESS_NONMAX_SIZE-1; jj++)
				{
					WindowSrc[0][ii][jj] = WindowSrc[0][ii][jj+1];					// scale 1
					WindowSrc[1][ii][jj] = WindowSrc[1][ii][jj+1];					// scale 2
					WindowSrc[2][ii][jj] = WindowSrc[2][ii][jj+1];					// scale 3

					Window[ii][jj] = Window[ii][jj+1];								// This window is for contrast and edge rejection
				}
			}

			for(unsigned char ii = 0; ii < SUPPRESS_NONMAX_SIZE; ii++)
			{
				WindowSrc[0][ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol1[ii];
				WindowSrc[1][ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol2[ii];
				WindowSrc[2][ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol3[ii];

				Window[ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol[ii];

			}


			// operate on the window and output the result on the output stream

			if ( (iRow>=SNMX_OFFSET) && (iCol>=SNMX_OFFSET)  )
			{

				ap_uint<8> kpOut1;
				ap_uint<8> kpOut2;


				bool bKeypointDetected = 0 ;


				bKeypointDetected = IsLocalExtremum(WindowSrc);			// check if center of the window is maximum


				ap_int<16> centerPixel = Window[SNMX_OFFSET][SNMX_OFFSET];
				bool bIsLowContrastPoint = (ABS(centerPixel) < ContrastThreshold );	// check if the center pixel is a low contrast point



				Fixed_t bKeypointEigenRatio = IsOnEdge(Window);

//				printf("%f \n", (float)bKeypointEigenRatio );

				if( bKeypointDetected )											// If a valid keypoint is detected, output it to the keypoint stream
					kpOut1 = 1;
				else
					kpOut1 = 0;


//				if( bKeypointEigenRatio == -1 || bIsLowContrastPoint)			// If keypoint is on edge or a low contrast keypoint then suppress it
//					kpOut2 = 0;
//				else
					kpOut2 = 1;


				pKeypointMap1.write(kpOut1);  // &&kpOut2);							// Combine keypoints to give final keypoint output

				pEigenRatio.write(bKeypointEigenRatio);

				nKeypointIdx++ ;

			}


		}//end of L2
	}//end of L1



}





Fixed_t IsOnEdge(ap_int<16> window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE])
{

	int rx =  SNMX_OFFSET;
	int ry =  SNMX_OFFSET ;


	// be careful with equations !!
	int fxx = window[ry][rx-1] + window[ry][rx+1] - 2*window[ry][rx];
	int fyy = window[ry-1][rx] + window[ry+1][rx] - 2*window[ry][rx];
	int fxy = window[ry-1][rx-1] + window[ry+1][rx+1] - window[ry+1][rx-1] - window[ry-1][rx+1];

	int trace = fxx + fyy;
	int deter = fxx*fyy - fxy*fxy ;

	//		fxx= dog{2}(rx-1,ry)+dog{2}(rx+1,ry)-2*dog{2}(rx,ry); 								// double derivate in x direction
	//	    fyy= dog{2}(rx,ry-1)+dog{2}(rx,ry+1)-2*dog{2}(rx,ry); 								// double derivate in y direction
	//	    fxy= dog{2}(rx-1,ry-1)+dog{2}(rx+1,ry+1)-dog{2}(rx-1,ry+1)-dog{2}(rx+1,ry-1); 		// derivate in x and y direction
	//	    trace=fxx+fyy;
	// 	    deter=fxx*fyy-fxy*fxy;

	float eigen_ratio = trace*trace/deter;

//	Fixed_t eigen_ratio = trace*trace/deter;

//	printf("%f \n",eigen_ratio );

	Fixed_t eigenRatioOut ;

	if( eigen_ratio < 0 || eigen_ratio > EIGEN_RATIO_TH)
		eigenRatioOut = -1;
	else
		eigenRatioOut = eigen_ratio ;

	return eigenRatioOut;


}




bool IsLocalExtremum(ap_int<16> window[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE])
{


	ap_int<16> centerPixel = window[N_SCALES/2][SUPPRESS_NONMAX_SIZE/2][SUPPRESS_NONMAX_SIZE/2]; 		//	center pixel that needs to be compared to the neighbours


bool local_maxima;
bool local_minima;
bool extrema;

	local_maxima =	(centerPixel>window[1][0][0])&& 	\
					(centerPixel>window[1][0][1])&& 	\
					(centerPixel>window[1][0][2])&& 	\
					(centerPixel>window[1][1][0])&& 	\
					(centerPixel>window[1][1][2])&& 	\
					(centerPixel>window[1][2][0])&& 	\
					(centerPixel>window[1][2][1])&&		\
					(centerPixel>window[1][2][2]) ;
	local_maxima=local_maxima && \
			(centerPixel>window[2][0][0])&& 	\
			(centerPixel>window[2][0][1])&& 	\
			(centerPixel>window[2][0][2])&& 	\
			(centerPixel>window[2][1][0])&& 	\
			(centerPixel>window[2][1][1])&& 	\
			(centerPixel>window[2][1][2])&& 	\
			(centerPixel>window[2][2][0])&& 	\
			(centerPixel>window[2][2][1])&&		\
			(centerPixel>window[2][2][2]) ;
	local_maxima=local_maxima && \
			(centerPixel>window[0][0][0])&& 	\
			(centerPixel>window[0][0][1])&& 	\
			(centerPixel>window[0][0][2])&& 	\
			(centerPixel>window[0][1][0])&& 	\
			(centerPixel>window[0][1][1])&& 	\
			(centerPixel>window[0][1][2])&& 	\
			(centerPixel>window[0][2][0])&& 	\
			(centerPixel>window[0][2][1])&&		\
			(centerPixel>window[0][2][2]) ;

	// look for a local minima
	local_minima =	(centerPixel<window[1][0][0])&& 	\
					(centerPixel<window[1][0][1])&& 	\
					(centerPixel<window[1][0][2])&& 	\
					(centerPixel<window[1][1][0])&& 	\
					(centerPixel<window[1][1][2])&& 	\
					(centerPixel<window[1][2][0])&& 	\
					(centerPixel<window[1][2][1])&&		\
					(centerPixel<window[1][2][2]) ;
	local_minima=local_minima && \
			(centerPixel<window[2][0][0])&& 	\
			(centerPixel<window[2][0][1])&& 	\
			(centerPixel<window[2][0][2])&& 	\
			(centerPixel<window[2][1][0])&& 	\
			(centerPixel<window[2][1][1])&& 	\
			(centerPixel<window[2][1][2])&& 	\
			(centerPixel<window[2][2][0])&& 	\
			(centerPixel<window[2][2][1])&&		\
			(centerPixel<window[2][2][2]) ;
	local_minima=local_minima && \
			(centerPixel<window[0][0][0])&& 	\
			(centerPixel<window[0][0][1])&& 	\
			(centerPixel<window[0][0][2])&& 	\
			(centerPixel<window[0][1][0])&& 	\
			(centerPixel<window[0][1][1])&& 	\
			(centerPixel<window[0][1][2])&& 	\
			(centerPixel<window[0][2][0])&& 	\
			(centerPixel<window[0][2][1])&&		\
			(centerPixel<window[0][2][2]) ;

	extrema=local_maxima || local_minima;




//printf("\n Window: ");
//	for(int i = 0 ; i < 3 ; i++)
//	{
//		printf("\n");
//		for(int j = 0; j< 3 ; j++ )
//		{
//
//			printf("%d, ", (int )window[1][i][j]);
//
//		}
//	}

return extrema ;

}





