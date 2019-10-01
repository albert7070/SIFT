



#include "DetectKeypoints.hpp"



















//================================================== HLS FUNCTIONS =========================================================================//

// This function operates on the DOG pyramid. It takes in the three DOGs and does an extrema detection in a 3x3x3 neighbourhood
// After extracting the extrema it rejects keypoints that have low contrast or that are on an edge.
// Low contrast rejection is based on the contrastTh provided as an input
// Edge rejection is based on the eigenratio and eigenthreshold (constant set in the hpp)


/// <Summary>
/// Input Arguments:
/// 	- All three DOGs in the pyramid - DOG1, DOG2, DOG3
/// 	- contrast threshold
/// Returns:
/// 	- A Keypoint Image/Mask (dim. HeightxWidth) which has a zero for all non-keypoints and ABS(contrast value) i.e ABS(DOG2) for all pixels where a keypoint is detected
/// 	- An EigenRatio Image/MAsk, which has a value equal to the eigenRatio is its a non-edge point and -1 otherwise
/// </Summary>



void detect_extrema_hls(	hls::stream< ap_int<16> > &pDOG1,
							hls::stream< ap_int<16> > &pDOG2,
							hls::stream< ap_int<16> > &pDOG3,
							hls::stream< ap_uint<16> > &pKeypointMap1,
							hls::stream< Fixed_t > &pEigenRatio,
							int contrastTh
						)
{


	ap_int<16> WindowSrc[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];				// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0								// This is a 3x3x3 window sliding across the 3 DOGs.


	ap_int<16> SrcImageBuf1[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf1 complete dim=1

	ap_int<16> SrcImageBuf2[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf2 complete dim=1

	ap_int<16> SrcImageBuf3[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf3 complete dim=1

	ap_int<16> SrcRightCol1[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG1)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol1 dim=0

	ap_int<16> SrcRightCol2[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG2)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol2 dim=0

	ap_int<16> SrcRightCol3[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG3)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol3 dim=0



	// The following buffers are used to create contrast and edge detection window
	// This is a sliding window on the DOG2. So this must have data type same as DOG2

	ap_int<16> Window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];							// Window to do contrast and edge detection on
#pragma HLS ARRAY_PARTITION variable=Window complete dim=0

	ap_int<16> SrcImageBuf[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	ap_int<16> SrcRightCol[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0





int ContrastThreshold = contrastTh;




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

				Window[ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol[ii];				// This window is for contrast and edge rejection

			}


			// operate on the window and output the result on the output stream

			if ( (iRow>=SNMX_OFFSET) && (iCol>=SNMX_OFFSET)  )
			{


		//		printf("iRow, iCol = %d, %d\n", iRow-SNMX_OFFSET, iCol - SNMX_OFFSET);
				ap_uint<16> kpOut;


				ap_int<16> centerPixel = ABS(Window[SNMX_OFFSET][SNMX_OFFSET]);		// To reject keypoints we look at the absolute value

				bool bKeypointDetected = IsLocalExtremum(WindowSrc);				// Check if center of the window is maximum

				bool bIsLowContrastPoint = ((centerPixel) < ContrastThreshold );	// Check if the center pixel is a low contrast point

				Fixed_t bKeypointEigenRatio = IsOnEdge(Window);						// Check if the keypoint is on edge - based on eigen ratio


				if(!bKeypointDetected || (bKeypointEigenRatio == -1 || bIsLowContrastPoint)) 	// if keypoint is not detected OR it does not satisfy the eigen and contrast thresholds then its not a valid keypoint
					kpOut = 0 ;
				else
					kpOut = centerPixel;



				pKeypointMap1.write(kpOut);										// write ABS(contrast) of the keypoint to the output. So that we can sort the keypoints later

				pEigenRatio.write(bKeypointEigenRatio);							// Eigen Ratio is also written to the output





			}


		}//end of L2
	}//end of L1



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

return extrema ;

}





Fixed_t IsOnEdge(ap_int<16> window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE])
{

	int rx =  SNMX_OFFSET;
	int ry =  SNMX_OFFSET ;


	// TODO: check equations :
	int fxx = window[ry][rx-1] + window[ry][rx+1] - 2*window[ry][rx];
	int fyy = window[ry-1][rx] + window[ry+1][rx] - 2*window[ry][rx];
	int fxy = window[ry-1][rx-1] + window[ry+1][rx+1] - window[ry+1][rx-1] - window[ry-1][rx+1];

	int trace = fxx + fyy;
	int deter = fxx*fyy - fxy*fxy ;


	float eigen_ratio ;


	//printf("eigen ratio = %f, deter = %d \n\n", eigen_ratio, deter);

	if(deter == 0)
		eigen_ratio = -1;
	else
		eigen_ratio = float(trace*trace)/float(deter);

	Fixed_t eigenRatioOut ;

	if( eigen_ratio < 0 || eigen_ratio > EIGEN_RATIO_TH)
		eigenRatioOut = -1;
	else
		eigenRatioOut = eigen_ratio ;



	return eigenRatioOut;


}





// ============================================================ combine_keypoints_hls ===================================================================

// This function extracts an array of keypoints from the image of keypoints
/// <Summary>
/// Input Arguments:
/// 	- A full image of keypoints(dim. HEIGHTxWIDTH) - which is 0 for non keypoints and has ABS(contrast) for keypoints
/// Returns:
/// 	- An array of valid keypoints. Keypoint structure has x,y,contrast
///		- The max number of keypoints returned are always fixed = MAX_KEYPOINTS, so in cases when less than MAX_KEYPOINTS are found. a bunch of zeros are added to the array
/// </Summary>




void combine_keypoints_hls(	hls::stream< ap_uint<16> > &pKeypointImage,
							hls::stream< Keypoint_t > &pKeypoints )
{


	int nKeypoints = 0 ;
	int idx = 0;

	int nKeypointsTotal = 0 ;

	for(int iRow = 0 ; iRow < HEIGHT ; iRow++ )
	{
		for(int iCol = 0 ; iCol < WIDTH; iCol++)
		{
#pragma HLS PIPELINE II = 1

			ap_uint<16> value = pKeypointImage.read();


			ap_uint<16> valueOut = 0 ;

			if((iRow > WINDOW_SIZE && iCol > WINDOW_SIZE)&&(iRow< HEIGHT - WINDOW_SIZE && iCol < WIDTH - WINDOW_SIZE)  )	// this is to tackle borders.
			{
				if(value == 0  )
					valueOut = 0 ;
				else
					valueOut = value;

			}
			else
				valueOut = 0 ;



			if(valueOut > 0)
				nKeypointsTotal++;


			if(valueOut > 0  && nKeypoints<MAX_KEYPOINTS)
			{
				Keypoint_t KP_Out ;
				KP_Out.x = iCol;
				KP_Out.y = iRow;
				KP_Out.contrast = valueOut;

				pKeypoints.write(KP_Out);

				nKeypoints++;
			}

			idx++;
		}
	}

//	printf("total keypoint in the image = %d\n", nKeypointsTotal);


	for(int n = 0 ; n < MAX_KEYPOINTS-nKeypoints; n++)				// Make sure that the output keypoint stream is completely populated with total no. of elements = MAX_KEYPOINTS
	{																// Otherwise it causes a hang in hardware as the consumer of this stream is expecting MAX_KEYPOINTS
		Keypoint_t KP;
		KP.x = 0;
		KP.y = 0;
		KP.contrast = 0;

		pKeypoints.write( KP );
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



	for(int iRow = 0 ; iRow < HEIGHT; iRow++)
	{

		for(int iCol = 0; iCol < WIDTH; iCol++)
		{

#pragma HLS PIPELINE II = 1

			value1 = pSrcImage.read();
			value2 = pGaussian1.read();
			value3 = pGaussian2.read();
			value4 = pGaussian3.read();



			Diff1 = value2 - value1 ;
			Diff2 = value3 - value2 ;
			Diff3 = value4 - value3 ;



			pDOG1.write(Diff1);
			pDOG2.write(Diff2);
			pDOG3.write(Diff3);


		}

	}


}




// ============================================ Culling ======================================================= //


void culling_hls(	hls::stream< ap_uint<16> > &pKeypointMap1,
					hls::stream< Fixed_t > &pEigenRatio,
					hls::stream< ap_uint<16> > &pKeypointOut
						)
{


	Fixed_t WindowSrc[CULL_SIZE][CULL_SIZE];									// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0

	Fixed_t SrcImageBuf1[CULL_SIZE][WIDTH];										// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf1 complete dim=1

	Fixed_t SrcRightCol1[CULL_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol1 dim=0





	// The following buffers are used to create window on pKeypointMap1

	ap_uint<16> Window[CULL_SIZE][CULL_SIZE];
#pragma HLS ARRAY_PARTITION variable=Window complete dim=0

	ap_uint<16> SrcImageBuf[CULL_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	ap_uint<16> SrcRightCol[CULL_SIZE]; 										// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0




L1:	for (int iRow = 0 ; iRow < HEIGHT + CULL_OFFSET; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + CULL_OFFSET; iCol++ )
		{
	#pragma HLS PIPELINE

			Fixed_t EigenRatioIn1;												// Input value on pEigenRatio
			ap_uint<16> PixelIn2;												// Input value on pKeypointMap1


			if(iRow<HEIGHT && iCol < WIDTH)
			{
				EigenRatioIn1 = pEigenRatio.read();								// Read EigenRatio (its -1 for non keypoints, EigenRatio for keypoints)
				PixelIn2 = pKeypointMap1.read();								// Read Contrast value (its 0 for non keypoints, ABS(contrast) for keypoints)
			}
			else
			{
				EigenRatioIn1 = 0 ;												// This is to handle borders. Zero values are assumed outside image dimensions
				PixelIn2 = 0 ;
			}


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < CULL_SIZE - 1; ii++)
				{
					SrcRightCol1[ii] = SrcImageBuf1[ii][iCol]=SrcImageBuf1[ii+1][iCol];
					SrcRightCol[ii] = SrcImageBuf[ii][iCol] = SrcImageBuf[ii+1][iCol];
				}

				SrcRightCol1[CULL_SIZE-1] = SrcImageBuf1[CULL_SIZE-1][iCol] = EigenRatioIn1;

				SrcRightCol[CULL_SIZE-1] = SrcImageBuf[CULL_SIZE-1][iCol] = PixelIn2;
			}
			else
			{
				for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
				{
					SrcRightCol1[ii] = 0 ;

					SrcRightCol[ii] = 0 ;
				}
			}



			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
			{
				for(unsigned char jj = 0; jj < CULL_SIZE-1; jj++)
				{
					WindowSrc[ii][jj] = WindowSrc[ii][jj+1];
					Window[ii][jj] = Window[ii][jj+1];
				}
			}

			for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
			{
				WindowSrc[ii][CULL_SIZE-1] = SrcRightCol1[ii];
				Window[ii][CULL_SIZE-1] = SrcRightCol[ii];

			}


			// operate on the window and output the result on the output stream

			if ( (iRow>=CULL_OFFSET) && (iCol>=CULL_OFFSET)  )
			{


				bool bKeypointDetected ;

				ap_uint<16> centerPixel = Window[CULL_OFFSET][CULL_OFFSET];

				bKeypointDetected  = IsBestKeypoint(WindowSrc, Window);

				ap_uint<16> pixOut ;

				if (centerPixel >0 && bKeypointDetected > 0)								// if center pixel is more than zero it means this is a valid keypoint
					pixOut = centerPixel;													// now check if center of the window is maximum, if max. its a good keypoint
				else
					pixOut = 0 ;

				pKeypointOut.write(centerPixel);											// output will be 	- 0 if not a good keypoint
																							//				  	- ABS(contrast) if a good keypoint

			}


		}//end of L2
	}//end of L1

}


bool IsBestKeypoint(Fixed_t EigWindow[CULL_SIZE][CULL_SIZE], ap_uint<16> KPWindow[CULL_SIZE][CULL_SIZE])
{


	Fixed_t centerPixel = EigWindow[CULL_SIZE/2][CULL_SIZE/2]; 					//	center pixel that needs to be compared to the neighbours


	bool local_minima;

	bool extrema;


	local_minima = 1;


//printf("\n\nEigen Ratio Matrix: \n");
	for(int i = 0 ; i< CULL_SIZE; i++ )
	{
		for(int j = 0 ; j < CULL_SIZE; j++)
		{
//			printf("%f, ",(float) EigWindow[i][j]);
//			printf("%d,\t ",(int) KPWindow[i][j]);

			if(KPWindow[i][j] != 0 && (i!=CULL_OFFSET && j!=CULL_OFFSET ))
				local_minima = local_minima&&(centerPixel < EigWindow[i][j]);			// centerPixel here is the eigen ratio value for the keypoint
		}
//		printf("\n");
	}


	extrema = local_minima ;

return extrema ;



}

// ================================================================================= GAUSSIAN BLUR ============================================================== //


// 5x5 Gaussian Kernel
Mask_t GaussianKernel1[FILTER_SIZE1][FILTER_SIZE1] ={\
		{0.0002,    0.0033,    0.0081,    0.0033,    0.0002},	\
		{0.0033,    0.0479,    0.1164,    0.0479,    0.0033},	\
		{0.0081,    0.1164,    0.2831,    0.1164,    0.0081},	\
		{0.0033,    0.0479,    0.1164,    0.0479,    0.0033},	\
		{0.0002,    0.0033,    0.0081,    0.0033,    0.0002}	};



// 11x11 Gaussian Kernel, sigma = 2.25
Mask_t GaussianKernel2[FILTER_SIZE2][FILTER_SIZE2] ={\
	{0.000231624,0.000563409,0.001124803,0.001843074,0.002478693,0.002735999,0.002478693,0.001843074,0.001124803,0.000563409,0.000231624 },	\
	{0.000563409,0.001370449,0.002735999,0.004483141,0.006029235,0.006655114,0.006029235,0.004483141,0.002735999,0.001370449,0.000563409},	\
	{0.001124803,0.002735999,0.005462217,0.008950254,0.012036915,0.013286436,0.012036915,0.008950254,0.005462217,0.002735999,0.001124803},	\
	{0.001843074,0.004483141,0.008950254,0.014665665,0.019723393,0.021770826,0.019723393,0.014665665,0.008950254,0.004483141,0.001843074},	\
	{0.002478693,0.006029235,0.012036915,0.019723393,0.026525371,0.029278899,0.026525371,0.019723393,0.012036915,0.006029235,0.002478693},	\
	{0.002735999,0.006655114,0.013286436,0.021770826,0.029278899,0.032318264,0.029278899,0.021770826,0.013286436,0.006655114,0.002735999},	\
	{0.002478693,0.006029235,0.012036915,0.019723393,0.026525371,0.029278899,0.026525371,0.019723393,0.012036915,0.006029235,0.002478693},	\
	{0.001843074,0.004483141,0.008950254,0.014665665,0.019723393,0.021770826,0.019723393,0.014665665,0.008950254,0.004483141,0.001843074},	\
	{0.001124803,0.002735999,0.005462217,0.008950254,0.012036915,0.013286436,0.012036915,0.008950254,0.005462217,0.002735999,0.001124803},	\
	{0.000563409,0.001370449,0.002735999,0.004483141,0.006029235,0.006655114,0.006029235,0.004483141,0.002735999,0.001370449,0.000563409},	\
	{0.000231624,0.000563409,0.001124803,0.001843074,0.002478693,0.002735999,0.002478693,0.001843074,0.001124803,0.000563409,0.000231624}   };




Mask_t GaussianKernel3[FILTER_SIZE3][FILTER_SIZE3] ={\
	{0.000104598,0.000167455,0.000253646,0.000363508,0.000492897,0.000632345,0.000767552,0.000881491,0.00095782,0.000984704,0.00095782,0.000881491,0.000767552,0.000632345,0.000492897,0.000363508,0.000253646,0.000167455,0.000104598},	\
	{0.000167455,0.000268085,0.000406071,0.000581953,0.000789096,0.001012343,0.001228802,0.00141121,0.001533408,0.001576448,0.001533408,0.00141121,0.001228802,0.001012343,0.000789096,0.000581953,0.000406071,0.000268085,0.000167455},	\
	{0.000253646,0.000406071,0.000615081,0.000881491,0.001195253,0.001533408,0.00186128,0.002137576,0.002322671,0.002387864,0.002322671,0.002137576,0.00186128,0.001533408,0.001195253,0.000881491,0.000615081,0.000406071,0.000253646},	\
	{0.000363508,0.000581953,0.000881491,0.001263292,0.001712954,0.002197575,0.002667458,0.003063427,0.003328691,0.003422122,0.003328691,0.003063427,0.002667458,0.002197575,0.001712954,0.001263292,0.000881491,0.000581953,0.000363508},	\
	{0.000492897,0.000789096,0.001195253,0.001712954,0.002322671,0.002979789,0.003616925,0.004153837,0.004513521,0.004640208,0.004513521,0.004153837,0.003616925,0.002979789,0.002322671,0.001712954,0.001195253,0.000789096,0.000492897},	\
	{0.000632345,0.001012343,0.001533408,0.002197575,0.002979789,0.003822817,0.004640208,0.005329021,0.005790464,0.005952993,0.005790464,0.005329021,0.004640208,0.003822817,0.002979789,0.002197575,0.001533408,0.001012343,0.000632345},	\
	{0.000767552,0.001228802,0.00186128,0.002667458,0.003616925,0.004640208,0.005632373,0.006468467,0.007028576,0.007225856,0.007028576,0.006468467,0.005632373,0.004640208,0.003616925,0.002667458,0.00186128,0.001228802,0.000767552},	\
	{0.000881491,0.00141121,0.002137576,0.003063427,0.004153837,0.005329021,0.006468467,0.007428674,0.008071928,0.008298494,0.008071928,0.007428674,0.006468467,0.005329021,0.004153837,0.003063427,0.002137576,0.00141121,0.000881491},	\
	{0.00095782,0.001533408,0.002322671,0.003328691,0.004513521,0.005790464,0.007028576,0.008071928,0.008770882,0.009017066,0.008770882,0.008071928,0.007028576,0.005790464,0.004513521,0.003328691,0.002322671,0.001533408,0.00095782},	\
	{0.000984704,0.001576448,0.002387864,0.003422122,0.004640208,0.005952993,0.007225856,0.008298494,0.009017066,0.00927016,0.009017066,0.008298494,0.007225856,0.005952993,0.004640208,0.003422122,0.002387864,0.001576448,0.000984704},	\
	{0.00095782,0.001533408,0.002322671,0.003328691,0.004513521,0.005790464,0.007028576,0.008071928,0.008770882,0.009017066,0.008770882,0.008071928,0.007028576,0.005790464,0.004513521,0.003328691,0.002322671,0.001533408,0.00095782},	\
	{0.000881491,0.00141121,0.002137576,0.003063427,0.004153837,0.005329021,0.006468467,0.007428674,0.008071928,0.008298494,0.008071928,0.007428674,0.006468467,0.005329021,0.004153837,0.003063427,0.002137576,0.00141121,0.000881491},	\
	{0.000767552,0.001228802,0.00186128,0.002667458,0.003616925,0.004640208,0.005632373,0.006468467,0.007028576,0.007225856,0.007028576,0.006468467,0.005632373,0.004640208,0.003616925,0.002667458,0.00186128,0.001228802,0.000767552},	\
	{0.000632345,0.001012343,0.001533408,0.002197575,0.002979789,0.003822817,0.004640208,0.005329021,0.005790464,0.005952993,0.005790464,0.005329021,0.004640208,0.003822817,0.002979789,0.002197575,0.001533408,0.001012343,0.000632345},	\
	{0.000492897,0.000789096,0.001195253,0.001712954,0.002322671,0.002979789,0.003616925,0.004153837,0.004513521,0.004640208,0.004513521,0.004153837,0.003616925,0.002979789,0.002322671,0.001712954,0.001195253,0.000789096,0.000492897},	\
	{0.000363508,0.000581953,0.000881491,0.001263292,0.001712954,0.002197575,0.002667458,0.003063427,0.003328691,0.003422122,0.003328691,0.003063427,0.002667458,0.002197575,0.001712954,0.001263292,0.000881491,0.000581953,0.000363508},	\
	{0.000253646,0.000406071,0.000615081,0.000881491,0.001195253,0.001533408,0.00186128,0.002137576,0.002322671,0.002387864,0.002322671,0.002137576,0.00186128,0.001533408,0.001195253,0.000881491,0.000615081,0.000406071,0.000253646},	\
	{0.000167455,0.000268085,0.000406071,0.000581953,0.000789096,0.001012343,0.001228802,0.00141121,0.001533408,0.001576448,0.001533408,0.00141121,0.001228802,0.001012343,0.000789096,0.000581953,0.000406071,0.000268085,0.000167455},	\
	{0.000104598,0.000167455,0.000253646,0.000363508,0.000492897,0.000632345,0.000767552,0.000881491,0.00095782,0.000984704,0.00095782,0.000881491,0.000767552,0.000632345,0.000492897,0.000363508,0.000253646,0.000167455,0.000104598}		};








int RoundToInt(Fixed_t a)
{

	Fixed_t POINT_FIVE = 0.5;
#pragma HLS INLINE

	int ia;
	if(a>=0)
	{
		ia = (a + POINT_FIVE); 			// adding 0.5 to round it off to correct value. Assigning float to int here is equivalent to taking floor
		return ia;						// this is equivalent to doing floor(a + 0.5)
	}
	else
	{
		ia = (a - POINT_FIVE);
		return ia;
	}
}




Pixel_t FilterKernelOperator1(Pixel_t Window[FILTER_SIZE1][FILTER_SIZE1])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE1; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE1; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel1[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer

	if(valueInt <1024)						// take care of data overflow. This depends on the bit width of Pixel_t
		return valueInt;
	else
		return 1024;
}











Pixel_t FilterKernelOperator2(Pixel_t Window[FILTER_SIZE2][FILTER_SIZE2])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE2; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE2; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel2[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer


	if(valueInt <1024)						// take care of data overflow.
		return valueInt;
	else
		return 1024;
}





Pixel_t FilterKernelOperator3(Pixel_t Window[FILTER_SIZE3][FILTER_SIZE3])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE3; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE3; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel3[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer


	if(valueInt <1024)						// take care of data overflow.
		return valueInt;
	else
		return 1024;
}






// ======================================================= COMBINED GAUSSIAN BLUR FUNCTION =============================================================== //





void GaussianBlurHLS(	hls::stream< Pixel_t > &SrcImageStream,
						hls::stream< Pixel_t > &Gaussian1,
						hls::stream< Pixel_t > &Gaussian2,
						hls::stream< Pixel_t > &Gaussian3,
						hls::stream< Pixel_t > &DupGaussian2,
						hls::stream< Pixel_t > &DupImageStream)
{


#pragma HLS ARRAY_PARTITION variable=GaussianKernel3 complete dim=0
#pragma HLS ARRAY_PARTITION variable=GaussianKernel2 complete dim=0
#pragma HLS ARRAY_PARTITION variable=GaussianKernel1 complete dim=0


	Pixel_t window3[FILTER_SIZE3][FILTER_SIZE3];					// Sliding window
#pragma HLS ARRAY_PARTITION variable=window3 complete dim=0

	Pixel_t window2[FILTER_SIZE2][FILTER_SIZE2];					// Sliding window for gaussian1
#pragma HLS ARRAY_PARTITION variable=window2 complete dim=0

	Pixel_t window1[FILTER_SIZE1][FILTER_SIZE1];					// Sliding window for gaussian2
#pragma HLS ARRAY_PARTITION variable=window1 complete dim=0

	Pixel_t srcImageBuf[FILTER_SIZE3][WIDTH];						// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=srcImageBuf complete dim=1		// Static initializes buffer with zeros

	Pixel_t right_col[FILTER_SIZE3]; 								// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=right_col dim=0



	int Win1_Offset = (FILTER_OFFS3 - FILTER_OFFS1);
	int Win2_Offset = (FILTER_OFFS3 - FILTER_OFFS2);

L1:	for (int iRow = 0 ; iRow < HEIGHT + FILTER_OFFS3; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + FILTER_OFFS3; iCol++ )
		{
#pragma HLS PIPELINE


			Pixel_t pixelIn;												// Pixel data coming in
			if(iRow<HEIGHT && iCol < WIDTH)
			{
				pixelIn = SrcImageStream.read();
			}
			else
				pixelIn = 0 ;


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < FILTER_SIZE3-1; ii++)
				{
					right_col[ii]=srcImageBuf[ii][iCol]=srcImageBuf[ii+1][iCol];
				}
				right_col[FILTER_SIZE3-1] = srcImageBuf[FILTER_SIZE3-1][iCol] = pixelIn;
			}
			else
			{
				for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				{
					right_col[ii]=0;
				}
			}


			//sliding window:
			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				for(unsigned char jj = 0; jj < FILTER_SIZE3-1; jj++)
					window3[ii][jj] = window3[ii][jj+1];

			for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				window3[ii][FILTER_SIZE3-1] = right_col[ii];




// operate on the window and output the result on the output stream

			Pixel_t filtOut1, filtOut2, filtOut3, pixelOut ;
			if ( (iRow>=FILTER_OFFS3) && (iCol>=FILTER_OFFS3)  )
			{


				pixelOut = window3[FILTER_OFFS3][FILTER_OFFS3];

				for(int ii = 0 ; ii< FILTER_SIZE1; ii++)
					for(int jj = 0 ; jj < FILTER_SIZE1; jj++)
						window1[ii][jj] = window3[Win1_Offset + ii][Win1_Offset + jj];

				for(int ii = 0 ; ii< FILTER_SIZE2; ii++)
					for(int jj = 0 ; jj < FILTER_SIZE2; jj++)
						window2[ii][jj] = window3[Win2_Offset + ii][Win2_Offset + jj];



				filtOut1 = FilterKernelOperator1(window1); //

				filtOut2 = FilterKernelOperator2(window2); //

				filtOut3 = FilterKernelOperator3(window3); //




				Gaussian1.write(filtOut1);
				Gaussian2.write(filtOut2);
				DupGaussian2.write(filtOut2);
				Gaussian3.write(filtOut3);
				DupImageStream.write(pixelOut);

			}
		}//end of L1
	}//end of L2
}







// ============================================================================================ //


#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL, pGradients:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT], pGradients[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pKeypoints:SEQUENTIAL)
#pragma SDS data copy( pKeypoints[0:MAX_KEYPOINTS])

#pragma SDS data data_mover(pKeypoints:AXIDMA_SIMPLE)
//#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)


void sift_detect(	ap_uint<8>* pSrcImage,
					Grad_t* pGradients,
					Keypoint_t* pKeypoints,
					int contrastTh
						)
{

#pragma HLS INLINE OFF
#pragma HLS DATAFLOW


	hls::stream< Pixel_t > srcImageStream;
	hls::stream< Pixel_t > DupSrcImageStream;
	hls::stream< Pixel_t > GaussianStream1;
	hls::stream< Pixel_t > GaussianStream2;
	hls::stream< Pixel_t > GaussianStream3;
	hls::stream< Pixel_t > DupGaussianStream2;

	hls::stream< ap_int<16> > DOGStream1;
	hls::stream< ap_int<16> > DOGStream2;
	hls::stream< ap_int<16> > DOGStream3;

	hls::stream< ap_uint<16> > KeypointMapStream1;
	hls::stream< Fixed_t > pEigenRatioStream2;

	hls::stream< ap_uint<16> > KeypointMapStream2;

	hls::stream< Keypoint_t > KeypointStream;

	hls::stream<Grad_t> GradientStream;


// populate input image stream
	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Pixel_t PixelIn = pSrcImage[i];
//		Pixel_t GaussPixel1 = pGaussian1[i];
//		Pixel_t GaussPixel2 = pGaussian2[i];
//		Pixel_t GaussPixel3 = pGaussian3[i];

		srcImageStream.write( PixelIn );
//		GaussianStream1.write( GaussPixel1  );
//		GaussianStream2.write( GaussPixel2 );
//		GaussianStream3.write( GaussPixel3 );
	}


// Calling HLS functions

	GaussianBlurHLS(srcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DupGaussianStream2, DupSrcImageStream);

	build_DOG_hls(DupSrcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DOGStream1, DOGStream2, DOGStream3);

	detect_extrema_hls(DOGStream1, DOGStream2, DOGStream3, KeypointMapStream1, pEigenRatioStream2, contrastTh);

	culling_hls(KeypointMapStream1, pEigenRatioStream2, KeypointMapStream2);

	combine_keypoints_hls(KeypointMapStream2, KeypointStream);

	ComputeGradientsHLS(DupGaussianStream2, GradientStream);

	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Grad_t Grad2Out = GradientStream.read();

		pGradients[i] = Grad2Out;

	}

	for(int i=0; i<MAX_KEYPOINTS;i++)
	{

		#pragma HLS PIPELINE
		#pragma HLS LOOP_FLATTEN off

		pKeypoints[i] = KeypointStream.read();


	}

}






// ============================================================================================================== //



Grad_t ComputeGradientKernel(Pixel_t Window[CO_SIZE][CO_SIZE])
{

	ap_int<16> gradx = Window[CO_OFFS][CO_OFFS + 1] - Window[CO_OFFS][CO_OFFS -1] ;
	ap_int<16> grady = Window[CO_OFFS + 1][CO_OFFS] - Window[CO_OFFS - 1][CO_OFFS] ;


	int gradx_sq = gradx*gradx;
	int grady_sq = grady*grady;

	int mag = sqrt(gradx_sq + grady_sq);

	ap_int<16> maggrad = mag ;

	ap_int<2> sgngradx = sign(gradx);
	ap_int<2> sgngrady = sign(grady);

	ap_int<16> absgradx = ABS(gradx);
	ap_int<16> absgrady = ABS(grady);

	ap_uint<16> anggrad = (sgngradx > 0 & sgngrady > 0 & absgradx >  absgrady )*1 + 	\
						(sgngradx > 0 & sgngrady > 0 & absgradx <= absgrady )*2 + 	\
						(sgngradx < 0 & sgngrady > 0 & absgradx <  absgrady )*3 + 	\
						(sgngradx < 0 & sgngrady > 0 & absgradx >= absgrady )*4 + 	\
						(sgngradx < 0 & sgngrady < 0 & absgradx > absgrady )* 5 +   \
						(sgngradx < 0 & sgngrady < 0 & absgradx <= absgrady )*6 +	\
						(sgngradx > 0 & sgngrady < 0 & absgradx <  absgrady )*7 +	\
						(sgngradx > 0 & sgngrady < 0 & absgradx >= absgrady )*8;



	Grad_t OG ;

	OG.angGrad = anggrad;
	OG.magGrad = maggrad;


	return OG;

}






// Signum function - For the nonzero elements of complex X, sign(X) = X ./ ABS(X).

// 	For each element of X, sign(X) returns
//    	1 if the element is greater than zero,
//    	0 if it equals zero and
//     -1 if it is less than zero.
// Note that you need only two bits to store 1,0,-1
ap_int<2> sign(ap_int<16> a)
{

#pragma HLS INLINE


	if(a>0)
		return 1;
	else if(a<0)
		return -1;
	else
		return 0;
}





ap_int<16> ABS(ap_int<16> a)
{

#pragma HLS INLINE

	if(a<0)
		return -1*(a);
	else
		return a; 		// never reaches here

}





void ComputeGradientsHLS(	hls::stream< ap_uint<8> > &pGaussian2,
							hls::stream<Grad_t> &pGrad
							)
{


	Pixel_t WindowSrc[CO_SIZE][CO_SIZE];						// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0

	Pixel_t SrcImageBuf[CO_SIZE][WIDTH];						// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	Pixel_t SrcRightCol[CO_SIZE]; 							// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0

int idx = 0 ;

L1:	for (int iRow = 0 ; iRow < HEIGHT + CO_OFFS; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + CO_OFFS; iCol++ )
		{
	#pragma HLS PIPELINE

			Pixel_t PixelIn;

			if(iRow<HEIGHT && iCol < WIDTH)
			{
				PixelIn = pGaussian2.read();
			}
			else
			{
				PixelIn = 0 ;

			}


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < CO_SIZE - 1; ii++)
				{
					SrcRightCol[ii] = SrcImageBuf[ii][iCol]=SrcImageBuf[ii+1][iCol];
				}
				SrcRightCol[CO_SIZE-1] = SrcImageBuf[CO_SIZE-1][iCol] = PixelIn;
			}
			else
			{
				for(unsigned char ii = 0; ii < CO_SIZE; ii++)
				{
					SrcRightCol[ii] = 0 ;
				}
			}



			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < CO_SIZE; ii++)
			{
				for(unsigned char jj = 0; jj < CO_SIZE-1; jj++)
				{
					WindowSrc[ii][jj] = WindowSrc[ii][jj+1];					// scale 1
				}
			}

			for(unsigned char ii = 0; ii < CO_SIZE; ii++)
			{
				WindowSrc[ii][CO_SIZE-1] = SrcRightCol[ii];

			}


			// operate on the window and output the result on the output stream

			if ( (iRow >= CO_OFFS) && (iCol >= CO_OFFS)  )
			{

				Grad_t  OG_Data = ComputeGradientKernel(WindowSrc);		// Oriented Gradient

				pGrad.write( OG_Data);

				idx++ ;
			}


		}//end of L2
	}//end of L1



}








/*

#include "DetectKeypoints.hpp"



















//================================================== HLS FUNCTIONS =========================================================================//

// This function operates on the DOG pyramid. It takes in the three DOGs and does an extrema detection in a 3x3x3 neighbourhood
// After extracting the extrema it rejects keypoints that have low contrast or that are on an edge.
// Low contrast rejection is based on the contrastTh provided as an input
// Edge rejection is based on the eigenratio and eigenthreshold (constant set in the hpp)


/// <Summary>
/// Input Arguments:
/// 	- All three DOGs in the pyramid - DOG1, DOG2, DOG3
/// 	- contrast threshold
/// Returns:
/// 	- A Keypoint Image/Mask (dim. HeightxWidth) which has a zero for all non-keypoints and ABS(contrast value) i.e ABS(DOG2) for all pixels where a keypoint is detected
/// 	- An EigenRatio Image/MAsk, which has a value equal to the eigenRatio is its a non-edge point and -1 otherwise
/// </Summary>



void detect_extrema_hls(	hls::stream< ap_int<16> > &pDOG1,
							hls::stream< ap_int<16> > &pDOG2,
							hls::stream< ap_int<16> > &pDOG3,
							hls::stream< ap_uint<16> > &pKeypointMap1,
							hls::stream< Fixed_t > &pEigenRatio

						)
{


	ap_int<16> WindowSrc[N_SCALES][SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];				// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0								// This is a 3x3x3 window sliding across the 3 DOGs.


	ap_int<16> SrcImageBuf1[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf1 complete dim=1

	ap_int<16> SrcImageBuf2[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf2 complete dim=1

	ap_int<16> SrcImageBuf3[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf3 complete dim=1

	ap_int<16> SrcRightCol1[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG1)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol1 dim=0

	ap_int<16> SrcRightCol2[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG2)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol2 dim=0

	ap_int<16> SrcRightCol3[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column (on DOG3)
#pragma HLS ARRAY_PARTITION variable=SrcRightCol3 dim=0



	// The following buffers are used to create contrast and edge detection window
	// This is a sliding window on the DOG2. So this must have data type same as DOG2

	ap_int<16> Window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE];							// Window to do contrast and edge detection on
#pragma HLS ARRAY_PARTITION variable=Window complete dim=0

	ap_int<16> SrcImageBuf[SUPPRESS_NONMAX_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	ap_int<16> SrcRightCol[SUPPRESS_NONMAX_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0





int ContrastThreshold = 20;




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

				Window[ii][SUPPRESS_NONMAX_SIZE-1] = SrcRightCol[ii];				// This window is for contrast and edge rejection

			}


			// operate on the window and output the result on the output stream

			if ( (iRow>=SNMX_OFFSET) && (iCol>=SNMX_OFFSET)  )
			{


				ap_uint<16> kpOut;


				ap_int<16> centerPixel = ABS(Window[SNMX_OFFSET][SNMX_OFFSET]);		// To reject keypoints we look at the absolute value

				bool bKeypointDetected = IsLocalExtremum(WindowSrc);				// Check if center of the window is maximum

				bool bIsLowContrastPoint = ((centerPixel) < ContrastThreshold );	// Check if the center pixel is a low contrast point

				Fixed_t bKeypointEigenRatio = IsOnEdge(Window);						// Check if the keypoint is on edge - based on eigen ratio


				if(!bKeypointDetected || (bKeypointEigenRatio == -1 || bIsLowContrastPoint)) 	// if keypoint is not detected OR it does not satisfy the eigen and contrast thresholds then its not a valid keypoint
					kpOut = 0 ;
				else
					kpOut = centerPixel;



				pKeypointMap1.write(kpOut);										// write ABS(contrast) of the keypoint to the output. So that we can sort the keypoints later

				pEigenRatio.write(bKeypointEigenRatio);							// Eigen Ratio is also written to the output





			}


		}//end of L2
	}//end of L1



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

return extrema ;

}





Fixed_t IsOnEdge(ap_int<16> window[SUPPRESS_NONMAX_SIZE][SUPPRESS_NONMAX_SIZE])
{

	int rx =  SNMX_OFFSET;
	int ry =  SNMX_OFFSET ;


	// TODO: check equations :
	int fxx = window[ry][rx-1] + window[ry][rx+1] - 2*window[ry][rx];
	int fyy = window[ry-1][rx] + window[ry+1][rx] - 2*window[ry][rx];
	int fxy = window[ry-1][rx-1] + window[ry+1][rx+1] - window[ry+1][rx-1] - window[ry-1][rx+1];

	int trace = fxx + fyy;
	int deter = fxx*fyy - fxy*fxy ;


	float eigen_ratio = trace*trace/deter;



	Fixed_t eigenRatioOut ;

	if( eigen_ratio < 0 || eigen_ratio > EIGEN_RATIO_TH)
		eigenRatioOut = -1;
	else
		eigenRatioOut = eigen_ratio ;

	return eigenRatioOut;


}





// ============================================================ combine_keypoints_hls ===================================================================

// This function extracts an array of keypoints from the image of keypoints
/// <Summary>
/// Input Arguments:
/// 	- A full image of keypoints(dim. HEIGHTxWIDTH) - which is 0 for non keypoints and has ABS(contrast) for keypoints
/// Returns:
/// 	- An array of valid keypoints. Keypoint structure has x,y,contrast
///		- The max number of keypoints returned are always fixed = MAX_KEYPOINTS, so in cases when less than MAX_KEYPOINTS are found. a bunch of zeros are added to the array
/// </Summary>




//void combine_keypoints_hls(	hls::stream< ap_uint<16> > &pKeypointImage,
//							hls::stream< Keypoint_t > &pKeypoints )
//{
//
//
//	int nKeypoints = 0 ;
//	int idx = 0;
//
//	int nKeypointsTotal = 0 ;
//
//	for(int iRow = 0 ; iRow < HEIGHT ; iRow++ )
//	{
//		for(int iCol = 0 ; iCol < WIDTH; iCol++)
//		{
//#pragma HLS PIPELINE II = 1
//
//			ap_uint<16> value = pKeypointImage.read();
//
//
//			ap_uint<16> valueOut = 0 ;
//
//			if((iRow > WINDOW_SIZE && iCol > WINDOW_SIZE)&&(iRow< HEIGHT - WINDOW_SIZE && iCol < WIDTH - WINDOW_SIZE)  )	// this is to tackle borders.
//			{
//				if(value == 0  )
//					valueOut = 0 ;
//				else
//					valueOut = value;
//
//			}
//			else
//				valueOut = 0 ;
//
//
//
//			if(valueOut > 0)
//				nKeypointsTotal++;
//
//
//			if(valueOut > 0  && nKeypoints<MAX_KEYPOINTS)
//			{
//				Keypoint_t KP_Out ;
//				KP_Out.x = iCol;
//				KP_Out.y = iRow;
//				KP_Out.contrast = valueOut;
//
//				pKeypoints.write(KP_Out);
//
//				nKeypoints++;
//			}
//
//			idx++;
//		}
//	}
//
////	printf("total keypoint in the image = %d\n", nKeypointsTotal);
//
//
//	for(int n = 0 ; n < MAX_KEYPOINTS-nKeypoints; n++)				// Make sure that the output keypoint stream is completely populated with total no. of elements = MAX_KEYPOINTS
//	{																// Otherwise it causes a hang in hardware as the consumer of this stream is expecting MAX_KEYPOINTS
//		Keypoint_t KP;
//		KP.x = 0;
//		KP.y = 0;
//		KP.contrast = 0;
//
//		pKeypoints.write( KP );
//	}
//
//
//}







void combine_keypoints_hls(	hls::stream< ap_uint<16> > &pKeypointImage,
							hls::stream< Pixel_t > &pOutputKeypointImage )
{


	int nKeypoints = 0 ;

	int nKeypointsTotal = 0 ;

	for(int iRow = 0 ; iRow < HEIGHT ; iRow++ )
	{
		for(int iCol = 0 ; iCol < WIDTH; iCol++)
		{
#pragma HLS PIPELINE II = 1

			ap_uint<16> value = pKeypointImage.read();


			Pixel_t valueOut = 0 ;

			if((iRow > WINDOW_SIZE && iCol > WINDOW_SIZE)&&(iRow< HEIGHT - WINDOW_SIZE && iCol < WIDTH - WINDOW_SIZE)  )	// this is to tackle borders.
			{
				if(value == 0  )
					valueOut = 0 ;
				else
					valueOut = 255;
					//valueOut = value;
			}
			else
				valueOut = 0 ;


			pOutputKeypointImage.write(valueOut);

		}
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



	for(int iRow = 0 ; iRow < HEIGHT; iRow++)
	{

		for(int iCol = 0; iCol < WIDTH; iCol++)
		{

#pragma HLS PIPELINE II = 1

			value1 = pSrcImage.read();
			value2 = pGaussian1.read();
			value3 = pGaussian2.read();
			value4 = pGaussian3.read();



			Diff1 = value2 - value1 ;
			Diff2 = value3 - value2 ;
			Diff3 = value4 - value3 ;



			pDOG1.write(Diff1);
			pDOG2.write(Diff2);
			pDOG3.write(Diff3);


		}

	}


}




// ============================================ Culling ======================================================= //


void culling_hls(	hls::stream< ap_uint<16> > &pKeypointMap1,
					hls::stream< Fixed_t > &pEigenRatio,
					hls::stream< ap_uint<16> > &pKeypointOut
						)
{


	Fixed_t WindowSrc[CULL_SIZE][CULL_SIZE];									// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0

	Fixed_t SrcImageBuf1[CULL_SIZE][WIDTH];										// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf1 complete dim=1

	Fixed_t SrcRightCol1[CULL_SIZE]; 											// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol1 dim=0





	// The following buffers are used to create window on pKeypointMap1

	ap_uint<16> Window[CULL_SIZE][CULL_SIZE];
#pragma HLS ARRAY_PARTITION variable=Window complete dim=0

	ap_uint<16> SrcImageBuf[CULL_SIZE][WIDTH];									// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	ap_uint<16> SrcRightCol[CULL_SIZE]; 										// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0




L1:	for (int iRow = 0 ; iRow < HEIGHT + CULL_OFFSET; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + CULL_OFFSET; iCol++ )
		{
	#pragma HLS PIPELINE

			Fixed_t EigenRatioIn1;												// Input value on pEigenRatio
			ap_uint<16> PixelIn2;												// Input value on pKeypointMap1


			if(iRow<HEIGHT && iCol < WIDTH)
			{
				EigenRatioIn1 = pEigenRatio.read();								// Read EigenRatio (its -1 for non keypoints, EigenRatio for keypoints)
				PixelIn2 = pKeypointMap1.read();								// Read Contrast value (its 0 for non keypoints, ABS(contrast) for keypoints)
			}
			else
			{
				EigenRatioIn1 = 0 ;												// This is to handle borders. Zero values are assumed outside image dimensions
				PixelIn2 = 0 ;
			}


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < CULL_SIZE - 1; ii++)
				{
					SrcRightCol1[ii] = SrcImageBuf1[ii][iCol]=SrcImageBuf1[ii+1][iCol];
					SrcRightCol[ii] = SrcImageBuf[ii][iCol] = SrcImageBuf[ii+1][iCol];
				}

				SrcRightCol1[CULL_SIZE-1] = SrcImageBuf1[CULL_SIZE-1][iCol] = EigenRatioIn1;

				SrcRightCol[CULL_SIZE-1] = SrcImageBuf[CULL_SIZE-1][iCol] = PixelIn2;
			}
			else
			{
				for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
				{
					SrcRightCol1[ii] = 0 ;

					SrcRightCol[ii] = 0 ;
				}
			}



			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
			{
				for(unsigned char jj = 0; jj < CULL_SIZE-1; jj++)
				{
					WindowSrc[ii][jj] = WindowSrc[ii][jj+1];
					Window[ii][jj] = Window[ii][jj+1];
				}
			}

			for(unsigned char ii = 0; ii < CULL_SIZE; ii++)
			{
				WindowSrc[ii][CULL_SIZE-1] = SrcRightCol1[ii];
				Window[ii][CULL_SIZE-1] = SrcRightCol[ii];

			}


			// operate on the window and output the result on the output stream

			if ( (iRow>=CULL_OFFSET) && (iCol>=CULL_OFFSET)  )
			{


				bool bKeypointDetected ;

				ap_uint<16> centerPixel = Window[CULL_OFFSET][CULL_OFFSET];

				bKeypointDetected  = IsBestKeypoint(WindowSrc, Window);

				ap_uint<16> pixOut ;

				if (centerPixel >0 && bKeypointDetected > 0)								// if center pixel is more than zero it means this is a valid keypoint
					pixOut = centerPixel;													// now check if center of the window is maximum, if max. its a good keypoint
				else
					pixOut = 0 ;

				pKeypointOut.write(centerPixel);											// output will be 	- 0 if not a good keypoint
																							//				  	- ABS(contrast) if a good keypoint

			}


		}//end of L2
	}//end of L1

}


bool IsBestKeypoint(Fixed_t EigWindow[CULL_SIZE][CULL_SIZE], ap_uint<16> KPWindow[CULL_SIZE][CULL_SIZE])
{


	Fixed_t centerPixel = EigWindow[CULL_SIZE/2][CULL_SIZE/2]; 					//	center pixel that needs to be compared to the neighbours


	bool local_minima;

	bool extrema;


	local_minima = 1;


//printf("\n\nEigen Ratio Matrix: \n");
	for(int i = 0 ; i< CULL_SIZE; i++ )
	{
		for(int j = 0 ; j < CULL_SIZE; j++)
		{
//			printf("%f, ",(float) EigWindow[i][j]);
//			printf("%d,\t ",(int) KPWindow[i][j]);

			if(KPWindow[i][j] != 0 && (i!=CULL_OFFSET && j!=CULL_OFFSET ))
				local_minima = local_minima&&(centerPixel < EigWindow[i][j]);			// centerPixel here is the eigen ratio value for the keypoint
		}
//		printf("\n");
	}


	extrema = local_minima ;

return extrema ;



}

// ================================================================================= GAUSSIAN BLUR ============================================================== //


// 5x5 Gaussian Kernel
Mask_t GaussianKernel1[FILTER_SIZE1][FILTER_SIZE1] ={\
		{0.0002,    0.0033,    0.0081,    0.0033,    0.0002},	\
		{0.0033,    0.0479,    0.1164,    0.0479,    0.0033},	\
		{0.0081,    0.1164,    0.2831,    0.1164,    0.0081},	\
		{0.0033,    0.0479,    0.1164,    0.0479,    0.0033},	\
		{0.0002,    0.0033,    0.0081,    0.0033,    0.0002}	};



// 11x11 Gaussian Kernel, sigma = 2.25
Mask_t GaussianKernel2[FILTER_SIZE2][FILTER_SIZE2] ={\
	{0.000231624,0.000563409,0.001124803,0.001843074,0.002478693,0.002735999,0.002478693,0.001843074,0.001124803,0.000563409,0.000231624 },	\
	{0.000563409,0.001370449,0.002735999,0.004483141,0.006029235,0.006655114,0.006029235,0.004483141,0.002735999,0.001370449,0.000563409},	\
	{0.001124803,0.002735999,0.005462217,0.008950254,0.012036915,0.013286436,0.012036915,0.008950254,0.005462217,0.002735999,0.001124803},	\
	{0.001843074,0.004483141,0.008950254,0.014665665,0.019723393,0.021770826,0.019723393,0.014665665,0.008950254,0.004483141,0.001843074},	\
	{0.002478693,0.006029235,0.012036915,0.019723393,0.026525371,0.029278899,0.026525371,0.019723393,0.012036915,0.006029235,0.002478693},	\
	{0.002735999,0.006655114,0.013286436,0.021770826,0.029278899,0.032318264,0.029278899,0.021770826,0.013286436,0.006655114,0.002735999},	\
	{0.002478693,0.006029235,0.012036915,0.019723393,0.026525371,0.029278899,0.026525371,0.019723393,0.012036915,0.006029235,0.002478693},	\
	{0.001843074,0.004483141,0.008950254,0.014665665,0.019723393,0.021770826,0.019723393,0.014665665,0.008950254,0.004483141,0.001843074},	\
	{0.001124803,0.002735999,0.005462217,0.008950254,0.012036915,0.013286436,0.012036915,0.008950254,0.005462217,0.002735999,0.001124803},	\
	{0.000563409,0.001370449,0.002735999,0.004483141,0.006029235,0.006655114,0.006029235,0.004483141,0.002735999,0.001370449,0.000563409},	\
	{0.000231624,0.000563409,0.001124803,0.001843074,0.002478693,0.002735999,0.002478693,0.001843074,0.001124803,0.000563409,0.000231624}   };




Mask_t GaussianKernel3[FILTER_SIZE3][FILTER_SIZE3] ={\
	{0.000104598,0.000167455,0.000253646,0.000363508,0.000492897,0.000632345,0.000767552,0.000881491,0.00095782,0.000984704,0.00095782,0.000881491,0.000767552,0.000632345,0.000492897,0.000363508,0.000253646,0.000167455,0.000104598},	\
	{0.000167455,0.000268085,0.000406071,0.000581953,0.000789096,0.001012343,0.001228802,0.00141121,0.001533408,0.001576448,0.001533408,0.00141121,0.001228802,0.001012343,0.000789096,0.000581953,0.000406071,0.000268085,0.000167455},	\
	{0.000253646,0.000406071,0.000615081,0.000881491,0.001195253,0.001533408,0.00186128,0.002137576,0.002322671,0.002387864,0.002322671,0.002137576,0.00186128,0.001533408,0.001195253,0.000881491,0.000615081,0.000406071,0.000253646},	\
	{0.000363508,0.000581953,0.000881491,0.001263292,0.001712954,0.002197575,0.002667458,0.003063427,0.003328691,0.003422122,0.003328691,0.003063427,0.002667458,0.002197575,0.001712954,0.001263292,0.000881491,0.000581953,0.000363508},	\
	{0.000492897,0.000789096,0.001195253,0.001712954,0.002322671,0.002979789,0.003616925,0.004153837,0.004513521,0.004640208,0.004513521,0.004153837,0.003616925,0.002979789,0.002322671,0.001712954,0.001195253,0.000789096,0.000492897},	\
	{0.000632345,0.001012343,0.001533408,0.002197575,0.002979789,0.003822817,0.004640208,0.005329021,0.005790464,0.005952993,0.005790464,0.005329021,0.004640208,0.003822817,0.002979789,0.002197575,0.001533408,0.001012343,0.000632345},	\
	{0.000767552,0.001228802,0.00186128,0.002667458,0.003616925,0.004640208,0.005632373,0.006468467,0.007028576,0.007225856,0.007028576,0.006468467,0.005632373,0.004640208,0.003616925,0.002667458,0.00186128,0.001228802,0.000767552},	\
	{0.000881491,0.00141121,0.002137576,0.003063427,0.004153837,0.005329021,0.006468467,0.007428674,0.008071928,0.008298494,0.008071928,0.007428674,0.006468467,0.005329021,0.004153837,0.003063427,0.002137576,0.00141121,0.000881491},	\
	{0.00095782,0.001533408,0.002322671,0.003328691,0.004513521,0.005790464,0.007028576,0.008071928,0.008770882,0.009017066,0.008770882,0.008071928,0.007028576,0.005790464,0.004513521,0.003328691,0.002322671,0.001533408,0.00095782},	\
	{0.000984704,0.001576448,0.002387864,0.003422122,0.004640208,0.005952993,0.007225856,0.008298494,0.009017066,0.00927016,0.009017066,0.008298494,0.007225856,0.005952993,0.004640208,0.003422122,0.002387864,0.001576448,0.000984704},	\
	{0.00095782,0.001533408,0.002322671,0.003328691,0.004513521,0.005790464,0.007028576,0.008071928,0.008770882,0.009017066,0.008770882,0.008071928,0.007028576,0.005790464,0.004513521,0.003328691,0.002322671,0.001533408,0.00095782},	\
	{0.000881491,0.00141121,0.002137576,0.003063427,0.004153837,0.005329021,0.006468467,0.007428674,0.008071928,0.008298494,0.008071928,0.007428674,0.006468467,0.005329021,0.004153837,0.003063427,0.002137576,0.00141121,0.000881491},	\
	{0.000767552,0.001228802,0.00186128,0.002667458,0.003616925,0.004640208,0.005632373,0.006468467,0.007028576,0.007225856,0.007028576,0.006468467,0.005632373,0.004640208,0.003616925,0.002667458,0.00186128,0.001228802,0.000767552},	\
	{0.000632345,0.001012343,0.001533408,0.002197575,0.002979789,0.003822817,0.004640208,0.005329021,0.005790464,0.005952993,0.005790464,0.005329021,0.004640208,0.003822817,0.002979789,0.002197575,0.001533408,0.001012343,0.000632345},	\
	{0.000492897,0.000789096,0.001195253,0.001712954,0.002322671,0.002979789,0.003616925,0.004153837,0.004513521,0.004640208,0.004513521,0.004153837,0.003616925,0.002979789,0.002322671,0.001712954,0.001195253,0.000789096,0.000492897},	\
	{0.000363508,0.000581953,0.000881491,0.001263292,0.001712954,0.002197575,0.002667458,0.003063427,0.003328691,0.003422122,0.003328691,0.003063427,0.002667458,0.002197575,0.001712954,0.001263292,0.000881491,0.000581953,0.000363508},	\
	{0.000253646,0.000406071,0.000615081,0.000881491,0.001195253,0.001533408,0.00186128,0.002137576,0.002322671,0.002387864,0.002322671,0.002137576,0.00186128,0.001533408,0.001195253,0.000881491,0.000615081,0.000406071,0.000253646},	\
	{0.000167455,0.000268085,0.000406071,0.000581953,0.000789096,0.001012343,0.001228802,0.00141121,0.001533408,0.001576448,0.001533408,0.00141121,0.001228802,0.001012343,0.000789096,0.000581953,0.000406071,0.000268085,0.000167455},	\
	{0.000104598,0.000167455,0.000253646,0.000363508,0.000492897,0.000632345,0.000767552,0.000881491,0.00095782,0.000984704,0.00095782,0.000881491,0.000767552,0.000632345,0.000492897,0.000363508,0.000253646,0.000167455,0.000104598}		};








int RoundToInt(Fixed_t a)
{

	Fixed_t POINT_FIVE = 0.5;
#pragma HLS INLINE

	int ia;
	if(a>=0)
	{
		ia = (a + POINT_FIVE); 			// adding 0.5 to round it off to correct value. Assigning float to int here is equivalent to taking floor
		return ia;						// this is equivalent to doing floor(a + 0.5)
	}
	else
	{
		ia = (a - POINT_FIVE);
		return ia;
	}
}




Pixel_t FilterKernelOperator1(Pixel_t Window[FILTER_SIZE1][FILTER_SIZE1])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE1; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE1; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel1[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer

	if(valueInt <255)						// take care of data overflow. This depends on the bit width of Pixel_t
		return valueInt;
	else
		return 255;
}











Pixel_t FilterKernelOperator2(Pixel_t Window[FILTER_SIZE2][FILTER_SIZE2])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE2; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE2; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel2[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer


	if(valueInt <255)						// take care of data overflow.
		return valueInt;
	else
		return 255;
}





Pixel_t FilterKernelOperator3(Pixel_t Window[FILTER_SIZE3][FILTER_SIZE3])
{

	Fixed_t value = 0 ;				// <32,17> provides better precision than <16,9>

	// Pipeline directive not required as the parent loop is pipelined
	for(int iWinRow = 0 ; iWinRow < FILTER_SIZE3; iWinRow++)
	{
		for(int iWinCol = 0 ; iWinCol< FILTER_SIZE3; iWinCol++)
		{
				value += Window[iWinRow][iWinCol]*GaussianKernel3[iWinCol][iWinRow] ;
		}
	}

	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer


	if(valueInt <255)						// take care of data overflow.
		return valueInt;
	else
		return 255;
}






// ======================================================= COMBINED GAUSSIAN BLUR FUNCTION =============================================================== //





void GaussianBlurHLS(	hls::stream< Pixel_t > &SrcImageStream,
						hls::stream< Pixel_t > &Gaussian1,
						hls::stream< Pixel_t > &Gaussian2,
						hls::stream< Pixel_t > &Gaussian3,
						hls::stream< Pixel_t > &DupGaussian2,
						hls::stream< Pixel_t > &DupImageStream)
{


#pragma HLS ARRAY_PARTITION variable=GaussianKernel3 complete dim=0
#pragma HLS ARRAY_PARTITION variable=GaussianKernel2 complete dim=0
#pragma HLS ARRAY_PARTITION variable=GaussianKernel1 complete dim=0


	Pixel_t window3[FILTER_SIZE3][FILTER_SIZE3];					// Sliding window
#pragma HLS ARRAY_PARTITION variable=window3 complete dim=0

	Pixel_t window2[FILTER_SIZE2][FILTER_SIZE2];					// Sliding window for gaussian1
#pragma HLS ARRAY_PARTITION variable=window2 complete dim=0

	Pixel_t window1[FILTER_SIZE1][FILTER_SIZE1];					// Sliding window for gaussian2
#pragma HLS ARRAY_PARTITION variable=window1 complete dim=0

	Pixel_t srcImageBuf[FILTER_SIZE3][WIDTH];						// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=srcImageBuf complete dim=1		// Static initializes buffer with zeros

	Pixel_t right_col[FILTER_SIZE3]; 								// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=right_col dim=0



	int Win1_Offset = (FILTER_OFFS3 - FILTER_OFFS1);
	int Win2_Offset = (FILTER_OFFS3 - FILTER_OFFS2);

L1:	for (int iRow = 0 ; iRow < HEIGHT + FILTER_OFFS3; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + FILTER_OFFS3; iCol++ )
		{
#pragma HLS PIPELINE


			Pixel_t pixelIn;												// Pixel data coming in
			if(iRow<HEIGHT && iCol < WIDTH)
			{
				pixelIn = SrcImageStream.read();
			}
			else
				pixelIn = 0 ;


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < FILTER_SIZE3-1; ii++)
				{
					right_col[ii]=srcImageBuf[ii][iCol]=srcImageBuf[ii+1][iCol];
				}
				right_col[FILTER_SIZE3-1] = srcImageBuf[FILTER_SIZE3-1][iCol] = pixelIn;
			}
			else
			{
				for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				{
					right_col[ii]=0;
				}
			}


			//sliding window:
			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				for(unsigned char jj = 0; jj < FILTER_SIZE3-1; jj++)
					window3[ii][jj] = window3[ii][jj+1];

			for(unsigned char ii = 0; ii < FILTER_SIZE3; ii++)
				window3[ii][FILTER_SIZE3-1] = right_col[ii];




// operate on the window and output the result on the output stream

			Pixel_t filtOut1, filtOut2, filtOut3, pixelOut ;
			if ( (iRow>=FILTER_OFFS3) && (iCol>=FILTER_OFFS3)  )
			{


				pixelOut = window3[FILTER_OFFS3][FILTER_OFFS3];

				for(int ii = 0 ; ii< FILTER_SIZE1; ii++)
					for(int jj = 0 ; jj < FILTER_SIZE1; jj++)
						window1[ii][jj] = window3[Win1_Offset + ii][Win1_Offset + jj];

				for(int ii = 0 ; ii< FILTER_SIZE2; ii++)
					for(int jj = 0 ; jj < FILTER_SIZE2; jj++)
						window2[ii][jj] = window3[Win2_Offset + ii][Win2_Offset + jj];



				filtOut1 = FilterKernelOperator1(window1); //

				filtOut2 = FilterKernelOperator2(window2); //

				filtOut3 = FilterKernelOperator3(window3); //




				Gaussian1.write(filtOut1);
				Gaussian2.write(filtOut2);
				DupGaussian2.write(filtOut2);
				Gaussian3.write(filtOut3);
				DupImageStream.write(pixelOut);

			}
		}//end of L1
	}//end of L2
}







// ============================================================================================ //


#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL, pGradients:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT], pGradients[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pOutputKeypointImage:SEQUENTIAL)
#pragma SDS data copy( pOutputKeypointImage[0:WIDTH*HEIGHT])
//#pragma SDS data data_mover(pOutputKeypointImage:AXIDMA_SIMPLE)
//#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)


void sift_detect(	ap_uint<8>* pSrcImage,
					Pixel_t* pOutputKeypointImage,
					Grad_t* pGradients
						)
{

#pragma HLS INLINE OFF
#pragma HLS DATAFLOW


	hls::stream< Pixel_t > srcImageStream;
	hls::stream< Pixel_t > DupSrcImageStream;
	hls::stream< Pixel_t > GaussianStream1;
	hls::stream< Pixel_t > GaussianStream2;
	hls::stream< Pixel_t > GaussianStream3;
	hls::stream< Pixel_t > DupGaussianStream2;

	hls::stream< ap_int<16> > DOGStream1;
	hls::stream< ap_int<16> > DOGStream2;
	hls::stream< ap_int<16> > DOGStream3;

	hls::stream< ap_uint<16> > KeypointMapStream1;
	hls::stream< Fixed_t > pEigenRatioStream2;

	hls::stream< ap_uint<16> > KeypointMapStream2;

//	hls::stream< Keypoint_t > KeypointStream;
	hls::stream<Pixel_t> OutputKeypointImageStream;


	hls::stream<Grad_t> GradientStream;


// populate input image stream
	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Pixel_t PixelIn = pSrcImage[i];
//		Pixel_t GaussPixel1 = pGaussian1[i];
//		Pixel_t GaussPixel2 = pGaussian2[i];
//		Pixel_t GaussPixel3 = pGaussian3[i];

		srcImageStream.write( PixelIn );
//		GaussianStream1.write( GaussPixel1  );
//		GaussianStream2.write( GaussPixel2 );
//		GaussianStream3.write( GaussPixel3 );
	}


// Calling HLS functions

	GaussianBlurHLS(srcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DupGaussianStream2, DupSrcImageStream);

	build_DOG_hls(DupSrcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DOGStream1, DOGStream2, DOGStream3);

	detect_extrema_hls(DOGStream1, DOGStream2, DOGStream3, KeypointMapStream1, pEigenRatioStream2);

	culling_hls(KeypointMapStream1, pEigenRatioStream2, KeypointMapStream2);

	combine_keypoints_hls(KeypointMapStream2, OutputKeypointImageStream);

	ComputeGradientsHLS(DupGaussianStream2, GradientStream);

	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Grad_t Grad2Out = GradientStream.read();

		Pixel_t KPOut = OutputKeypointImageStream.read();

		pGradients[i] = Grad2Out;
		pOutputKeypointImage[i] = KPOut;

	}



}






// ============================================================================================================== //



Grad_t ComputeGradientKernel(Pixel_t Window[CO_SIZE][CO_SIZE])
{

	ap_int<16> gradx = Window[CO_OFFS][CO_OFFS + 1] - Window[CO_OFFS][CO_OFFS -1] ;
	ap_int<16> grady = Window[CO_OFFS + 1][CO_OFFS] - Window[CO_OFFS - 1][CO_OFFS] ;


//	printf("window- y0, y1, y2 = %d, %d, %d \n", (int)Window[0][1],(int)Window[1][1], (int)Window[2][1]);
//	printf(" gradx, grady = %d, %d \n",(int)gradx, (int)grady);

	int gradx_sq = gradx*gradx;
	int grady_sq = grady*grady;

	int mag = sqrt(gradx_sq + grady_sq);



	ap_int<16> maggrad = mag ;

	ap_int<2> sgngradx = sign(gradx);
	ap_int<2> sgngrady = grady;

	ap_int<16> absgradx = ABS(gradx);
	ap_int<16> absgrady = ABS(grady);

	ap_uint<16> anggrad = (sgngradx > 0 & sgngrady > 0 & absgradx >  absgrady )*1 + 	\
						(sgngradx > 0 & sgngrady > 0 & absgradx <= absgrady )*2 + 	\
						(sgngradx < 0 & sgngrady > 0 & absgradx <  absgrady )*3 + 	\
						(sgngradx < 0 & sgngrady > 0 & absgradx >= absgrady )*4 + 	\
						(sgngradx < 0 & sgngrady < 0 & absgradx > absgrady )* 5 +   \
						(sgngradx < 0 & sgngrady < 0 & absgradx <= absgrady )*6 +	\
						(sgngradx > 0 & sgngrady < 0 & absgradx <  absgrady )*7 +	\
						(sgngradx > 0 & sgngrady < 0 & absgradx >= absgrady )*8;



	Grad_t OG ;

	OG.angGrad = anggrad;
	OG.magGrad = maggrad;


	return OG;

}






// Signum function - For the nonzero elements of complex X, sign(X) = X ./ ABS(X).

// 	For each element of X, sign(X) returns
//    	1 if the element is greater than zero,
//    	0 if it equals zero and
//     -1 if it is less than zero.
// Note that you need only two bits to store 1,0,-1
ap_int<2> sign(ap_int<16> a)
{

#pragma HLS INLINE


	if(a>0)
		return 1;
	else if(a<0)
		return -1;
	else
		return 0;
}





ap_int<16> ABS(ap_int<16> a)
{

#pragma HLS INLINE

	if(a<0)
		return -1*(a);
	else
		return a; 		// never reaches here

}





void ComputeGradientsHLS(	hls::stream< ap_uint<8> > &pGaussian2,
							hls::stream<Grad_t> &pGrad
							)
{


	Pixel_t WindowSrc[CO_SIZE][CO_SIZE];						// Window is completely partitioned providing random access
#pragma HLS ARRAY_PARTITION variable=WindowSrc complete dim=0

	Pixel_t SrcImageBuf[CO_SIZE][WIDTH];						// Image Buffer on which the kernel operates.
#pragma HLS ARRAY_PARTITION variable=SrcImageBuf complete dim=1

	Pixel_t SrcRightCol[CO_SIZE]; 							// right-most, incoming column
#pragma HLS ARRAY_PARTITION variable=SrcRightCol dim=0

int idx = 0 ;

L1:	for (int iRow = 0 ; iRow < HEIGHT + CO_OFFS; iRow++ )
	{
#pragma HLS LATENCY min=0
	L2:	for(int iCol = 0 ; iCol < WIDTH + CO_OFFS; iCol++ )
		{
	#pragma HLS PIPELINE

			Pixel_t PixelIn;

			if(iRow<HEIGHT && iCol < WIDTH)
			{
				PixelIn = pGaussian2.read();
			}
			else
			{
				PixelIn = 0 ;

			}


			// Shift image buffer and get new line in
			if(iCol < WIDTH)
			{
				for(unsigned char ii = 0; ii < CO_SIZE - 1; ii++)
				{
					SrcRightCol[ii] = SrcImageBuf[ii][iCol]=SrcImageBuf[ii+1][iCol];
				}
				SrcRightCol[CO_SIZE-1] = SrcImageBuf[CO_SIZE-1][iCol] = PixelIn;
			}
			else
			{
				for(unsigned char ii = 0; ii < CO_SIZE; ii++)
				{
					SrcRightCol[ii] = 0 ;
				}
			}



			//Shift from left to right the sliding window to make room for the new column
			for(unsigned char ii = 0; ii < CO_SIZE; ii++)
			{
				for(unsigned char jj = 0; jj < CO_SIZE-1; jj++)
				{
					WindowSrc[ii][jj] = WindowSrc[ii][jj+1];					// scale 1
				}
			}

			for(unsigned char ii = 0; ii < CO_SIZE; ii++)
			{
				WindowSrc[ii][CO_SIZE-1] = SrcRightCol[ii];

			}


			// operate on the window and output the result on the output stream

			if ( (iRow >= CO_OFFS) && (iCol >= CO_OFFS)  )
			{

				Grad_t  OG_Data = ComputeGradientKernel(WindowSrc);		// Oriented Gradient


//				printf("iRow, iCol = %d, %d, grad Mag = %d\n", iRow-CO_OFFS, iCol-CO_OFFS, (int)OG_Data.magGrad);

				pGrad.write( OG_Data);

				idx++ ;
			}


		}//end of L2
	}//end of L1



}
*/


