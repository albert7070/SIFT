
#include "GaussianBlur.hpp"




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


	// 5x5 Gaussian Kernel, sigma = .75
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





	// 19x19 Gaussian Kernel, sigma = 4.25

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

			Fixed_t kernelValue = GaussianKernel3[iWinCol][iWinRow];
			value += Window[iWinRow][iWinCol]*kernelValue ;
		}
	}

//	int valueInt = RoundToInt(value);		// Matlab's int16() rounds off to nearest integer

	int valueInt = RoundToInt(value);

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



				if(iRow == 11 && iCol == 11)
				{
					for(int iWinRow = 0 ; iWinRow < FILTER_SIZE1; iWinRow++)
					{
						printf("\n" );
						for(int iWinCol = 0 ; iWinCol< FILTER_SIZE1; iWinCol++)
						{
								printf("%d, ", (int)window1[iWinRow][iWinCol]) ;

						}
						printf("\n" );
						for(int iWinCol = 0 ; iWinCol< FILTER_SIZE1; iWinCol++)
						{
								printf("%f, ", (float)GaussianKernel1[iWinRow][iWinCol]) ;

						}
						printf("\n" );
					}

				}



				filtOut1 = FilterKernelOperator1(window1); //

				filtOut2 = FilterKernelOperator2(window2); //

				filtOut3 = FilterKernelOperator3(window3); //




				Gaussian1.write(filtOut1);
				Gaussian2.write(filtOut2);
				Gaussian3.write(filtOut3);

				DupImageStream.write(pixelOut);

			}
		}//end of L1
	}//end of L2
}








