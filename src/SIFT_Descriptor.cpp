#include "SIFT_Descriptor.hpp"
#include "mask_all.hpp"
#include "circle_mask.hpp"
#include "string.h"



//========================================================================================================================//
//=================================== Compute Main Orientation for the descriptor ========================================//
//========================================================================================================================//

#pragma SDS data access_pattern(pKeypoint:SEQUENTIAL,  pMainOrientation:SEQUENTIAL)
#pragma SDS data copy( pKeypoint[0:MAX_KEYPOINTS], pMainOrientation[0:MAX_KEYPOINTS])
#pragma SDS data data_mover(pKeypoint:AXIDMA_SIMPLE)

#pragma SDS data zero_copy(  pGrad[0:WIDTH*HEIGHT])

void compute_main_orientation(	Grad_t* pGrad,
								Keypoint_t* pKeypoint,
								ap_uint<8>* pMainOrientation)
{

	Grad_t GradLine[WINDOW_SIZE];


#pragma HLS ARRAY_PARTITION variable=MaskAll complete dim=2

int radius = 71/2;

L1:	for (int kpIdx = 0; kpIdx <MAX_KEYPOINTS; kpIdx++)
	{

		int Hist[9];
#pragma HLS ARRAY_PARTITION variable=Hist complete dim=0

		Keypoint_t kptIn = pKeypoint[kpIdx];

		ap_uint<16> ptX = kptIn.x;
		ap_uint<16> ptY = kptIn.y;


//		printf("\n kp = %d", kpIdx);

		for(int k = 0 ; k < 9; k++)
		{
#pragma HLS UNROLL
			Hist[k] = 0 ;
		}


		ap_uint<8> mainOrientation = 0 ;
		if(ptX>0 && ptY>0)
				{

//			printf("keypoint(x,y) = %d, %d, ",(int)ptX,(int)ptY);

				for(int y=ptY - radius; y <= ptY + radius; y++ )
				{

					//Burst read response MagGrad and AngGrad line from DDR and copy to local buffer
					{
	//#pragma HLS DATAFLOW
	// Combine the gradients into one structure so that you don't have to do two memcopies.
						memcpy(GradLine, (Grad_t*)&pGrad[y*WIDTH + ptX - radius], WINDOW_SIZE*sizeof(Grad_t));
			//			memcpy(MagGradLine, (ap_int<16>*)&pMagGrad[y*WIDTH + ptX - radius], WINDOW_SIZE*sizeof(ap_int<16>));
					}

					int iLine = y - ptY + radius ;

					for(int x=0; x < 2* radius; x++)
					{
						// #pragma HLS LOOP_TRIPCOUNT min=9 max=9
						#pragma HLS LOOP_FLATTEN off
						//#pragma HLS UNROLL
						#pragma HLS PIPELINE

	//					ap_uint<8> AngGrad  = AngGradLine[x];
	//					ap_int<16> MagGrad  = MagGradLine[x];

						ap_uint<16> AngGrad = GradLine[x].angGrad;
						ap_int<16> MagGrad  = GradLine[x].magGrad;

						ap_uint<16> maskValue = MaskAll[iLine][x];


						int HistData = MagGrad*maskValue;

						//int HistData = MagGrad;

						Hist[AngGrad] += HistData;

					}


				}


				mainOrientation = ComputeMainOrientation(Hist);
				}
			pMainOrientation[kpIdx] = mainOrientation;
	}

}



ap_uint<8> ComputeMainOrientation(int Hist[9])
{
//	printf("computing orientation\n");
	int value ;
	int maxValue = 0 ;
	ap_uint<8> mainOrientation = 0;
//	printf(" Histogram: ");
	for(int i = 1; i <9; i++)
	{
		value = Hist[i]  ;

//		printf("%d ,", value);

		if(value > maxValue )
		{
			maxValue = value;
			mainOrientation = i;
		}

	}
//	printf("\n");
	return mainOrientation;

}













//========================================================================================================================//
//================================= Compute Histograms for the circular sub-regions ======================================//
//========================================================================================================================//




void compute_histograms_hls(	Grad_t* pGrad,
								hls::stream<Keypoint_t> &pKeypoint,
								hls::stream<Hist_t> &pHist0,
								hls::stream<Hist_t> &pHist1,
								hls::stream<Hist_t> &pHist2,
								hls::stream<Hist_t> &pHist3,
								hls::stream<Hist_t> &pHist4,
								hls::stream<Hist_t> &pHist5,
								hls::stream<Hist_t> &pHist6,
								hls::stream<Hist_t> &pHist7,
								hls::stream<Hist_t> &pHist8
							)
{

	Grad_t GradBuf[WINDOW_SIZE][WINDOW_SIZE];

#pragma HLS ARRAY_PARTITION variable=MaskAll complete dim=2

int radius = 71/2;

L1:	for (int kpIdx = 0; kpIdx <MAX_KEYPOINTS; kpIdx++)
	{

		ap_uint<32> Hist[9][9];
#pragma HLS ARRAY_PARTITION variable=Hist complete dim=0

		Keypoint_t kptIn = pKeypoint.read();

		ap_uint<16> ptX = kptIn.x;
		ap_uint<16> ptY = kptIn.y;


//		printf("\n");

		for(int j = 0 ; j < 9; j++)
		{
			for(int k = 0 ; k < 9; k++)
			{
				#pragma HLS UNROLL
				Hist[j][k] = 0 ;
			}
		}

		Hist_t histData[NCIRCLES]; // for all 9 subregions

		if(ptX>0 && ptY>0)
				{
		int LineID = 0 ;


		for(int y=ptY - radius; y <= ptY + radius; y++ )
		{
			//Burst read response MagGrad and AngGrad line from DDR and copy to local buffer
			memcpy(&GradBuf[LineID][0], (Grad_t*)&pGrad[y*WIDTH + ptX - radius], WINDOW_SIZE*sizeof(Grad_t));
			LineID++;
		}

			ap_uint<16> AngGrad[NCIRCLES];
			ap_uint<16> MagGrad[NCIRCLES];

L_CalcHist:for(int i = 0; i < MASK_SIZE; i++)
			{
				for(int j = 0 ; j< MASK_SIZE ; j++)
				{
					#pragma HLS LOOP_FLATTEN off
					//#pragma HLS UNROLL
					#pragma HLS PIPELINE

					for(int n = 0 ; n < NCIRCLES; n++)
					{

						#pragma HLS UNROLL
						AngGrad[n]  = GradBuf[YStart[n] + i][XStart[n] + j].angGrad;
						MagGrad[n]  = GradBuf[YStart[n] + i][XStart[n] + j].magGrad;


						ap_uint<8> MaskValue = CircleMask[i*25 + j];			// the mask remains same for all sub-regions

						ap_uint<16> iDir = AngGrad[n];

						Hist[n][iDir] += MaskValue * MagGrad[n];
					}
				}
			}




			// Pack Histograms to a single data structure
		L_PackHist:	for(int n = 0 ; n < NCIRCLES; n++)
			{
			#pragma HLS UNROLL

//#pragma HLS PIPELINE
				histData[n].bin0 = Hist[n][1];
				histData[n].bin1 = Hist[n][2];
				histData[n].bin2 = Hist[n][3];
				histData[n].bin3 = Hist[n][4];
				histData[n].bin4 = Hist[n][5];
				histData[n].bin5 = Hist[n][6];
				histData[n].bin6 = Hist[n][7];
				histData[n].bin7 = Hist[n][8];
			}

				}
//			printf("\n\nHistogram:");
//			for(int icircle = 0 ; icircle < NCIRCLES; icircle++)
//			{
//				printf("\nsubregion%d : ", icircle);
//				for(int i = 7 ; i >= 0; i--)
//				{
//					printf("%d, ", (int)histData[icircle](i*32+31, i*32) );
//				}
//			}

			pHist0.write( histData[0] );
			pHist1.write( histData[1] );
			pHist2.write( histData[2] );
			pHist3.write( histData[3] );
			pHist4.write( histData[4] );
			pHist5.write( histData[5] );
			pHist6.write( histData[6] );
			pHist7.write( histData[7] );
			pHist8.write( histData[8] );

	}

}










//========================================================================================================================//
//================================= Reorder Histograms for Final Descriptor Generation ===================================//
//========================================================================================================================//



void reorder_histograms_hls(hls::stream< ap_uint<8> >  &pMainOrientation,
							hls::stream< Hist_t > &pHist0,
							hls::stream< Hist_t > &pHist1,
							hls::stream< Hist_t > &pHist2,
							hls::stream< Hist_t > &pHist3,
							hls::stream< Hist_t > &pHist4,
							hls::stream< Hist_t > &pHist5,
							hls::stream< Hist_t > &pHist6,
							hls::stream< Hist_t > &pHist7,
							hls::stream< Hist_t > &pHist8,
							hls::stream< ap_uint<32> > &pDescriptorOut)
{




	int iDescOutIdx = 0 ;

	for(int kpIdx = 0 ; kpIdx < MAX_KEYPOINTS; kpIdx++)
	{

		Hist_t histDataIn[NCIRCLES]; 								// for all circular subregions
		Hist_t histDataOut[NCIRCLES]; 								// for all circular subregions


		ap_uint<32> Hist[9][9];
#pragma HLS ARRAY_PARTITION variable=Hist complete dim=1




		// Read the incoming Histograms for each individual sub-region
		histDataIn[0] = pHist0.read() ;
		histDataIn[1] = pHist1.read() ;
		histDataIn[2] = pHist2.read() ;
		histDataIn[3] = pHist3.read() ;
		histDataIn[4] = pHist4.read() ;
		histDataIn[5] = pHist5.read() ;
		histDataIn[6] = pHist6.read() ;
		histDataIn[7] = pHist7.read() ;
		histDataIn[8] = pHist8.read() ;





		ap_uint<8> mainOrientation = pMainOrientation.read() ;	// Read main orientation


//		printf("\nmain orientation = %d", (int) mainOrientation);

		ap_uint<3> nBinID = 0 ;
		nBinID = nBinID - mainOrientation;						// Circular Shift binID & Circle ID. This is intentionally designed to be a 3 bit number
																// So that it circ shifts when main orientation is added/subtracted from it
		ap_uint<3> nCircleID = 0 ;
		nCircleID = nCircleID + mainOrientation;

// Unpack histograms into histData buffer and reorder sub-regions.
// Do not change the last circle.

//		printf("\n\nIncoming Histogram: ");
		for(int n = 0 ; n < NCIRCLES-1; n++)
		{
//		#pragma HLS UNROLL
			Hist[n][ap_uint<3>(nBinID+0) ] = histDataIn[nCircleID].bin0;
			Hist[n][ap_uint<3>(nBinID+1) ] = histDataIn[nCircleID].bin1;
			Hist[n][ap_uint<3>(nBinID+2) ] = histDataIn[nCircleID].bin2;
			Hist[n][ap_uint<3>(nBinID+3) ] = histDataIn[nCircleID].bin3;
			Hist[n][ap_uint<3>(nBinID+4) ] = histDataIn[nCircleID].bin4;
			Hist[n][ap_uint<3>(nBinID+5) ] = histDataIn[nCircleID].bin5;
			Hist[n][ap_uint<3>(nBinID+6) ] = histDataIn[nCircleID].bin6;
			Hist[n][ap_uint<3>(nBinID+7) ] = histDataIn[nCircleID].bin7;

//			printf("\nSubregion%d - %d, %d, %d, %d, %d, %d, %d, %d",n,(int) histDataIn[ n].bin0, (int) histDataIn[ n].bin1 , (int) histDataIn[n].bin2 , (int) histDataIn[n].bin3 , (int) histDataIn[n].bin4 , (int) histDataIn[n].bin5 , (int) histDataIn[n].bin6 , (int) histDataIn[n].bin7  );

			nCircleID++ ;
		}

		Hist[NCIRCLES-1][ap_uint<3>(nBinID+0) ] = histDataIn[NCIRCLES-1].bin0;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+1) ] = histDataIn[NCIRCLES-1].bin1;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+2) ] = histDataIn[NCIRCLES-1].bin2;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+3) ] = histDataIn[NCIRCLES-1].bin3;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+4) ] = histDataIn[NCIRCLES-1].bin4;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+5) ] = histDataIn[NCIRCLES-1].bin5;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+6) ] = histDataIn[NCIRCLES-1].bin6;
		Hist[NCIRCLES-1][ap_uint<3>(nBinID+7) ] = histDataIn[NCIRCLES-1].bin7;



		// Hist buffer now has the reordered descriptor. Output it on a single stream.

		for(int n = 0; n< NCIRCLES; n++)
		{
			for(int m = 0 ; m < 8 ; m++)
			{
				ap_uint<32>  DescOut = Hist[n][m];
				pDescriptorOut.write( DescOut);
			}
		}

	}


}







//========================================================================================================================//
//=================================================== SIFT DESCRIPTOR ====================================================//
//========================================================================================================================//


// SDSoC Wrapper for the HLS functions

#pragma SDS data access_pattern(pKeypoint:SEQUENTIAL,  pDescriptorOut:SEQUENTIAL, pMainOrientation:SEQUENTIAL)
#pragma SDS data copy( pKeypoint[0:MAX_KEYPOINTS], pDescriptorOut[0:MAX_KEYPOINTS*DESC_SIZE], pMainOrientation[0:MAX_KEYPOINTS])
#pragma SDS data data_mover(pKeypoint:AXIDMA_SIMPLE, pDescriptorOut:AXIDMA_SIMPLE)

#pragma SDS data zero_copy(  pGrad[0:WIDTH*HEIGHT])

void compute_descriptor(	Grad_t* pGrad,
							Keypoint_t* pKeypoint,
							ap_uint<8>* pMainOrientation,
							ap_uint<32>* pDescriptorOut)
{

#pragma HLS INLINE OFF
#pragma HLS DATAFLOW


	hls::stream< Keypoint_t > KeypointStream;

	hls::stream< ap_uint<8> > MainOrientationStream;

	// Histogram streams
	hls::stream< Hist_t > Hist0;
	hls::stream< Hist_t > Hist1;
	hls::stream< Hist_t > Hist2;
	hls::stream< Hist_t > Hist3;
	hls::stream< Hist_t > Hist4;
	hls::stream< Hist_t > Hist5;
	hls::stream< Hist_t > Hist6;
	hls::stream< Hist_t > Hist7;
	hls::stream< Hist_t > Hist8;

	hls::stream< ap_uint<32> > DescriptorOutStream;

// populate input image stream
	for(int i=0; i<MAX_KEYPOINTS;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE


		KeypointStream.write( pKeypoint[i] );
		MainOrientationStream.write(pMainOrientation[i]);

	}


// Calling HLS functions. They will be called simultaneously as they are in Dataflow Region


	compute_histograms_hls(pGrad, KeypointStream, Hist0, Hist1, Hist2, Hist3, Hist4, Hist5, Hist6, Hist7, Hist8);

	reorder_histograms_hls(MainOrientationStream, Hist0, Hist1, Hist2, Hist3, Hist4, Hist5, Hist6, Hist7, Hist8, DescriptorOutStream );



	for(int i=0; i<MAX_KEYPOINTS*DESC_SIZE;i++)
	{

		#pragma HLS PIPELINE
		#pragma HLS LOOP_FLATTEN off

		pDescriptorOut[i] = DescriptorOutStream.read();

	}

}

















//// ===================================== CAM1 BELOW THIS LINE ================================================================ ////







