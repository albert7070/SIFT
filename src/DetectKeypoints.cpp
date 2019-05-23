
#include "DetectKeypoints.hpp"

#include "GaussianBlur.hpp"




// ===================================== Wrapper for SDSoC ==================================== //


#pragma SDS data access_pattern(pSrcImage:SEQUENTIAL, pDupGaussian2:SEQUENTIAL)
#pragma SDS data copy(pSrcImage[0:WIDTH*HEIGHT], pDupGaussian2[0:WIDTH*HEIGHT])

#pragma SDS data access_pattern(pGaussian1:SEQUENTIAL, pGaussian2:SEQUENTIAL,  pGaussian3:SEQUENTIAL)
#pragma SDS data copy(pGaussian1[0:WIDTH*HEIGHT], pGaussian2[0:WIDTH*HEIGHT], pGaussian3[0:WIDTH*HEIGHT])

#pragma SDS data data_mover(pSrcImage:AXIDMA_SIMPLE)
#pragma SDS data data_mover(pGaussian1:AXIDMA_SIMPLE, pGaussian2:AXIDMA_SIMPLE, pGaussian3:AXIDMA_SIMPLE, pDupGaussian2:AXIDMA_SIMPLE)

void sift_detect(	ap_uint<8>* pSrcImage,
					ap_uint<8>* pGaussian1,
					ap_uint<8>* pGaussian2,
					ap_uint<8>* pGaussian3,
					ap_uint<8>* pDupGaussian2
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





// populate input image stream
	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Pixel_t PixelIn = pSrcImage[i];

		srcImageStream.write( PixelIn );

	}


// Calling HLS functions

	GaussianBlurHLS(srcImageStream, GaussianStream1, GaussianStream2, GaussianStream3, DupGaussianStream2, DupSrcImageStream);



	for(int i=0; i<HEIGHT*WIDTH;i++)
	{
		#pragma HLS LOOP_FLATTEN off
		#pragma HLS PIPELINE

		Pixel_t DupGaussOut2 = DupGaussianStream2.read();

		Pixel_t GaussOut1 = GaussianStream1.read();
		Pixel_t GaussOut2 = GaussianStream2.read();
		Pixel_t GaussOut3 = GaussianStream3.read();

		pDupGaussian2[i] = DupGaussOut2;

		pGaussian1[i] = GaussOut1;
		pGaussian2[i] = GaussOut2;
		pGaussian3[i] = GaussOut3;

	}


}
