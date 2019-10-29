
#include "SIFTParameters.hpp"



#pragma SDS data access_pattern(pKeypoint:SEQUENTIAL,  pMainOrientation:SEQUENTIAL)
#pragma SDS data copy( pKeypoint[0:MAX_FINAL_KEYPOINTS], pMainOrientation[0:MAX_FINAL_KEYPOINTS])
#pragma SDS data data_mover(pKeypoint:AXIDMA_SIMPLE)

#pragma SDS data zero_copy(  pGrad[0:WIDTH*HEIGHT])

void compute_main_orientation(	Grad_t* pGrad,
								Keypoint_t* pKeypoint,
								ap_uint<8>* pMainOrientation);

ap_uint<8> ComputeMainOrientation(int Hist[9]);



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
							);




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
							hls::stream< ap_uint<32> > &pDescriptorOut);






#pragma SDS data access_pattern(pKeypoint:SEQUENTIAL,  pDescriptorOut:SEQUENTIAL, pMainOrientation:SEQUENTIAL)
#pragma SDS data copy( pKeypoint[0:MAX_FINAL_KEYPOINTS], pDescriptorOut[0:MAX_FINAL_KEYPOINTS*DESC_SIZE], pMainOrientation[0:MAX_FINAL_KEYPOINTS])
#pragma SDS data data_mover(pKeypoint:AXIDMA_SIMPLE, pDescriptorOut:AXIDMA_SIMPLE)

#pragma SDS data zero_copy(  pGrad[0:WIDTH*HEIGHT])

void compute_descriptor(	Grad_t* pGrad,
							Keypoint_t* pKeypoint,
							ap_uint<8>* pMainOrientation,
							ap_uint<32>* pDescriptorOut);








//================================== For Cam1 ======================================//


