#pragma once

#include "sds_lib.h"
#include <ap_int.h>
#include <ap_fixed.h>
#include <stdint.h>
#include <assert.h>
#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"






#define WIDTH  1600
#define HEIGHT 1300

#define N_SCALES 3						// Number of scales used in the SIFT detector

#define MAX_KEYPOINTS 10000				// Maximum number of keypoints. This has to be a fixed number as FPGA cannot dynamically allocate memmory

#define MAX_FINAL_KEYPOINTS 1000

#define DESC_SIZE 72    				// Descriptor size is the total number of elements in each descriptor

#define NCIRCLES 9						// Number of Circular sub-regions

#define NDIRECTIONS 8					// Number of directions / number of bins in the histogram

#define MASK_SIZE 25					// Circular Mask Size

#define WINDOW_SIZE 71

#define RADIUS WINDOW_SIZE/2;

#define WINDOW_OFFS WINDOW_SIZE/2





#define MASK_SIZE 25					// Circular Mask Size for cam0

#define MASK_SIZE_1 29					// Circular Mask Size for cam1

#define WINDOW_SIZE 71
#define RADIUS WINDOW_SIZE/2;
#define WINDOW_OFFS WINDOW_SIZE/2


#define DESC_WIN_SIZE_1 81
#define DESC_RADIUS_1 DESC_WIN_SIZE_1/2;
#define DESC_WIN_OFFS_1 DESC_WIN_SIZE_1/2





#define FILTER_SIZE1 5
#define FILTER_OFFS1 FILTER_SIZE1/2


#define FILTER_SIZE2 11
#define FILTER_OFFS2 FILTER_SIZE2/2


#define FILTER_SIZE3 19
#define FILTER_OFFS3 FILTER_SIZE3/2

typedef ap_fixed<16,1,AP_RND_ZERO> Mask_t ;

#define CO_SIZE 3
#define CO_OFFS CO_SIZE/2



typedef struct
{
	ap_uint<16> x;				// Col
	ap_uint<16> y;				// Row
	ap_uint<32> contrast; 		// Contrast

} Keypoint_t;


typedef struct
{
	ap_uint<16> angGrad ;
	ap_uint<16> magGrad ;
}Grad_t;


typedef ap_fixed<32,17, AP_RND_ZERO> Fixed_t ;

typedef ap_uint<8> Pixel_t ;


//
//typedef ap_uint<256> HistPk_t ;

typedef ap_uint<32> Desc_t ;


// HLS doesn't seem to comprehend arrays inside structures very well
// Thus for maximizing the parallel operations making a structure with histogram bins as member variables.
typedef struct
{
	ap_uint<32> bin0;
	ap_uint<32> bin1;
	ap_uint<32> bin2;
	ap_uint<32> bin3;
	ap_uint<32> bin4;
	ap_uint<32> bin5;
	ap_uint<32> bin6;
	ap_uint<32> bin7;

}Hist_t;


typedef struct
{
	ap_uint<32> distance;		// This is float in opencv. But we are avoiding any floating pt computations in FPGA
	ap_int<16>  querryIdx;
	ap_int<16>  trainIdx;

}DMatch_t;




