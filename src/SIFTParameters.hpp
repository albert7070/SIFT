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

#define MAX_KEYPOINTS 1000				// Maximum number of keypoints. This has to be a fixed number as FPGA cannot dynamically allocate memmory

#define DESC_SIZE 72    				// Descriptor size is the total number of elements in each descriptor

#define NCIRCLES 9						// Number of Circular sub-regions

#define NDIRECTIONS 8					// Number of directions / number of bins in the histogram

#define MASK_SIZE 25					// Circular Mask Size

#define WINDOW_SIZE 71

#define RADIUS WINDOW_SIZE/2;

#define WINDOW_OFFS WINDOW_SIZE/2


typedef struct
{
	ap_uint<16> x;		// Col
	ap_uint<16> y;		// Row

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



