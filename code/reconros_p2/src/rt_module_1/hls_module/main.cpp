#include <stdint.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#include "string.h"

#define DATA_WIDTH_IN 	24
#define DATA_WIDTH_OUT 	24

typedef ap_axiu<DATA_WIDTH_IN,1,1,1> data_in_t; //defines AXIS signals
typedef hls::stream<data_in_t> stream_in_t;

typedef ap_axiu<DATA_WIDTH_OUT,1,1,1> data_out_t; //defines AXIS signals
typedef hls::stream<data_out_t> stream_out_t;

using namespace std;

void process_module(uint32_t ram[3*8], stream_in_t& inStream, stream_out_t& outStream) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM
#pragma HLS INTERFACE axis port=outStream
#pragma HLS INTERFACE axis port=inStream

	data_in_t dataIn;

	bool startFrame = false;
	bool endLine = false;
	
	//arrays to hold 8 bin counters for each color channel
	uint32_t histogram_r[8] = {};
	uint32_t histogram_g[8] = {};
	uint32_t histogram_b[8] = {};
	
	int bin_r = 0;
	int bin_g = 0;
	int bin_b = 0;
	
	//loop over pixels of one frame
	while(!startFrame)
	{
		#pragma HLS pipeline
			
		//read next pixel
		inStream.read(dataIn);
		startFrame = dataIn.user;
		endLine = dataIn.last;

		//count bins
		bin_r = ((dataIn.data & 0xff0000) >> 16) / 8;
		histogram_r[bin_r] ++;
		
		bin_b = ((dataIn.data & 0x00ff00) >>  8) / 8;
		histogram_b[bin_b] ++;
		
		bin_g = ((dataIn.data & 0x0000ff) >>  0) / 8;
		histogram_g[bin_g] ++;
		
		//output video stream (pass unaltered input stream through)
		data_out_t dataOut;
		
		dataOut.strb = -1;
		dataOut.keep = -1;
		dataOut.id = 0;
		dataOut.dest = 0;

		dataOut.user = dataIn.user;
		dataOut.last = dataIn.last;
		dataOut.data = dataIn.data;

		outStream.write(dataOut);
	}

	//write result to BRAM
	for (int i=0; i<8; i++)
	{
		ram[0+i] = histogram_r[i];
		ram[8+i] = histogram_g[i];
		ram[16+i] = histogram_b[i];
	}
	
	return;
}
