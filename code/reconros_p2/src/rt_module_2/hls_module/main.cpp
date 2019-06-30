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

void process_module(uint32_t ram[31], stream_in_t& inStream, stream_out_t& outStream) {
#pragma HLS INTERFACE bram port=ram
#pragma HLS RESOURCE variable=ram core=RAM_1P_BRAM
#pragma HLS INTERFACE axis port=outStream
#pragma HLS INTERFACE axis port=inStream

	data_in_t dataIn;

	bool startFrame = false;
	bool endLine = false;
	
	//array to count "white" pixels in each of the 15 columns (partitioned to make pipelining easier)
	uint32_t white_pixels[15] = {};
	#pragma HLS ARRAY_PARTITION variable=white_pixels complete dim=1
	
    uint32_t column_pixels = 1;
    uint32_t column = 0;
    int line = 0;
	
	//read in brightness threshold from BRAM
	uint32_t threshold = ram[0];
	
	//read in column width for each column
	uint32_t columns[15] = {};
    for(int i = 0; i<15; i++)
    {
		columns[i] = ram[1+i];
    }

	//loop over all pixels of one frame
	while(line < 720)
	{
		#pragma HLS pipeline
			
		data_out_t dataOut;
		
		//read next pixel
		inStream.read(dataIn);
		startFrame = dataIn.user;
		endLine = dataIn.last;

		if(startFrame) line = 0;
		if(endLine) line++;

		//count pixel as white if each color channel is above threshold
		if (((dataIn.data & 0xff0000) >> 16)>threshold && ((dataIn.data & 0x00ff00) >> 8) > threshold && (dataIn.data & 0x0000ff) > threshold)
		{
			white_pixels[column]++;
			dataOut.data = dataIn.data; //output the original pixel if it is detected as "white"
		}
		else
		{
			dataOut.data = 0x000000; //output a black pixel otherwise
		}

		column_pixels++;
		
		if(endLine)
		{
			//reset counters at the end of each line
			column=0;
			column_pixels=0;
		} 
		else if (column_pixels>columns[column])
		{
			//switch to next column based on configured width
			column++;
		}
		
		//output stream
		dataOut.strb = -1;
		dataOut.keep = -1;
		dataOut.id = 0;
		dataOut.dest = 0;

		dataOut.user = dataIn.user;
		dataOut.last = dataIn.last;
		//dataOut.data = dataIn.data;

		outStream.write(dataOut);

	}

	//write result to BRAM
	for (int i=0; i<15; i++)
	{
		ram[16+i] = white_pixels[i];
	}
	
	return;
}
