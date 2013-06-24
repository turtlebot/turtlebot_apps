
#include "pano_core/panoutils.h"

namespace pano{

void mdImageByFloatWeights(const cv::Mat& img, const cv::Mat& weights, cv::Mat& output,
		MDoperator md, std::vector<cv::Mat>* cache_channels) {

	std::vector<cv::Mat> _tchannels(3);
	if(cache_channels == NULL)  /* cache the channels if possible to avoid re-mallocs */
		cache_channels = &_tchannels;
	std::vector<cv::Mat>& channels = *cache_channels;


	if( img.type () != CV_32FC3 || weights.type () != CV_32FC1 )
	{ /* force the input to be float. otherwise, weird artifacts can happen!
               calling func is responsible for scaling / saturation. */
		throw "nonsense, must have float type!" ;
	}

	cv::split(img, channels);
	md(channels[0], weights, channels[0], 1, -1);
	md(channels[1], weights, channels[1], 1, -1);
	md(channels[2], weights, channels[2], 1, -1);
	cv::merge(channels,output);

}


void mdImageByDoubleWeights(const cv::Mat& img, const cv::Mat& weights, cv::Mat& output,
		MDoperator md, std::vector<cv::Mat>* cache_channels) {

	std::vector<cv::Mat> _tchannels(3);
	if(cache_channels == NULL)  /* cache the channels if possible to avoid re-mallocs */
		cache_channels = &_tchannels;
	std::vector<cv::Mat>& channels = *cache_channels;


	if( img.type () != CV_64FC3 || weights.type () != CV_64FC1 )
	{ /* force the input to be double. otherwise, weird artifacts can happen!
               calling func is responsible for scaling / saturation. */
		throw "nonsense, must have float type!" ;
	}

	cv::split(img, channels);
	md(channels[0], weights, channels[0], 1, -1);
	md(channels[1], weights, channels[1], 1, -1);
	md(channels[2], weights, channels[2], 1, -1);
	cv::merge(channels,output);

}


}//namespace pano

