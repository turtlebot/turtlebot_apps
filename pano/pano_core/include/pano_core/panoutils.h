#define _CRT_SECURE_NO_DEPRECATE
#ifndef PANOUTILS_H
#define PANOUTILS_H

#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include <exception>
#include <map>
#include <string>
#include <exception>
#include <typeinfo>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace pano {


//*****************************************************************************
// cv::Math utils
/** return the effect of R on a unit vector on z axis
  */
inline cv::Mat  getRv( const cv::Mat& R ) {
    cv::Mat v = cv::Mat::zeros(3, 1, cv::DataType<float>::type);
    v.at<float> (2, 0) = 1;
    cv::Mat Rv = R * v;
    return Rv;
}

/** \brief helper to sort a vector of distance index pairs
  */
inline bool compareRealToIndexPair(const std::pair<double,int>& lhs, const std::pair<double,int>& rhs){
    return lhs.first < rhs.first;
}

///** \brief allows each output of cv::Mat in cv::Matlab forcv::Mat to std::cout
// * use like
// @verbatim
// std::cout << mycv::Mat;
// @endverbatim
// */
//std::ostream & operator<<(std::ostream & out, const cv::Mat & mat);
//
//inline std::ostream & operator<<(std::ostream & out, const cv::Point3f & p){
//    out << "[ " << p.x << "," << p.y << "," << p.z << " ]";
//    return out;
//}
//
///** \brief write a cv::Matrix in csv forcv::Mat for cv::Matlab.
// This means that the rows are seperated by newlines and the
// columns by commas ....
// 331.413896619595,0,122.365880226491
// 0,249.320451610369,122.146722131871
// 0,0,1
//
// *	\param out output stream to write to
// *	\param cv::Mat write a cv::Mat to a csv
// */
//std::ostream & writeCSV(std::ostream & out, const cv::Mat & mat);

/** \brief	Saves a cv::Mat csv file for using in Matlab
 *  \param	Mat		The Mat.
 *  \param	name	The name.
 *  \param	append	true to append.
 **/
template <typename CSVSavable>
void saveMatCsv( const CSVSavable& mat, const std::string& name, bool append) {
    std::ofstream fs(name.c_str(), append ? std::fstream::app
            : std::fstream::out);
    writeCSV(fs, mat);
    fs.close();
}

typedef void (*MDoperator)(cv::InputArray, cv::InputArray, cv::OutputArray, double, int);

void mdImageByDoubleWeights(const cv::Mat& img, const cv::Mat& weights,
cv::Mat& output, MDoperator md,std::vector<cv::Mat>* cache_channels);

void mdImageByFloatWeights(const cv::Mat& img, const cv::Mat& weights,
cv::Mat& output, MDoperator md,std::vector<cv::Mat>* cache_channels);

inline void multiplyImageByFloatWeights(const cv::Mat& img, const cv::Mat& weights, cv::Mat& output,std::vector<cv::Mat>* cache_channels=0){
    mdImageByFloatWeights(img,weights,output,cv::multiply,cache_channels);
}

inline void divideImageByFloatWeights(const cv::Mat& img, const cv::Mat& weights,cv::Mat& output,std::vector<cv::Mat>* cache_channels = 0){
    mdImageByFloatWeights(img,weights,output,cv::divide,cache_channels);
}

inline void multiplyImageByDoubleWeights(const cv::Mat& img, const cv::Mat& weights, cv::Mat& output,std::vector<cv::Mat>* cache_channels=0){
    mdImageByDoubleWeights(img,weights,output,cv::multiply,cache_channels);
}

inline void divideImageByDoubleWeights(const cv::Mat& img, const cv::Mat& weights,cv::Mat& output,std::vector<cv::Mat>* cache_channels = 0){
    mdImageByDoubleWeights(img,weights,output,cv::divide,cache_channels);
}


inline void  rescaleFloatImage256( cv::Mat& img ) {

    cv::Mat flimage;
    img.convertTo(flimage, CV_32FC3);
    std::vector<cv::Mat> channels;
      split(flimage, channels);
      for (size_t k = 0; k < channels.size(); k++) {
        cv::Mat cc = channels[k];
        double fmin,fmax;
        cv::minMaxLoc( cc, &fmin, &fmax );
        if( fmax > 1.0 )
            fmax = 255.0 ;
        else
            fmax = 1.0;
        channels[k] = 255.0 * ( channels[k] / (fmax + 1e-9) );
      }

      merge(channels, flimage);
      flimage.convertTo( img, img.type() );
}

inline void  rescaleFloatImage1_shiftMin_divideMax( cv::Mat& img ) {

    cv::Mat flimage;
    img.convertTo(flimage, CV_32FC3);
    std::vector<cv::Mat> channels;
      split(flimage, channels);
      for (size_t k = 0; k < channels.size(); k++) {
        cv::Mat cc = channels[k];
        double fmin,fmax;
        cv::minMaxLoc( cc, &fmin, &fmax );
        channels[k] = channels[k] - fmin;
        channels[k] = ( channels[k] / (fmax + 1e-9) );
      }
      merge(channels, flimage);
      flimage.convertTo( img, img.type() );
}

inline void  rescaleFloatImage1( cv::Mat& img ) {

    cv::Mat flimage;
    img.convertTo(flimage, CV_32FC3);
    std::vector<cv::Mat> channels;
      split(flimage, channels);
      for (size_t k = 0; k < channels.size(); k++) {
        cv::Mat cc = channels[k];
        double fmin,fmax;
        cv::minMaxLoc( cc, &fmin, &fmax );
        if( fmax > 1.0 )
            fmax = 255.0 ;
        else
            fmax = 1.0;
        channels[k] = ( channels[k] / (fmax + 1e-9) );
      }
      merge(channels, flimage);
      flimage.convertTo( img, img.type() );
}


inline void  rescaleFloatImage1_clip( cv::Mat& img ) { // For Use with imshow !!! Not needed for imwrite

    cv::Mat flimage;
    img.convertTo(flimage, CV_32FC3);
    std::vector<cv::Mat> channels;
      split(flimage, channels);
      for (size_t k = 0; k < channels.size(); k++) {
        cv::Mat cc = channels[k];
        double fmin,fmax;
        cv::minMaxLoc( cc, &fmin, &fmax );
        if( fmax > 1.0 )
            fmax = 255.0 ;
        else
            fmax = 1.0;
        cc = ( cc / (fmax + 1e-9) );
        for( int i = 0; i < cc.rows; i++ ) {
            for( int j = 0; j < cc.cols; j++ ) {
                if( cc.at<float>(i,j) > 1.0 )
                    cc.at<float>(i,j)  = 1.0;
                else if( cc.at<float>(i,j) < 0.0 )
                    cc.at<float>(i,j)  = 0.0;
                else
                    cc.at<float>(i,j)  = 1.0 * cc.at<float>(i,j);
            }
        }

      }
      merge(channels, flimage);
      flimage.convertTo( img, img.type() );
}

inline void  rescaleFloatImage256_clip( cv::Mat& img ) { // For Use with imwrite! must scale to 256 before writing or converting to uint8C3

    cv::Mat flimage;
    img.convertTo(flimage, CV_32FC3);
    std::vector<cv::Mat> channels;
      split(flimage, channels);
      for (size_t k = 0; k < channels.size(); k++) {
        cv::Mat cc = channels[k];
        double fmin,fmax;
        cv::minMaxLoc( cc, &fmin, &fmax );
        if( fmax > 1.0 )
            fmax = 255.0 ;
        else
            fmax = 1.0;
        cc = ( cc / (fmax + 1e-9) );
        for( int i = 0; i < cc.rows; i++ ) {
            for( int j = 0; j < cc.cols; j++ ) {
                if( cc.at<float>(i,j) > 1.0 )
                    cc.at<float>(i,j)  = 255.0;
                else if( cc.at<float>(i,j) < 0.0 )
                    cc.at<float>(i,j)  = 0.0;
                else
                    cc.at<float>(i,j)  = 255.0 * cc.at<float>(i,j);
            }
        }

      }
      merge(channels, flimage);
      flimage.convertTo( img, img.type() );
}

}
#endif // PANOUTILS_H
