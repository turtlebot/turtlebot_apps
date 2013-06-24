#ifndef PANO_PROJECTOR_H_
#define PANO_PROJECTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace pano {


    class Projector {
      friend class SparseProjector;
        cv::Mat K, Kinv;
        cv::Size outimage_size,input_image_sz;
        cv::Mat spherical_coords;
        cv::Mat remap1, remap2;
        cv::Mat R;
        std::vector<cv::Mat> working_mats;
    public:
        Projector(){}
        /** \brief
         * \param output_size size of final spherical projection image width = 2 pi height = pi
         * 			center pixel is 0,0 essentially, with x going from -pi to pi and y going from
         * 			-pi/2 to pi/2 ???TODO:define direction of y
         * */
        Projector(const cv::Size& output_size, float theta_range = 2 * CV_PI, float phi_range = CV_PI);


        /** \brief Sets up the projector to remap based on the given R
          * and K.  Call this before calling any of the member project functions
          */
        void setSRandK(const cv::Size& inputsz,const cv::Mat& R, const cv::Mat& K);

        void projectMat(const cv::Mat& m, cv::Mat& outimage, int filltype = cv::BORDER_TRANSPARENT,const cv::Scalar& value = cv::Scalar());

        ~Projector(void);

        /*** static members *****/

    /** \brief
     * \param sphere_size
     * \param theta_range the longitudinal range of the spherical cooridinates produced
     * \param phi_range the latitudinal range of the coords produced
     * \return a Mat(sphere_size, cv::DataType<cv::Point3d>::type) where every point in the mat lies on the unit sphere, in the
     * 			lat long ranges specified. So to go from spherical coorinate (theta,phi) to (x,y,z) just look in the mat.at(phi,theta)
     * 			<code>
     * 			cv::Point3d& point = sphere.at<cv::Point3d> (phi, theta)
     * 			</code>
     */
        static cv::Mat createSphericalCoords(const cv::Size& sphere_size = cv::Size(200,200),
                                             float theta_range = 2 * CV_PI, float phi_range = CV_PI);

        static void createSphericalCoords(const cv::Size& sphere_size, float theta_0,float theta_1, float phi_0,float phi_1, cv::Mat& spherical_coords);
        /** Like createSphereicalCoords but with row of 1's appended so
      * 4x4 matrices can operate on it
      */
        static cv::Mat createHomogSphrCoords(const cv::Size& sphere_size = cv::Size(200,200),
                                             float theta_range = 2 * CV_PI, float phi_range = CV_PI);

        static void getSphereMask(const cv::Mat& onezies, const cv::Mat& remap1,
                                  const cv::Mat&remap2, cv::Mat& mask);

        static void projectImage(const cv::Mat& image, const cv::Mat& remap1,
                                 const cv::Mat&remap2, cv::Mat & outputimage,int filltype = cv::BORDER_TRANSPARENT,
                                 const cv::Scalar& border_value = cv::Scalar());

        /** create a remap matrix that is the size of the output image
     *
     * \param K camera calibration matrix
     * \param R the rotation to calc the remap matrix  - identity means the center pixel of the output image,
     * 			or the vector (0,0,1)
     * \param remap1 out parameter, that will hold a matrix of size given by the projector of outimage
     * 			see the opencv documentation for remap
     * 			http://opencv.willowgarage.com/documentation/cpp/geometric_image_transformations.html?highlight=remap#remap
         *\param remap2 out parameter, these are both necessary to pass to the project image function.  the remap1 and remap2 are generated
     *			using the cv::convertMaps function, for fixed point math, supposedly faster
     * 	intended use of the remapMat matrix, this will leave outliers alone(transparent)
     *  cv::remap(img, outputimge, remapMat, cv::Mat(), cv::INTER_LINEAR);
     *
     *  to get a
     */
        static void getSphereRMap(const cv::Mat& K,
                                  const cv::Mat& R, cv::Mat& remap1, cv::Mat&remap2,
                                  const cv::Mat& spherical_points,
                                  const cv::Size& output_size);

        static void getSphereRMapMask(const cv::Mat& K,
                                  const cv::Mat& R, cv::Mat& remap, cv::Mat& mask,
                                  const cv::Mat& spherical_points,cv::Mat& tm);

        /** create forward and backward maps to send and image described by
      * calibration matrix K into spherical coords. Image is treated as lying
      * on the sphere with flat distribution of rads per pixel. G is 4x4 matrix.
      */
        static void getSphereGMap(const cv::Mat& K,
                                  const cv::Mat& G, cv::Mat& remap1, cv::Mat&remap2,
                                  const cv::Mat& homog_sphr_coords,
                                  const cv::Size& output_size);

    private:
        static void getSphereRMap(const cv::Mat& K,
                                  const cv::Mat& R, cv::Mat& remap1, cv::Mat&remap2,
                                  const cv::Mat& spherical_points,
                                  const cv::Size& output_size,
                                  std::vector<cv::Mat>&
                                  working_mats);

        static void getSphereGMap(const cv::Mat& K,
                                  const cv::Mat& G, cv::Mat& remap1, cv::Mat&remap2,
                                  const cv::Mat& homog_sphr_coords,
                                  const cv::Size& output_size,
                                  std::vector<cv::Mat>&
                                  working_mats);


    };


    class SparseProjector{
    public:
      SparseProjector(){}
      SparseProjector(const cv::Size& output_size, const cv::Size& n_grids);
      /** \brief Sets up the projector to remap based on the given R
       * and K.  Call this before calling any of the member project functions
       */
      void setSRandK(const cv::Size& inputsz,const cv::Mat& R, const cv::Mat& K, std::vector<int>& roi_ids);
      void projectMat(int roi_id, const cv::Mat& m, cv::Mat& outimage, int filltype = cv::BORDER_TRANSPARENT,const cv::Scalar& value = cv::Scalar());
      void setWorkingRoi(int id);
      cv::Rect getRoi(int id) const{
        return rois_[id];
      }
    private:

      cv::Size output_size_;
      cv::Mat spherical_coords_small_;
      cv::Size grids_;
      std::vector<cv::Rect> rois_;
      std::vector<cv::Rect> small_rois_;
      cv::Mat remap_,mask_,spherical_coords_,tm_,remap1_,remap2_;
      int workingid_;
      cv::Mat R_,K_;

    };
}
#endif
