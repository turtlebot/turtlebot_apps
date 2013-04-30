#ifndef IMAGE_ATOM_H_
#define IMAGE_ATOM_H_

#include <pano_core/Images.h>
#include <pano_core/Features.h>
#include <pano_core/Camera.h>
#include <pano_core/Extrinsics.h>
#include <pano_core/pano_interfaces.h>

namespace pano
{
class ImageAtom : public drawable, public serializable
{
public:

  ImageAtom();
  virtual ~ImageAtom();

  ImageAtom(const Camera& camera, const Images& images);

  void detect(const cv::FeatureDetector& detector);

  template<typename DescriptorMatcherT>
    void extract(const cv::DescriptorExtractor& extractor, const DescriptorMatcherT& matcher)
    {
      features_.extract(extractor, images_.grey(), matcher);
    }
//  //template<cv::BruteForceMatcher<Hamming> >
//    void extract(const cv::BriefDescriptorExtractor& extractor, const cv::BruteForceMatcher<cv::Hamming>& matcher)
//    {
//      features_.extract(extractor, images_.sum(), matcher);
//    }
  void match(const ImageAtom& atom, std::vector<cv::DMatch>& matches,const cv::Mat& H = cv::Mat(), float uncertainty = 25) const;

  void descriptorMatchMask(const ImageAtom& atom, cv::Mat& mask,const cv::Mat& H = cv::Mat(), float uncertainty = 25) const;

  void setUid(int id);
  int uid() const
  {
    return uid_;
  }

  Images& images()
  {
    return images_;
  }
  Features& features()
  {
    return features_;
  }
  Camera& camera()
  {
    return camera_;
  }
  Extrinsics & extrinsics()
  {
    return extrinsics_;
  }
  const Images& images() const
  {
    return images_;
  }
  const Features& features() const
  {
    return features_;
  }
  const Camera& camera() const
  {
    return camera_;
  }
  const Extrinsics & extrinsics() const
  {
    return extrinsics_;
  }

  cv::Mat undistortPoints();
  /*
   * drawable functions
   */
  virtual void draw(cv::Mat* out, int flags = 0);
  /*
   * serializable functions
   */
  virtual int version() const
  {
    return 0;
  }

  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);

  void flush()
  {
    images_.clear();
  }

  ImageAtom* clone() const
  {
    ImageAtom* atom = new ImageAtom(*this);
    atom->images_ = images_;
   // atom->images_.load(images_.src(), images_.fname(), images_.path());
    return atom;
  }

  /** warning this is dangerous - don't hold onto the smart pointer after this goes out of scope or gets deleted
    */
  cv::Ptr<ImageAtom> ptrToSelf(){
      cv::Ptr<ImageAtom> ptr(this);
      ptr.addref();
      return ptr;
    }
  enum
  {
    DRAW_FEATURES = 1
  };

private:

  Images images_;
  Features features_;
  Camera camera_;
  Extrinsics extrinsics_;
  int uid_;
};
}
#endif //IMAGE_ATOM_H_
