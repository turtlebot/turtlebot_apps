#ifndef PANO_INTERFACES_H_H
#define PANO_INTERFACES_H_H

#include <opencv2/core/core.hpp>

namespace pano{

class drawable{
public:
  virtual ~drawable(){}
  /**
   * \brief interface for drawing
   * \param out the output mat to draw to
   * \param flags optional drawing flags, default is 0
   */
  virtual void draw(cv::Mat* out, int flags = 0) = 0;
};

class serializable{
public:
  virtual ~serializable(){}
  virtual int version() const = 0;
  virtual void serialize(cv::FileStorage& fs) const = 0;
  virtual void deserialize(const cv::FileNode& fn) = 0;
};

template <typename Base>
class Copier{
public:
  virtual ~Copier(){}
  virtual Base* make() const = 0;
  virtual Base* clone(const Base& n) const = 0;
};

template <typename Sub, typename Base>
class SCopier:public Copier<Base> {
public:
  virtual ~SCopier(){}
  virtual Base* make() const{
    return new Sub();
  }
  virtual Base* clone(const Base& sub) const{
    return new Sub(dynamic_cast<const Sub&>(sub));
  }
};


}
#endif
