/*
 * callbacks.h
 *
 *  Created on: Oct 31, 2010
 *      Author: ethan
 */

#ifndef PANO_CALLBACKS_H_
#define PANO_CALLBACKS_H_

#include <map>
#include <opencv2/core/core.hpp>

namespace pano{
struct Callback{
  virtual ~Callback(){}
  template<typename T>
  T& cast(){return dynamic_cast<T&>(*this);}
};

template<typename Data>
struct DCallback : public Callback{
  virtual ~DCallback(){}
  virtual void operator()(const Data& data) = 0;
};

template<typename Data, typename Function>
struct GCallback:public DCallback<Data>{
  Function f_;
  GCallback(const Function& f):f_(f){}
  virtual ~GCallback(){}
  virtual void operator()(const Data& data){
    f_(data);
  }
};

class CallbackEngine{
public:
  template<typename Data, typename Function>
  void addCallback(int idx, const Function& f){
    cbs_[idx] = cv::Ptr<Callback>(new GCallback<Data,Function>(f));
  }
  template<typename Data>
  void callBack(const Data& data, int idx){
    if(cbs_.count(idx)){
      DCallback<Data>& dcb = cbs_[idx]->cast<DCallback<Data> >();
      dcb(data);
    }
  }
  void clearCallback(int idx){
	  cbs_.erase(idx);
  }
private:
  std::map<int,cv::Ptr<Callback> > cbs_;
};
}

#endif /* PANO_CALLBACKS_H_ */
