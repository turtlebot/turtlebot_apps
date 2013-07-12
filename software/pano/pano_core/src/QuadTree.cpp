//#include <pano_core/QuadTree.h>
//#include <pano_core/feature_utils.h>
//#include <list>
//
//using namespace cv;
//using namespace std;
//namespace pano{
//typedef std::list<int> Idxs;
//typedef cv::Rect_<float> RectF;
//
//struct QuadTreeNode
//{
//private:
//  friend class QuadTree;
//
//  typedef Idxs::const_iterator Idxs_cit;
//  typedef Idxs::iterator Idxs_it;
//
//  const std::vector<cv::Point2f> * mother_;
//  cv::Rect_<float> region_;
//  cv::Point2f center_;
//  cv::Ptr<QuadTreeNode> q1_, q2_, q3_, q4_;
//  Idxs idxs_;
//  bool empty_;
//  QuadTreeNode() :
//    mother_(0),empty_(true)
//  {
//  }
//  QuadTreeNode(const std::vector<cv::Point2f> * mother, const RectF& region) :
//    mother_(mother), region_(region), center_(getCenter(region)),empty_(true)
//  {
//
//  }
//  QuadTreeNode(const std::vector<cv::Point2f> * mother, const Idxs& idxs, const RectF& region) :
//    mother_(mother), region_(region), center_(getCenter(region)), idxs_(idxs),empty_(true)
//  {
//    propigatePoints();
//  }
//
//  float dist(const cv::Point2f& p) const
//  {
//    Point2f dv = center_ - p;
//    return sqrt(dv.dot(dv));
//  }
//
//  bool empty() const {return empty_;}
//
//  void radiusSearch(_RadiusPoint<Point2f> rp, vector<int>& result) const
//  {
//    RectF p_region(region_.tl() - Point2f(rp.r, rp.r), Size(region_.width + rp.r, region_.height + rp.r));
//    if (!p_region.contains(rp.pt2))
//      return;
//
//    Idxs_cit it = idxs_.begin();
//    while (idxs_.end() != it)
//    {
//      int i = *it;
//      if (rp.cp((*mother_)[i]))
//      {
//        result.push_back(i);
//      }
//      ++it;
//    }
//
//    if(!q1_.empty())
//    q1_->radiusSearch(rp, result);
//    if(!q2_.empty())
//    q2_->radiusSearch(rp, result);
//    if(!q3_.empty())
//    q3_->radiusSearch(rp, result);
//    if(!q4_.empty())
//    q4_->radiusSearch(rp, result);
//
//  }
//
//  void newChildren()
//  {
//    q1_ = new QuadTreeNode(mother_, getRegion(1));
//    q2_ = new QuadTreeNode(mother_, getRegion(2));
//    q3_ = new QuadTreeNode(mother_, getRegion(3));
//    q4_ = new QuadTreeNode(mother_, getRegion(4));
//  }
//  void propigatePoints()
//  {
//    empty_ = false;
//
//    if (idxs_.size() <= 4)
//      return;
//
//    newChildren();
//
//    while (idxs_.size())
//    {
//      QuadTreeNode * node = closest((*mother_)[idxs_.back()]);
//      node->idxs_.push_back(idxs_.back());
//      idxs_.pop_back();
//    }
//
//    q1_->propigatePoints();
//    q2_->propigatePoints();
//    q3_->propigatePoints();
//    q4_->propigatePoints();
//
//  }
//
//  QuadTreeNode * closest(const Point2f& p)
//  {
//    float d1 = q1_->dist(p), d2 = q2_->dist(p), d3 = q3_->dist(p), d4 = q4_->dist(p);
//    //float min12 = min(d1, d2);
//    float min34 = min(d3, d4);
//    if (d1 < min(d2, min34))
//    {
//      return &(*q1_);
//    }
//    if (d2 < min34)
//    {
//      return &(*q2_);
//    }
//    if (d3 < d4)
//    {
//      return &(*q3_);
//    }
//    return &(*q4_);
//  }
//
//  RectF getRegion(int q)
//  {
//    RectF r(center_.x, center_.y, region_.width / 2, region_.height / 2);
//    switch (q)
//    {
//      case 1:
//        r.y -= r.height;
//        break;
//      case 2:
//        r.x -= r.width;
//        r.y -= r.height;
//        break;
//      case 3:
//        r.x -= r.width;
//        break;
//      case 4:
//        //r.x = center_.x
//        //r.y = center_.y
//        break;
//    }
//    return r;
//  }
//
//  static cv::Point2f getCenter(const RectF region)
//  {
//    return cv::Point2f(region.x + region.width / 2, region.y + region.height / 2);
//  }
//};
//
//
//QuadTree::QuadTree():root_(0)
//
//{
//
//}
//QuadTree::~QuadTree(){
//  delete root_;
//}
//QuadTree::QuadTree(const QuadTree& rhs) :
//   root_(0)
// {
//   addPoints(rhs.points_, rhs.size_);
// }
// void QuadTree::operator=(const QuadTree& rhs)
// {
//   addPoints(rhs.points_, rhs.size_);
// }
//
//void QuadTree::addPoints(const std::vector<cv::Point2f>& points, cv::Size size)
//{
//
//  size_ = size;
//  points_ = points;
//  std::list<int> idxs;
//  for (int i = 0; i < (int)points_.size(); i++)
//  {
//    idxs.push_back(i);
//  }
//
//  delete root_;
//  root_ = new QuadTreeNode(&points_, idxs, RectF(0, 0, size.width, size.height));
//
//}
//void QuadTree::radiusSearch(const cv::Point2f& pt, float radius, std::vector<int>& result) const
//{
//  _RadiusPoint<Point2f> rp( radius, pt);
//  result.clear();
//  if(root_ == 0)return;
//
//  result.reserve(20);
//  root_->radiusSearch(rp, result);
//}
//void QuadTree::radiusSearch(const vector< cv::Point2f >& pts, float radius, std::vector<vector<int> >& result) const
//{
//
//  result.clear();
//  if(root_ == 0)return;
//
//  result.resize(pts.size());
//  for(size_t i = 0; i <  pts.size(); ++i){
//    radiusSearch(pts[i],radius,result[i]);
//  }
//}
//}
