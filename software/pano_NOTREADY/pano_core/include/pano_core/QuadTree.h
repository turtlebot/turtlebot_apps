//#ifndef PANO_QUADTREE_H_
//#define PANO_QUADTREE_H_
//
//#include <opencv2/core/core.hpp>
//#include <vector>
//
//namespace pano
//{
//struct QuadTreeNode;
//class QuadTree
//{
//public:
//  QuadTree();
//  ~QuadTree();
//  QuadTree(const QuadTree& rhs);
//  void operator=(const QuadTree& rhs);
//
//  void addPoints(const std::vector<cv::Point2f>& points, cv::Size size);
//  void radiusSearch(const cv::Point2f& pt, float radius, std::vector<int>& result) const;
//  void radiusSearch(const std::vector<cv::Point2f>& pts, float radius, std::vector<std::vector<int> >& result) const;
//
//private:
//  std::vector<cv::Point2f> points_;
//  cv::Size size_;
//  QuadTreeNode* root_;
//};
//
//}//namespace pano
//#endif //PANO_QUADTREE_H_
