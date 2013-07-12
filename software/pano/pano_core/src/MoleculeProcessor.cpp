/*
 * MoleculeProcessor.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#include "pano_core/MoleculeProcessor.h"
#include "pano_core/ModelFitter.h"
#include "pano_core/ImageAtom.h"
#include "pano_core/PairNode.h"
#include "pano_core/panoutils.h"
#include "pano_core/feature_utils.h"

#include <iostream>
#include <algorithm>
#include <iomanip>
#include <map>

using namespace cv;
using namespace std;
namespace pano
{

const std::string MoleculeProcessor::SHOW_PAIRS = "SHOW_PAIRS";
const std::string MoleculeProcessor::GRAPHVIZ_FILENAME = "pairgraph.viz";
const std::string MoleculeProcessor::VERBOSE_GRAPHVIZ_FILENAME = "pairgraph.verbose.viz";

MoleculeProcessor::MoleculeProcessor()
{
}

AtomPair MoleculeProcessor::matchwithFitter(Ptr<ImageAtom> prior, Ptr<ImageAtom> query, ModelFitter& modelfitter)
{
  CV_Assert(query != prior)
    ;

  std::vector<DMatch> matches;
  prior->match(*query, matches, Mat(), 10);
  AtomPair pair(prior, query, matches);
  modelfitter.fit(pair);
  return pair;
}

std::list<AtomPair> MoleculeProcessor::queryWithAtom(const ImageMolecule& molecule, Ptr<ImageAtom> query, Ptr<
    ModelFitter> modelfitter, float angle_thresh)
{
  float min_err = 10.0e8;
  std::list<AtomPair> pairsFound;
  if (modelfitter.empty())
  {
    cerr << "no fitter passed!" << endl;
    return pairsFound;
  }
  const set<Ptr<ImageAtom> >& atoms = molecule.getAtoms();
  if (0 == atoms.size())
  { // return empty list if no pairs to make
    cerr << "no atoms" << endl;
    return pairsFound;
  }
  if (atoms.count(query) > 0)
  { //handle if the atom is here already
    cerr << "ImageMolecule: already contains the atom passed! \n";
    return pairsFound;
  }
  else
  {
    Extrinsics& est_ext = query->extrinsics();
    set<Ptr<ImageAtom> >::const_iterator ait = atoms.begin();
    for (; ait != atoms.end(); ++ait)
    {
      Ptr<ImageAtom> prior = *ait;
      // first: if atom1 and atom2 both have trinsics set,
      // are they within the valid range of angle_thresh?
      if (angle_thresh > 0.0 && prior->extrinsics().flag(Extrinsics::ESTIMATED) && est_ext.flag(Extrinsics::ESTIMATED))
      {
        bool bAreCloseEnough = angularDist(prior->extrinsics(), est_ext) < angle_thresh;
        if (!bAreCloseEnough)
        {
          //cout << " atoms not close enough... " << endl;
          continue;
        }
      }

      AtomPair atompair = matchwithFitter(prior, query, modelfitter);

      if (atompair.result().success())
      {

        if (min_err > atompair.result().err())
        {
          min_err = atompair.result().err();
          est_ext = atompair.generateExtrinsics(query);
          if (angle_thresh > 0)
          {
            angle_thresh *= 0.7;
          }
          else
            angle_thresh = query->camera().fov_max();
        }
        pairsFound.push_back(atompair);
      }
    }
  }
  return pairsFound;
}

void DijkstraWay(ImageMolecule& mol)
{

  if (mol.getAtoms().size() < 2)
    return;
  map<Ptr<ImageAtom> , PairNode> dijkstra_node_map;
  PairNodeData::max_depth = 0;
  PairNodeData::min_depth = 0;

  Ptr<ImageAtom> current = mol.getAnchor();
  dijkstra_node_map[current].node_data.distance = 0;
  if (current)
  {
    Extrinsics& anchorExt = current->extrinsics();
    anchorExt.val(Extrinsics::CONFIDENCE) = dijkstra_node_map[current].bconf_func(1e-1, 1e-1);
  }
  set<Ptr<ImageAtom> > Q = mol.getAtoms();

  while (!Q.empty() && !current.empty())
  {
    //  cout << "querying atom with UUID:  " << current->uid() << endl;
    const list<int>* pairs = mol.getPairIndices(current);
    PairNode& node = dijkstra_node_map[current];
    node.atom = current;
    for (list<int>::const_iterator it = pairs->begin(); it != pairs->end(); ++it)
    {
      const AtomPair* pair = mol.getPairByIndex(*it);
      PairNode& nextnode = dijkstra_node_map[pair->other(current)];
      nextnode.atom = pair->other(current);
      if (!nextnode.node_data.visited)
      {
        nextnode.setDist(*pair, node);
      }

    }
    float minnext = INFINITY;
    node.node_data.visited = true;
    Q.erase(current);
    current = NULL;
    for (set<Ptr<ImageAtom> >::iterator it = Q.begin(); it != Q.end(); ++it)
    {
      if (minnext > dijkstra_node_map[*it].node_data.distance)
      {
        minnext = dijkstra_node_map[*it].node_data.distance;
        current = *it;
        const std::list<int>* cur_pairs = mol.getPairIndices(current);
        for (std::list<int>::const_iterator pair_idx = cur_pairs->begin(); pair_idx != cur_pairs->end(); ++pair_idx)
        {
          dijkstra_node_map[current].node_data.neighbors.insert(*(mol.getPairByIndex(*pair_idx)));
        }
      }
    }
  }

#if 1
  // no graphviz on phone!
#else
  // gv file 1: the path used to set trinsics
  std::stringstream ss;
  ss << MoleculeProcessor::GRAPHVIZ_FILENAME.c_str();
  std::ofstream out(ss.str().c_str());
  PairNode::graphviz_dump(out, dijkstra_node_map);

  // gv file 2: verbose file, with all 'implicit errors' between pairs
  std::stringstream ss_verbose;
  ss_verbose << MoleculeProcessor::VERBOSE_GRAPHVIZ_FILENAME.c_str();
  std::ofstream vout(ss_verbose.str().c_str());
  PairNode::graphviz_dump_all(vout, dijkstra_node_map);
#endif

}

void MoleculeProcessor::findAndSetTrinsics(ImageMolecule& mol, TFinder WAY)
{

  switch (WAY)
  {
    case DIJKSTRA:
      DijkstraWay(mol);
      break;
  }

}
MoleculeGlob::MoleculeGlob() :
  uid_count_(0)
{

}
float MoleculeGlob::minDistToAtom(const ImageAtom& atom) const
{

  Ptr<ImageAtom> minAtom = minDistAtom(atom);
  if (!minAtom.empty())
    return angularDist(atom.extrinsics(), minAtom->extrinsics());
  return CV_PI;
}
cv::Ptr<ImageAtom> MoleculeGlob::minDistAtom(const ImageAtom& atom) const
{
  float min_dist = CV_PI * 2;

  if (!atom.extrinsics().flag(Extrinsics::ESTIMATED))
    return NULL;

  Ptr<ImageAtom> min_atom;

  std::set<Ptr<ImageMolecule> >::const_iterator m_iter = molecules.begin();

  for (; m_iter != molecules.end(); ++m_iter)
  {
    const set<Ptr<ImageAtom> >& atoms = (*m_iter)->getAtoms();
    set<Ptr<ImageAtom> >::const_iterator ait = atoms.begin();

    for (; ait != atoms.end(); ++ait)
    {
      const Ptr<ImageAtom>& prior = *ait;

      float n_min = angularDist(prior->extrinsics(), atom.extrinsics());
      if (n_min < min_dist)
      {
        min_dist = n_min;
        min_atom = prior;
      }
    }
  }
  return min_atom;
}
namespace
{

struct QueryIdx
{
  inline int operator()(const DMatch& m) const
  {
    return m.queryIdx;
  }
};

struct TrainIdx
{
  inline int operator()(const DMatch& m) const
  {
    return m.trainIdx;
  }
};

template<class Idx>
  struct CompareOpIdx
  {
    Idx idx;
    inline bool operator()(const DMatch& lhs, const DMatch& rhs) const
    {
      return idx(lhs) < idx(rhs) || (idx(lhs) == idx(rhs) && lhs.distance < rhs.distance);
    }

  };

template<class Idx>
  struct EqualOpIdx
  {
    Idx idx;
    inline bool operator()(const DMatch& lhs, const DMatch& rhs) const
    {
      return idx(lhs) == idx(rhs);// || (idx(lhs) == idx(rhs) && lhs.distance < rhs.distance);
    }

  };

template<class IdxOp>
  inline void uniqueMatches(const MatchesVector& matches, MatchesVector& output, const IdxOp& idx)
  {
    output = matches;
    std::sort(output.begin(), output.end(), CompareOpIdx<IdxOp> ());
    std::unique(output.begin(), output.end(), EqualOpIdx<IdxOp> ());
  }

void uniqueMatches(const MatchesVector& matches, MatchesVector& output)
{
  uniqueMatches(matches, output, QueryIdx());
  uniqueMatches(output, output, TrainIdx());
  std::sort(output.begin(),output.end());
}

void flattenKNN(const vector<vector<DMatch> >& matches, vector<DMatch>& out)
{
  out.clear();
  size_t count = 0;
  for (size_t i = 0; i < matches.size(); i++)
  {
    count += matches[i].size();
  }
  out.reserve(count);
  for (size_t i = 0; i < matches.size(); i++)
  {
    out.insert(out.end(), matches[i].begin(), matches[i].end());
  }

}
void maskMatchesByTrainImgIdx(const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask)
{
  mask.resize(matches.size());
  fill(mask.begin(), mask.end(), 0);
  for (size_t i = 0; i < matches.size(); i++)
  {
    if (matches[i].imgIdx == trainImgIdx)
      mask[i] = 1;
  }
}

void condenseByTrainImgIdx(vector<DMatch>& matches, int trainImgIdx, vector<DMatch>& tmatches)
{
  tmatches.clear();
  for (size_t i = 0; i < matches.size(); i++)
  {
    //distance threshold is from experimentation
    if (matches[i].imgIdx == trainImgIdx)
    {
      //if (matches[i].distance < 90)
      tmatches.push_back(matches[i]);
      matches[i] = matches.back();
      matches.pop_back();
      i--;
    }
  }
  uniqueMatches(tmatches, tmatches);
}
struct ptrReplace
{
  Ptr<ImageAtom> query;
  ptrReplace(Ptr<ImageAtom> query) :
    query(query)
  {
  }
  void operator()(AtomPair& apair)
  {
    apair.setAtom2(query);
  }
};
}
Ptr<ImageAtom> MoleculeGlob::queryAtomToGlob(cv::Ptr<ModelFitter> fitter, const ImageAtom& _atom,
                                             std::list<AtomPair>& pairs, bool clone)
{

  Ptr<ImageAtom> query;// = atom.clone();
  if (matcher_.empty() || atoms_.empty())
    return query;
  ImageAtom atom = _atom;
  pairs.clear();
  vector<DMatch> matches_all;

  vector<vector<DMatch> > knnmatches;
  int knn = 4;// + 2 * log(atoms_.size()) / log(2);

  vector<Mat> masks;
  //generateMasks(atom, masks);
  matcher_->knnMatch(atom.features().descriptors(),knnmatches,knn,masks);
  //matcher_->match(atom.features().descriptors(), matches_all, masks);
  flattenKNN(knnmatches, matches_all);

  for (size_t j = 0; j < atoms_.size(); j++)
  {
    vector<DMatch> matches;
    condenseByTrainImgIdx(matches_all, j, matches);
    if (matches.size() >= 2)
    {
      AtomPair atom_pair(atoms_[j], atom.ptrToSelf(), matches);
      pairs.push_back(atom_pair);
    }
  }

  cout << "found " << pairs.size() << " to test." << endl;
  FitPair pair_fitter(fitter, -1, new list<AtomPair> ());
  std::for_each(pairs.begin(), pairs.end(), pair_fitter);
  if (clone)
  {
    if (pair_fitter.good_pairs->size() > 0)
    {
      query = atom.clone();
      std::for_each(pair_fitter.good_pairs->begin(), pair_fitter.good_pairs->end(), ptrReplace(query));
    }
  }
  pairs = *pair_fitter.good_pairs;
  cout << "found " << pairs.size() << " are good." << endl;
  return query;
}
void MoleculeGlob::addAtomDescriptors(cv::Ptr<ImageAtom> atom)
{
  if (atom_uids_idxs_.count(atom->uid()))
    return;
  if (!atom.empty())
  {
    if (!atom->features().descriptors().empty())
    {
      all_descriptions_.push_back(atom->features().descriptors());
      atoms_.push_back(atom);
      if (matcher_.empty())
      {
        matcher_ = atom->features().makeMatcher();
        matcher_->clear();
      }
      matcher_->add(vector<Mat> (1, all_descriptions_.back()));
      matcher_->train();
    }
    atom_uids_idxs_[atom->uid()] = atoms_.size() - 1;
  }
}

void MoleculeGlob::setMatcher(cv::Ptr<cv::DescriptorMatcher> matcher)
{
  matcher_ = matcher;
}

void MoleculeGlob::generateMasks(const ImageAtom& atom, std::vector<cv::Mat>& masks) const
{

  masks.resize(atoms_.size());
  for (size_t i = 0; i < atoms_.size(); i++)
  {
    atoms_[i]->descriptorMatchMask(atom, masks[i], Mat(), 50);
  }

}
Ptr<ImageAtom> MoleculeGlob::addAtomToGlob(cv::Ptr<ModelFitter> fitter, const ImageAtom& atom)
{

  std::list<AtomPair> pairs;
  Ptr<ImageAtom> query = queryAtomToGlob(fitter, atom, pairs);

  if (pairs.empty())
  {
    query = atom.clone();
    query->setUid(uid_count_++);
    // either there are no _molecules yet or I didn't match any existing ones
    Ptr<ImageMolecule> mol_new = new ImageMolecule();
    mol_new->insertAtom(query);
    molecules.insert(mol_new);
    addAtomDescriptors(query);
  }
  else
    addPrefittedPairs(pairs);

  return query;
}

struct PairGlobber
{
  MoleculeGlob* glob;
  PairGlobber(MoleculeGlob* glob) :
    glob(glob)
  {
  }
  void operator()(const AtomPair & pair)
  {
    if (!pair.result().success())
    {
      return;
    }

    if (pair.atom1()->uid() < 0)
    {
      Ptr<ImageAtom> atom = pair.atom1();
      atom->setUid(glob->uid_count_++);

    }

    if (pair.atom2()->uid() < 0)
    {
      Ptr<ImageAtom> atom = pair.atom2();
      atom->setUid(glob->uid_count_++);

    }

    glob->addAtomDescriptors(pair.atom2());
    glob->addAtomDescriptors(pair.atom1());
    std::set<Ptr<ImageMolecule> > matched_mols;
    // iterate over the molecules, query with each, merge if multiple matches
    std::set<Ptr<ImageMolecule> >::iterator mol = glob->getMolecules().begin();
    for (; mol != glob->getMolecules().end(); ++mol)
    {
      Ptr<ImageMolecule> molecule = *mol;
      if (molecule->hasAtom(pair.atom1()) || molecule->hasAtom(pair.atom2()))
      {
        molecule->insertPair(pair);
        matched_mols.insert(molecule);
      }
    }
    if (matched_mols.empty())
    {
      ImageMolecule* mol = new ImageMolecule();
      mol->insertPair(pair);
      glob->addMolecule(mol);
    }
    else
    {

      Ptr<ImageMolecule> match_mol_first = *matched_mols.begin();

      matched_mols.erase(match_mol_first);
      while (matched_mols.size())
      {

        Ptr<ImageMolecule> mol = *matched_mols.begin();
        Ptr<ImageAtom> shared = match_mol_first->hasAtom(pair.atom1()) ? pair.atom1() : pair.atom2();
        match_mol_first->merge(shared, *mol);
        matched_mols.erase(mol);
        glob->getMolecules().erase(mol);
      }

    }
  }

};

void MoleculeGlob::addPrefittedPairs(const std::list<AtomPair>& pairs, cv::Ptr<ImageAtom> atom)
{

  for_each(pairs.begin(), pairs.end(), PairGlobber(this));
}

void MoleculeGlob::addPrefittedPairs(const std::vector<AtomPair>& pairs, cv::Ptr<ImageAtom> atom)
{
  for_each(pairs.begin(), pairs.end(), PairGlobber(this));

}

cv::Ptr<ImageMolecule> MoleculeGlob::getBiggestMolecule() const
{

  if (molecules.empty())
    return NULL;
  cv::Ptr<ImageMolecule> biggest;

  std::set<cv::Ptr<ImageMolecule> >::iterator it = molecules.begin();
  biggest = *(it);
  while (++it != molecules.end())
  { // try to keep molecule with the most atoms!
    if ((*it)->getAtoms().size() > biggest->getAtoms().size())
    {
      biggest = (*it);
    }
  }
  return biggest;
}

void MoleculeGlob::truncateMolecules(Ptr<ImageMolecule> mol_in)
{
  if (mol_in.empty())
  {
    mol_in = getBiggestMolecule();
  }
  molecules.clear();
  molecules.insert(mol_in);
}

cv::Ptr<ImageMolecule> MoleculeGlob::getMerged() const
{
  Ptr<ImageMolecule> globmol = new ImageMolecule();
  set<Ptr<ImageMolecule> > mols = getMolecules();
  while (!mols.empty())
  {
    Ptr<ImageMolecule> mol = *mols.begin();

    globmol->insertAtoms(mol->getAtoms());
    //todo verify that this is a bettter/worse option
    //globmol->insertPairs(mol->getPairs());
    mols.erase(mol);
  }
  return globmol;
}

namespace
{
struct BatchTrinsics
{
  void operator()(Ptr<ImageMolecule> mol)
  {
    if (mol->getAtoms().size() < 2)
      return;
    mol->setAnchor(mol->getMaximallyConnectedAtom());
    MoleculeProcessor::findAndSetTrinsics(*mol, MoleculeProcessor::DIJKSTRA);
  }
};
struct BatchWrite
{
  cv::FileStorage& fs_;
  BatchWrite(cv::FileStorage& fs) :
    fs_(fs)
  {

  }
  void operator()(Ptr<ImageMolecule> mol)
  {
    mol->serialize(fs_);
  }
};

struct DirOverrider
{
  const std::string& dir;
  DirOverrider(const std::string& dir) :
    dir(dir)
  {

  }

  void operator()(Ptr<ImageAtom> atom)
  {
    atom->images().path() = dir;
  }

  void operator()(Ptr<ImageMolecule> mol)
  {
    for_each(mol->getAtoms().begin(), mol->getAtoms().end(), *this);
  }
};

}
void MoleculeGlob::batchFindAndSetTrinsics()
{
  for_each(molecules.begin(), molecules.end(), BatchTrinsics());
}

void MoleculeGlob::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  fs << "molecules";
  fs << "[";
  for_each(molecules.begin(), molecules.end(), BatchWrite(fs));
  fs << "]";
  fs << "}";
}
void MoleculeGlob::deserialize(const cv::FileNode& fn)
{

  FileNode mols = fn["molecules"];
  CV_Assert(mols.type() == FileNode::SEQ)
    ;

  for (size_t i = 0; i < mols.size(); i++)
  {
    Ptr<ImageMolecule> mol(new ImageMolecule());
    mol->deserialize(mols[i]);
    addMolecule(mol);
  }

}

void MoleculeGlob::overideDirectory(std::string directory)
{
  for_each(molecules.begin(), molecules.end(), DirOverrider(directory));
}

const std::string Globber::VERBOSE = "GlobberVerbose";
void Globber::operator()(cv::Ptr<ImageAtom> atom)
{
  glob->addAtomToGlob(fitter, *atom);
}

PairPointsCSV::PairPointsCSV(std::ostream&out) :
  out(out)
{
}

void PairPointsCSV::operator()(const std::pair<const Point2f&, const Point2f&>& pp)
{

  out << pp.first.x << "," << pp.first.y << "," << pp.second.x << "," << pp.second.y << std::endl;
}
void PairPointsCSV::operator()(const AtomPair&pair)
{
  out << "##### " << pair.atom1()->images().fname() << "," << pair.atom2()->images().fname() << std::endl;
  const vector<Point2f>& pts1 = pair.pts1();
  const vector<Point2f>& pts2 = pair.pts1();
  for (size_t i = 0; i < pts1.size(); i++)
  {

    (*this)(std::make_pair(pts1[i], pts2[i]));
  }
}

int PairNodeData::max_depth = 0;
int PairNodeData::min_depth = 0;

void PairNode::setDist(const AtomPair& pair, const PairNode& prev_node)
{

  float in_pair_error = error_func(pair, prev_node.node_data);
  float next_error_candidate = in_pair_error + prev_node.node_data.distance;

  if (this->node_data.distance > next_error_candidate)
  {
    this->min_prev = pair.other(this->atom);
    this->node_data.distance = next_error_candidate;
    Extrinsics& ext = atom->extrinsics();

    Mat R_increment = pair.TMtoOther(min_prev, FitterResult::R);
    Mat R_previous = min_prev->extrinsics().mat(Extrinsics::R).clone();
    ext.mat(Extrinsics::R) = R_increment * R_previous;
    Mat omega;
    cv::Rodrigues(ext.mat(Extrinsics::R), omega);
    ext.mat(Extrinsics::W) = omega;

    double err_new = in_pair_error;
    double err_prv = next_error_candidate - err_new;
    ext.flag(Extrinsics::ESTIMATED) = pair.result().success();
    ext.val(Extrinsics::CONFIDENCE) = bconf_func(err_new, err_prv);

    // update the node data
    this->node_data.depth = prev_node.node_data.depth + 1;
    this->node_data.mode = ext.flag(Extrinsics::ESTIMATED);
    if (this->node_data.depth > PairNodeData::max_depth)
    {
      PairNodeData::max_depth++;
    }

  }
}

std::ostream& operator <<(std::ostream& out, const PairNode& node)
{

  std::string name1 = "TOP";
  std::string color_str = "color=black";
  if (node.min_prev)
  {
    name1 = (*node.min_prev).images().fname();
  }
  std::string name2 = "TOP";
  double dbl_conf = 0.0;
  if (node.atom)
  {
    name2 = (*node.atom).images().fname();
    dbl_conf = node.atom->extrinsics().val(Extrinsics::CONFIDENCE);
    bool tm_mode = node.atom->extrinsics().flag(Extrinsics::ESTIMATED);
    if (tm_mode)
    {
      color_str = "color=blue";
    }
    else// if (ImageAtomTrinsincs::NOT_TRUSTED == tm_mode)
    {
      color_str = "color=red";
    }
  }
  out << "node [" << color_str << ",fontname=Arial] \n" << "edge [color=black, style=solid] \n" //setup options
      << "\"" << name1 << "\" -> \"" << name2 << "\" [label=\"" << "E=" << std::setprecision(3)
      << node.node_data.distance << "  C=" << setprecision(3) << dbl_conf << "\", fontcolor=darkgreen];";
  return out;
}

std::ostream& operator <<(std::ostream& out, const AtomPair& pair)
{
  std::string name1 = pair.atom1()->images().fname();
  std::string name2 = pair.atom2()->images().fname();

  // once trinsics are set, unless atom1 is up the chain from atom2,
  // the implicit R will not match the pair-wise R we found during matching!
  cv::Mat R2 = pair.atom2()->extrinsics().mat(Extrinsics::R);
  cv::Mat R1 = pair.atom1()->extrinsics().mat(Extrinsics::R);
  cv::Mat Rpair = pair.TMtoOther(pair.atom1(), Extrinsics::R);
  cv::Mat Rimplicit = R2 * R1.t();

  float reproj_err = calcReprojectonError(pair.pts1(), pair.pts2(), pair.result().inlier_mask(), Rimplicit,
                                          pair.atom1()->camera().K());
  float result_err = calcReprojectonError(pair.pts1(), pair.pts2(), pair.result().inlier_mask(), Rpair,
                                          pair.atom1()->camera().K());

  out << "node [color=black,fontname=Arial] \n" << "edge [color=blue, style=dashed] \n" //setup options
      << "\"" << name1 << "\" -> \"" << name2 << "\" [label=\"" << "PairError= " << result_err << " ImplicitError= "
      << reproj_err << "\", fontcolor=blue];";
  return out;
}

void PairNode::graphviz_dump(std::ostream& out, const std::map<cv::Ptr<ImageAtom>, PairNode>& node_map)
{
  out << "digraph PairNodes{\n";
  std::map<cv::Ptr<ImageAtom>, PairNode>::const_iterator it = node_map.begin();
  while (it != node_map.end())
  {
    out << it->second << std::endl;
    ++it;
  }
  out << "}" << std::endl;
}

void PairNode::graphviz_dump_all(std::ostream& out, const std::map<cv::Ptr<ImageAtom>, PairNode>& node_map)
{
  out << "digraph PairNodes{\n";
  std::map<cv::Ptr<ImageAtom>, PairNode>::const_iterator it = node_map.begin();
  while (it != node_map.end())
  {

    if (it->first)
    {
      AtomPairSet pairs = it->second.node_data.neighbors;
      AtomPairSet::iterator pit = pairs.begin();

      for (; pit != pairs.end(); ++pit)
      {
        out << *pit << std::endl;
      }
    }
    out << it->second << std::endl;
    ++it;
  }
  out << "}" << std::endl;
}

std::string strip(const std::string& str, const std::string & what)
{
  return str.substr(0, str.find(what));
}

} // end namespace pano
