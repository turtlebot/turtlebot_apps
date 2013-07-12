/*

 * ImageMolecule.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee

 */
#include "pano_core/ImageMolecule.h"
#include "pano_core/ModelFitter.h"

#include <algorithm>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>
#include <functional>
#include <set>
#include <stdexcept>
#include <exception>

using namespace cv;
using namespace std;

namespace pano
{

namespace
{
//    * Helper functor for iterating over pairs
//      * and inserting them into a molecule

struct IAPairInserter
{
  IAPairInserter(ImageMolecule*mol) :
    mol(mol)
  {
  }
  ImageMolecule* mol;
  void operator()(const AtomPair& pair)
  {
    mol->insertPair(pair);
  }

};

struct IAInserter
{
  IAInserter(ImageMolecule*mol) :
    mol(mol)
  {
  }
  ImageMolecule* mol;
  void operator()(const Ptr<ImageAtom>& atom)
  {
    mol->insertAtom(atom);
  }
};
}

ImageMolecule::ImageMolecule()
{

}

ImageMolecule::ImageMolecule(const ImageMolecule& rhs) :
  pairmap_(rhs.pairmap_), atoms_(rhs.atoms_), pairs_(rhs.pairs_), anchor_(rhs.anchor_)
{

}

ImageMolecule::ImageMolecule(const std::list<AtomPair> & l_pairs)
{
  if (l_pairs.size())
    setAnchor(l_pairs.front().atom1());
  insertPairs(l_pairs);
}

ImageMolecule::ImageMolecule(const std::vector<AtomPair> & pairs)
{
  if (pairs.size())
    setAnchor(pairs.front().atom1());
  insertPairs(pairs);
}

void ImageMolecule::insertAtom(const Ptr<ImageAtom>& atom) throw ()
{
  if (!hasAtom(atom))
    atoms_.insert(atom);
  if (anchor_.empty())
    setAnchor(atom);
}

void ImageMolecule::insertPair(const AtomPair& pair) throw ()
{
  insertAtom(pair.atom1());
  insertAtom(pair.atom2());
  pairs_.push_back(pair);
  size_t idx = pairs_.size() - 1;
  pairmap_[pair.atom1()].push_back(idx);
  pairmap_[pair.atom2()].push_back(idx);
}

void ImageMolecule::peelAtoms(int flag)
{
  set<Ptr<ImageAtom> >::iterator aitr = this->atoms_.begin();
  for (; aitr != atoms_.end(); ++aitr)
  {
    Ptr<ImageAtom> atom = *aitr;//)->flush();
    atom->flush();
  }
}

namespace
{
bool badpairpredicate(const AtomPair& pair)
{
  return !pair.result().empty() && pair.result().success();
}

}

bool ImageMolecule::removePair(const AtomPair& pair) throw ()
{
  vector<AtomPair>::iterator it = std::find(pairs_.begin(), pairs_.end(), pair);
  if (it != pairs_.end())
  {
    pairs_.erase(it);
    pairmap_.clear();
    std::vector<AtomPair> tpairs = pairs_;
    pairs_.clear();
    insertPairs(tpairs);
    return true;
  }
  return false;
}

void ImageMolecule::removeBadPairs()
{
  std::remove_if(pairs_.begin(), pairs_.end(), badpairpredicate);
  pairmap_.clear();
  std::vector<AtomPair> tpairs = pairs_;
  pairs_.clear();
  insertPairs(tpairs);
}

void ImageMolecule::insertPairs(const vector<AtomPair>& pairs) throw ()
{
  for_each(pairs.begin(), pairs.end(), IAPairInserter(this));
}

void ImageMolecule::insertPairs(const list<AtomPair>& pairs) throw ()
{
  for_each(pairs.begin(), pairs.end(), IAPairInserter(this));
}

void ImageMolecule::insertAtoms(const std::set<cv::Ptr<ImageAtom> > & atoms)
{
  for_each(atoms.begin(), atoms.end(), IAInserter(this));
}

void ImageMolecule::setAnchor(const Ptr<ImageAtom>& atom) throw ()
{
  anchor_ = atom;
}

vector<AtomPair>& ImageMolecule::getPairs() throw ()
{
  return pairs_;
}

const vector<AtomPair>& ImageMolecule::getPairs() const throw ()
{
  return pairs_;
}
const Ptr<ImageAtom>& ImageMolecule::getAnchor() const throw (std::logic_error)
{

  if (atoms_.empty())
  {
    throw logic_error("The molecule is empty!");
  }

  //if the anchor has not been set then just return the begining of
  //atoms
  if (anchor_.empty())
    return *getAtoms().begin();

  return anchor_;
}

// TODO: Get list<int>* of atom pointers by UID Query

const list<int>* ImageMolecule::getPairIndices(const Ptr<ImageAtom>& atom) const
{
  map<Ptr<ImageAtom> , list<int> >::const_iterator it = pairmap_.find(atom);
  if (it == pairmap_.end())
  {
    cerr << "ERROR: requested atom is not in the molecule! " << endl;
    return NULL;
  }
  return &it->second;
}

const AtomPair* ImageMolecule::getPairByIndex(size_t idx) const
{
  if (idx >= pairs_.size()) {
    cerr << "ERROR: requested atom index is bogus! " << endl;
    return NULL;
  }
  return &pairs_[idx];
}

void ImageMolecule::merge(Ptr<ImageAtom> pivot, const ImageMolecule& molecule)
{
  //erase the pivot atom - if they are shared between the two
  //molecules
  if (atoms_.count(pivot))
  {
    atoms_.erase(pivot);
  }

  //first insert all the atoms
  atoms_.insert(molecule.atoms_.begin(), molecule.atoms_.end());

  //now insert the atom pair bonds
  pairs_.reserve(pairs_.size() + molecule.pairs_.size());

  //uses helper function object IAPairInserter
  //for stl for_each functionality
  for_each(molecule.pairs_.begin(), molecule.pairs_.end(), IAPairInserter(this));
}

set<Ptr<ImageAtom> >& ImageMolecule::getAtoms() throw ()
{
  return atoms_;
}

const set<Ptr<ImageAtom> >& ImageMolecule::getAtoms() const throw ()
{
  return atoms_;
}

bool ImageMolecule::hasAtom(const Ptr<ImageAtom>& atom) const throw ()
{
  return atoms_.count(atom);
}

void ImageMolecule::hasAtomThrow(const Ptr<ImageAtom>& atom) const throw (std::logic_error)
{
  if (!hasAtom(atom))
    throw logic_error("Atom not in the Molecule!");
}

namespace
{

typedef std::pair<int, Ptr<ImageAtom> > node_pair;
bool node_sorter(const node_pair& lhs, const node_pair& rhs)
{
  return lhs.first > rhs.first;
}
}

Ptr<ImageAtom> ImageMolecule::getMaximallyConnectedAtom(int node_offset) const
{
  std::vector<std::pair<int, Ptr<ImageAtom> > > node_connections;
  node_connections.reserve(atoms_.size());

  set<Ptr<ImageAtom> >::const_iterator it = atoms_.begin();

  while (it != atoms_.end())
  {
    const std::list<int>* indices = getPairIndices(*it);
    if (indices)
      node_connections.push_back(make_pair(indices->size(), *it));
    ++it;
  }

  sort(node_connections.begin(), node_connections.end(), node_sorter);

  if (node_connections.size())
  {
    if (node_offset < 0 || node_offset >= (int)node_connections.size())
    {
      std::cerr << "bogus value of node offset! " << std::endl;
      node_offset = 0;
    }
    return (node_connections[node_offset]).second;
  }
  else
    return NULL;

}

void ImageMolecule::serialize(cv::FileStorage& fs) const
{
  fs << "{";
  fs << "atoms";
  {
    std::set<Ptr<ImageAtom> >::const_iterator atom = atoms_.begin();
    fs << "[";
    while (atom != atoms_.end())
    {
      (*atom)->serialize(fs);
      atom++;
    }
    fs << "]";
  }
  fs << "pairs";

  {
    std::vector<AtomPair>::const_iterator pit;
    fs << "[";
    for (pit = pairs_.begin(); pit != pairs_.end(); ++pit)
    {
      pit->serialize(fs);
    }
    fs << "]";
  }
  fs << "}";
}
void ImageMolecule::deserialize(const cv::FileNode& fn)
{
  FileNode atoms = fn["atoms"];
  CV_Assert(atoms.type() == FileNode::SEQ);

  std::map<int, Ptr<ImageAtom> > a_map;
  for (size_t i = 0; i < atoms.size(); i++)
  {
    Ptr<ImageAtom> atom(new ImageAtom);
    atom->deserialize(atoms[i]);
    a_map[atom->uid()] = atom;
    //we will insert from pairs...
    insertAtom(atom);
  }

  FileNode pairs = fn["pairs"];
  CV_Assert(pairs.type() == FileNode::SEQ);
  vector<AtomPair> pairs_temp;
  pairs_temp.resize(pairs.size());
  for (size_t i = 0; i < pairs.size(); i++)
  {
    pairs_temp[i].deserialize(pairs[i]);

    pairs_temp[i].setAtom1(a_map[pairs_temp[i].atom1()->uid()]);
    pairs_temp[i].setAtom2(a_map[pairs_temp[i].atom2()->uid()]);

  }

  insertPairs(pairs_temp);

}

//void ImageMolecule::serialize(std::ostream& out, int  flags)const{
//
//    std::set<Ptr<ImageAtom> >::const_iterator atom = imageatoms.begin();
//    out << "[" << imageatoms.size() << "]";
//    while(atom != imageatoms.end()){
//        (*atom)->serialize(out,ImageAtom::SERIALIZE_ATOM);
//        atom++;
//    }
//    out << "[" << atom_pair_bonds.size() << "]";
//
//    std::vector<AtomPair>::const_iterator pit;
//
//    for (pit = atom_pair_bonds.begin(); pit != atom_pair_bonds.end(); ++pit) {
//        pit->serialize(out);
//    }
//
//}


//void ImageMolecule::deserialize(std::istream& in, int  flags){
//
//    Atoms().resetPool();
//
//    int atoms_size;
//    eatchar(in, '[');
//    in >> atoms_size;
//    eatchar(in, ']');
//
//    for(int i = 0; i < atoms_size; i++){
//        cv::Ptr<ImageAtom> atom = new ImageAtom();
//
//        atom->deserialize(in,ImageAtom::DESERIALIZE_ATOM);
//
//        //add the atom to the atom pool to alleviate redundant
//        //atoms
//        atom = Atoms().getAtom(*atom);
//        insertAtom(atom);
//    }
//
//    int pairs_size;
//    eatchar(in, '[');
//    in >> pairs_size;
//    eatchar(in, ']');
//
//    for(int i = 0; i < pairs_size; i++){
//       AtomPair pair;
//       pair.deserialize(in);
//       insertPair(pair);
//    }
//
//}


}
