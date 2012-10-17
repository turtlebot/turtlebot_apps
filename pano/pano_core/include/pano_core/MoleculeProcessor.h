/*
 * MoleculeProcessor.h
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#ifndef MOLECULEPROCESSOR_H_
#define MOLECULEPROCESSOR_H_

#include "pano_core/ImageAtom.h"
#include "pano_core/ImageMolecule.h"

#include <list>
#include <map>
#include <set>

namespace pano
{

/** A container for algorithms that run on a molecule. For now does not store any particular data.
 *  In the future it might have some settings stored.
 */
class MoleculeProcessor
{

private:
  MoleculeProcessor();
public:
  /** show pairs while matching (e.g. point correspondence) */
  static const std::string SHOW_PAIRS;

  /** default filename for saving meta data to graphviz 'dot' file */
  static const std::string GRAPHVIZ_FILENAME;

  /** default filename for saving meta data to verbose graphviz 'dot' file */
  static const std::string VERBOSE_GRAPHVIZ_FILENAME;

  /** Take two atoms and a generic modelfitter, create ImageAtomPair.
   * success/failure of the matching is stored inside the ImageAtomPair.
   */
  static AtomPair matchwithFitter(cv::Ptr<ImageAtom> atom1, cv::Ptr<ImageAtom> atom2, ModelFitter& modelfitter);
  static AtomPair matchwithFitter(cv::Ptr<ImageAtom> atom1, cv::Ptr<ImageAtom> atom2, cv::Ptr<ModelFitter> modelfitter){
     return matchwithFitter(atom1,atom2,*modelfitter);
    }

  /** Using a 'fit method' to align <this> atom with input atom
   * \param atom2 : incoming atom to match
   * \param modelfitter: a type of fit scheme to match
   * Does NOT change molecule, generates list of pairs containing atom2 and atoms
   * in molecule that atom2 wants to bind to.
   */
  static std::list<AtomPair> queryWithAtom(const ImageMolecule& molecule, cv::Ptr<ImageAtom> atom2,
                                           cv::Ptr<ModelFitter> modelfitter, float angle_thresh = -1.0);

  /** \brief create the map that tells how many pairs it takes to
   * travel along longest route
   * from atom1 to atom2 of the anchor_pair. Recursive function.
   */
  static void create_num_steps_map(const ImageMolecule& mol, const cv::Ptr<ImageAtom> anchor_atom, const cv::Ptr<
      ImageAtom>& atom, std::map<cv::Ptr<ImageAtom>, size_t>& num_steps_map,
                                   std::map<cv::Ptr<ImageAtom>, std::set<std::string> > atoms_queried, std::map<
                                       cv::Ptr<ImageAtom>, bool>& node_lock, const std::string& graphvizDbgFilename =
                                       std::string(""));

  /**  \brief  work backward from 'atom2' in anchor pair, store list of pairs
   *    and atoms along the way until getting back to atom1 anchor.
   *   kept hidden to discourage improper external usage.
   */
  static void getCyclePairsAndAtoms(const ImageMolecule& mol, const std::map<cv::Ptr<ImageAtom>, size_t>& num_steps,
                                    const cv::Ptr<ImageAtom>& in_atom,
                                    std::map<cv::Ptr<ImageAtom>, AtomPair>& atom_to_pair_map,
                                    std::list<cv::Ptr<ImageAtom> >& atomchain, std::list<AtomPair>& cyclePairs);

  enum TFinder
  {
    DIJKSTRA
  };

  /** this finds and sets the TM and confidence for each atom in the molecule
   *\param mol
   */
  static void findAndSetTrinsics(ImageMolecule& mol, TFinder WAY);
};

class MoleculeGlob;

/** Function object for stl::algorithms that
 * iterator over collections of cv::Ptr<ImageAtom>
 * This one - takes atoms and queryies the MoleculeGlob with the
 * ModelGitter
 */
struct Globber
{
  static const std::string VERBOSE;
  MoleculeGlob* glob;
  cv::Ptr<ModelFitter> fitter;
  Globber(MoleculeGlob*glob, cv::Ptr<ModelFitter> fitter) :
    glob(glob), fitter(fitter)
  {
  }
  void operator()(cv::Ptr<ImageAtom> atom);

};

/** A collection of Molecules that are easy to add atoms too and handles
 * matching and merging of multiple molecules
 */

class MoleculeGlob : public serializable
{

public:
  MoleculeGlob();

  float minDistToAtom(const ImageAtom & atom) const;

  /** get glob atom that is closest to the input atom
   */
  cv::Ptr<ImageAtom> minDistAtom( const ImageAtom & atom) const;

  /** see if atom fits the glob
    */
  cv::Ptr<ImageAtom> queryAtomToGlob( cv::Ptr<ModelFitter> fitter,
                         const ImageAtom& atom, std::list<AtomPair>& pairs, bool clone = true);

  /**  This adds the atom to the glob given the fitter,
   * internally this calls MoleculeProcessor::queryWithAtom
   * and only adds it to a molecule if the fitter result is "good"
   * if the result is bad then this creates a new molecule in the
   * glob - containing the unmatched atom.
   */
  cv::Ptr<ImageAtom> addAtomToGlob(cv::Ptr<ModelFitter> fitter, const ImageAtom& atom);

  /** convience function for greating a Globber functor
   * that is assocatiated with this MoleculeGlob
   *<code>
   MoleculeGlob glob;
   for_each(atoms.begin(),atoms.end(),glob.getGlobber(fitter));
   </code>
   *
   *the Globber is used to add atoms to the MoleculeGlob
   * basically calls addAtomToGlob for each atom passed to the
   * Globber operator() function.
   */
  Globber getGlobber(cv::Ptr<ModelFitter> fitter)
  {
    return Globber(this, fitter);
  }

  /** \brief Returns the Molecule with the most Atoms
   * This function searchs the set of molecules associated with this
   * glob, and returns the one that contains the most atoms.
   *
   * this is meaningful because Molecules produced with
   * MoleculeGlob are guarenteed to only have
   * atoms that are interconnected.
   */
  cv::Ptr<ImageMolecule> getBiggestMolecule() const;

  /** \brief given a pointer to a molecule - this deletes all other
   * molecules from the Glob and keeps only the given molecule
   *
   * \param mol_in is an optional pointer to the ImageMolecule to keep,
   *        the default value - NULL - will truncate all but the largest
   *        ImageMolecule - given by getBiggestMolecule()
   */
  void truncateMolecules(cv::Ptr<ImageMolecule> mol_in = cv::Ptr<ImageMolecule>());

  /** \brief attach pairs to a MoleculeGlobber
   */
  void addPrefittedPairs(const std::list<AtomPair>& pairs, cv::Ptr<ImageAtom> atom = cv::Ptr<ImageAtom>());

  /** \brief attach pairs to a MoleculeGlobber
   */
  void addPrefittedPairs(const std::vector<AtomPair>& pairs, cv::Ptr<ImageAtom> atom= cv::Ptr<ImageAtom>());

  /** \brief This will remove all the molecules from the glob.
   */
  void reset()
  {
    molecules.clear();
  }

  void addMolecule(cv::Ptr<ImageMolecule> molecule)
  {
    molecules.insert(molecule);
  }

  /** this returns the set of ImageMolecules upon which this glob is based.
   */
  const std::set<cv::Ptr<ImageMolecule> > & getMolecules() const
  {
    return molecules;
  }

  void overideDirectory(std::string directory);

  std::set<cv::Ptr<ImageMolecule> > & getMolecules()
  {
    return molecules;
  }

  cv::Ptr<ImageMolecule> getMerged() const;

  void batchFindAndSetTrinsics();

  /*
   * serializable functions
   */
  virtual int version() const
  {
    return 0;
  }

  virtual void serialize(cv::FileStorage& fs) const;
  virtual void deserialize(const cv::FileNode& fn);
  void setMatcher(cv::Ptr<cv::DescriptorMatcher> matcher);
private:
  void generateMasks(const ImageAtom& atom, std::vector<cv::Mat>& masks)const;
  void addAtomDescriptors(cv::Ptr<ImageAtom> atom);
  /** MoleculeGlob holds all the molecules by cv::Ptr
   * in a set.
   */
  std::set<cv::Ptr<ImageMolecule> > molecules;
  int uid_count_;
  friend struct PairGlobber;

  std::vector<cv::Ptr<ImageAtom> > atoms_;
  std::map<int,int> atom_uids_idxs_;
  std::vector<cv::Mat> all_descriptions_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;

};

struct MoleculePeeler
{
  void operator()(cv::Ptr<ImageMolecule>& mol)
  {
    mol->peelAtoms();
  }
};

}

#endif // MOLECULEPROCESSOR_H
