/*
 * ImageMolecule.h
 *
 *  Created on: Oct 20, 2010
 *      Author: erublee
 */

#ifndef IMAGEMOLECULE_H_
#define IMAGEMOLECULE_H_
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>

#include <pano_core/ImageAtom.h>
#include <pano_core/ModelFitter.h>

namespace pano {


/** \class ImageMolecule is meant to be a collection of ImageAtom s, holding them in
 * a way that describes their relationship to one another.
 *
 */
class ImageMolecule: public serializable {
public:
    /** Default constructor, doesn't initialize anything
     */
    ImageMolecule();

    /** copy constructor - this is relatively heavy operation - as it
      * copies the sets and maps that keep track of atoms,
      * however - atoms are all by pointer.
      * Above is Debatable; ? everything is irrelevant compared to image copies and blending ?
      */
    ImageMolecule(const ImageMolecule& rhs);

    /** this creates an ImageMolecule out of a list of pairs.  These pairs
     * should already be well defined
     *
     *  Please note that the pairs are not required to completely interconnect
     *  And you may get a molecule with disjoint chains of atoms
     *
     * \param pairs the pairs to initialize with - these should have valid
     *  pointers to atoms and preferably valid ModelFitter results.
     *
     */
    ImageMolecule(const std::list<AtomPair> & pairs);



    ImageMolecule(const std::vector<AtomPair> & pairs);

    ~ImageMolecule(){}

    /** Insert an atom into the molecule, atoms are unique, and may be
     *  added more than once.
     * \param atom the atom to insert - atoms are stored by their pointer, so
     *  the same image may be in two atoms if it
     *   has been loaded more than once(be careful with this)
     *
     */
    void insertAtom(const cv::Ptr<ImageAtom>& atom) throw();

    void insertAtoms(const std::set<cv::Ptr<ImageAtom> > & atoms);

    /** Insert a pair
     * this inserts a pair into the molecule, no check for uniqueness is done
     * \param the pair to be inserted - atoms are unique though
     *
     */
    void insertPair(const AtomPair& pair) throw();

    bool removePair(const AtomPair& pair) throw();

    void removeBadPairs();

    /** This inserts a list of pairs into the atom
     * \param pairs the pairs to insert, no uniqueness done on pairs
     *          - atoms will not be double added though
     */
    void insertPairs(const std::list<AtomPair>& pairs) throw();
    void insertPairs(const std::vector<AtomPair>& pairs) throw();

    /** revert ImageAtom objects to AtomShells by freeing their images and keypoints.
      * flag = 0 for free everything but Imageraw + trinsics (all we need to blend)
      * flag = 1 for free all including the image (must reload from filename to do anything)
      */
    void peelAtoms( int flag = 0);

    /** \brief Gets a reference to the set of atoms that defines this
      * molecule
      * \return a mutable reference to the set of atoms;
      */
    std::set<cv::Ptr<ImageAtom> >& getAtoms() throw();

    /** \brief Gets a reference to the set of atoms that defines this
      * molecule
      * \return an immutable reference to the set of atoms;
      */
    const std::set<cv::Ptr<ImageAtom> >& getAtoms() const throw();


    /** \brief Gets a reference to the vector of pairs that defines this
      * molecule
      * \return a mutable reference to pairs
      */
    std::vector<AtomPair>& getPairs() throw();

    /** \brief Gets a reference to the vector of pairs that defines this
      * molecule
      * \return a mutable reference to pairs
      */
    const std::vector<AtomPair>& getPairs() const throw();

    /** Set's the given atom as the anchor to this molecule,
     * The anchor may have varying definitions to one's algorithms,
     * For MoleculeProcessor::findAndSetTrinsics - this atom's rotation
     * becomes "the"
     * pivotal rotation that every other atom's rotation is relative to
     *
     * \param atom - the atom that is to act as the anchor for the molecule
     */
    void setAnchor(const cv::Ptr<ImageAtom>& atom) throw();

    /** Get the anchor to this molecule
     * \return a pointer to the ImageAtom that is the anchor,
     *  may crash if the molecule is empty!
     */
    const cv::Ptr<ImageAtom>& getAnchor() const throw(std::logic_error);

    /** \brief check if the given atom is in the molecule
      * \return true if its already in the molecule
      */
    bool hasAtom(const cv::Ptr<ImageAtom>& atom) const
            throw();

    /** \brief check if the given atom is in the molecule
      * but throw if its not - does not return a value
      */
    void hasAtomThrow(const cv::Ptr<ImageAtom>& atom)const
            throw(std::logic_error);

    /** get a list of indexes for the pairs that include the incoming atom,
     * don't hang onto the list pointer.
     *
     * \return a list of the associated pairs for the given atom,
     *           null if not found
     *
     */
    const std::list<int>* getPairIndices(const cv::Ptr<ImageAtom>& atom) const;



    const AtomPair* getPairByIndex(size_t idx) const;

    cv::Ptr<ImageAtom> getMaximallyConnectedAtom( int node_offset = 0 ) const;

    void merge(cv::Ptr<ImageAtom> pivot, const ImageMolecule& molecule);

    /*
     * serializable functions
     */
    virtual int version() const
    {
      return 0;
    }

    virtual void serialize(cv::FileStorage& fs) const;
    virtual void deserialize(const cv::FileNode& fn);

private:
    std::map<cv::Ptr<ImageAtom> , std::list<int> > pairmap_;
    std::set<cv::Ptr<ImageAtom> > atoms_;
    std::vector<AtomPair > pairs_;
    cv::Ptr<ImageAtom> anchor_;
};

}


#endif /* IMAGEMOLECULE_H_ */
