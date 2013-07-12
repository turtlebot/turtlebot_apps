#ifndef ATOMPAIR_H_
#define ATOMPAIR_H_

#include "pano_core/ModelFitter.h"
#include <iostream>

namespace pano {

/** structure that captures the "things that are not available in atom/pair"
  * for possible use with function pointer to pair error.
  * e.g. how deep the node is in the graph, etc
  */
struct PairNodeData {

    float distance;
    int  depth;
    int visited;
    bool mode;

    PairNodeData( float dist, int depth, int vis ) :
            distance(dist), depth(depth), visited(vis),
            mode(true) { }
    PairNodeData( const PairNodeData& node ) :
    distance(node.distance), depth(node.depth), visited(node.visited),
        mode(node.mode) { }
    AtomPairSet neighbors;

    static int max_depth;
    static int min_depth;
};

/** \brief a typedef for easily swapping out the error
  *     function for PairNode.
  */
typedef float (*PairMatchErrFPT)(const AtomPair& pair, const PairNodeData& node_data );

/** \brief a typedef for easily swapping out the blending confidence
  *     function for PairNode. Want to allow low-confidence things to get drawn, but
  * only if they don't overlap something else that has high confidence.
  */
typedef float (*PairBlendConfFPT)(  float err_new, float err_prv );

/** exponential decrease of blender confidence with match error
 */
inline float PairConfExpLaw(  float err_new, float err_prv ) {
 double C_0   = 20.0;
 double C_1   = 0.5;
 double beta0 = 0.5;   // E=beta0 => C=C_0
 double beta1 = 100.0; // E=beta1 => C=C_1
 double alpha = 1e-2;  // lim(C)_{E->infty) = alpha
 double gamma = -1.0/(beta1-beta0) * log( (C_1-alpha) / C_0 );
 double err_net = err_new + err_prv;
 return (float) alpha + C_0 * exp( -(err_net-beta0) * gamma );
}

/** 1/x decrease of blender confidence with match error
  * m-code:
  * E = logspace(-1,3,500); gam0 = beta0 * C0;
  * CofE = gam0 ./ (E+eps0) + alpha; loglog(E,CofE)
  */
inline float PairConfInvLaw( float err_new, float err_prv) {
 double C_0   = 1.0;
 double eps0  = 1e-1;
 double alpha = 1e-3;  // lim(C)_{E->infty) = alpha
 double err   = err_new + err_prv; // allow previous weight somewhat
        err   = pow(err,1.5);
 return (float) alpha + C_0 / (err + eps0 );
}

/** Return inliers-weighted error. Assumes things about how Modelfitter Error relates
  * to inliers, when ModelFitter is a generic class. Makes sense for SVD as it stands.
  * \return result.err + 1/result.inliers
  */
inline float PairErrorInliers(const AtomPair& pair, const PairNodeData& node_data ){
   return 10*pair.result().err()*pair.result().err()   + 10/(float)pair.result().inliers();
}

inline float PairErrorSimple(const AtomPair& pair, const PairNodeData& node_data){
    return  10*pair.result().err()*pair.result().err() ;
}

/** example of error function that is use something regarding the entire graph,
  * which would not be available to the modelfitter + fitter_result
  * \return result.error + increasing_function_of( depth_in_tree )
  */
inline float PairErrorAvoidCompassChain(const AtomPair& pair, const PairNodeData& node_data){ // big penalty to chaining compass atoms together
    float gamma0 = 0.0;
    if( node_data.mode      ==  true )
        gamma0 += 1e2;
    return pair.result().err() + gamma0;
}

struct PairNode{

    PairNode(PairMatchErrFPT error_func = PairErrorInliers,
     PairBlendConfFPT bconf_func = PairConfInvLaw ):
            error_func(error_func), bconf_func(bconf_func),
            node_data(INFINITY,0,false) {  }

    PairNode( const PairNode& node ) : error_func(node.error_func),
     bconf_func(node.bconf_func), node_data(node.node_data) { }

    PairMatchErrFPT     error_func;
    PairBlendConfFPT    bconf_func;
    PairNodeData node_data;

    cv::Ptr<ImageAtom> min_prev; /* The N-1 optimal atom for sub-problem
                                         terminating at 'this' Nth node */
    cv::Ptr<ImageAtom> atom;     /* 'this' Nth node in dynamic-programming-like traversal  */

    bool operator < (const PairNode& rhs){
        return node_data.distance < rhs.node_data.distance;
    }

    /** Give the 'other' Atom in the incoming pair a chance to occupy the role
      * of N-1 for the N-chain ending at 'this' atom, if it further minimizes this node's error.
      */
    void setDist(const AtomPair& pair, const PairNode& prev_node);

    /** Pipe the whole structure of Atoms and associated PairNodes to a graphviz text file
      * that can be made into a nice display with 'dot', 'neato', etc.
      */
    static void graphviz_dump(std::ostream& out, const std::map<cv::Ptr<ImageAtom>,PairNode>&  node_map);

    /** dump the nodes and also all pair matches with error values before
      * and after having trinsics set.
      */
    static void graphviz_dump_all(std::ostream& out, const std::map<cv::Ptr<ImageAtom>,PairNode>&  node_map);

};


/** encapsulate dumping node meta-data along with the pair of atoms it contains
  */
std::ostream& operator << (std::ostream& out, const PairNode& node);

/** encapsulate dumping pair-only, less meta-data than a PairNode
  */
std::ostream& operator << (std::ostream& out, const AtomPair& pair);


}

#endif
