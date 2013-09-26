/*
 * xares.hpp
 *
 * eXploration module for Autonomous Robotic Embedded Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-19
 * license: BSD
 */
#ifndef XARES_HPP
#define XARES_HPP

#include <vector>

#include "gladys/weight_map.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/frontier_exploration.hpp"

#ifndef YAW_DIFF_SENSIBILITY
#define YAW_DIFF_SENSIBILITY 0.8 // about PI/4, max diff being Pi
#endif

namespace xares {

/* Decison making for frontier attirbutes :
 *
 * Use to sort attributes (thus frontiers) to plan exploration.
 * The lesser, the better.
 *
 */
struct {//{{{
    bool operator()( gladys::f_attributes l, gladys::f_attributes r) {

        // First criteria is proximity (the lesser, the better)
        if (l.proximity == r.proximity ) {
            // 2nd criteria is yaw difference :
            if ( fabs( l.yaw_diff - r.yaw_diff) < YAW_DIFF_SENSIBILITY )
                // 3rd & 4th criterias are :
                // - size (the bigger, the better)
                // - and cost (distance) : the cheaper, the better
                // We use a ratio
                //return (r.size * r.size * l.cost) < ( l.size * l.size * r.cost);
                return (r.size * l.cost) < ( l.size * r.cost);
            return ( l.yaw_diff < r.yaw_diff );
        }
        return l.proximity < r.proximity ;
    }
} decision_making;//}}}

/*{{{ xares class
 ****************************************************************************/
/* xares class */
class xares{//{{{

private :
    /* internal data */
    //const gladys::weight_map& wm ;                    // the weight map (data)
    gladys::nav_graph ng ;                              // nav graph (from wm)
    gladys::frontier_detector fd ;                      // frontier detector (from ng)

    std::vector< gladys::points_t > frontiers ;         // the list of the frontiers
    std::vector< gladys::f_attributes > attributes ;    // the frontiers attributes

    gladys::f_attributes goal ;                         // the current goal chosen by the planner

    /* internal parameters */
    size_t max_nf = 10 ;                                // max nbr of frontiers to consider
    size_t min_size = 2 ;                               // minimal size of the frontiers to consider
    gladys::frontier_detector::algo_t algo = gladys::frontier_detector::WFD ; // algo use to compute frontiers
    double min_dist = 1.6 ;                             // minimal cost to the frontiers to consider (meter*[1-100])
    double max_dist = 50.0 ;                            // maximal cost to the frontiers to consider (meter*[1-100])

    bool dump = false ;                                 // enable/disable dumping 
    std::string dumpdir = "/tmp" ;                           // path where to dump the data

    /* hidden computing functions */

public:
    typedef enum { XARES_SUCCESS, XARES_NO_FRONTIER, XARES_FAILURE} return_value;

    /** xares constructor
     *
     * Create a frontier_detector which loads the given weight map
     *
     * @param wm the weight_map (gladys)
     *
     */
    xares( const gladys::weight_map& _wm ) ;

    /** reload new nav_graph and frontier detector
     * (from the same weight_map, assuming it has changed)
     *
     * Reset internal data (keep the internal parameters)
     *
     */
    //void reload() ;

    /** set the internal parameters
     *
     * Keep the internal data.
     *
     * @param _max_nf : max nbr of frontiers to consider
     *
     * @param _min_size : minimal size of the frontier to consider
     *
     * @param _algo : algo use to compute frontiers
     *
     */
    int set_params(//{{{
        size_t _max_nf = 10,
        size_t _min_size = 2, 
        double _min_dist = 1.6,
        double _max_dist = 50.0,
        gladys::frontier_detector::algo_t _algo = gladys::frontier_detector::WFD
    ){
        max_nf      = _max_nf ;
        min_size    = _min_size ;
        min_dist    = _min_dist ;
        max_dist    = _max_dist ;
        algo        = _algo ;
    }//}}}

    void enable_dump(){ dump = true; };

    void disable_dump(){ dump = false; };

    /** plan a goal and a path
     *
     * Compute the frontiers with the given algorithm and parameters.
     * Compute their attributes.
     * Choose a goal.
     *
     * @param r_pos : the position of all the robot in the team. The first one
     * is assume to be the robot running the algorithm.
     *
     * @param yaw : the yaw of the robot running the algorithm.
     *
     */
    return_value plan( const gladys::points_t &r_pos, double yaw );

    /* getters */
    std::array<double, 4> get_transform() const {//{{{
        //TODO take "North up" into account
        std::array<double, 4> transform;
        const gladys::weight_map& map = fd.get_graph().get_map() ;

        transform[0] = map.get_scale_x() ;
        transform[1] = map.get_scale_y() ;
        transform[2] = map.get_utm_pose_x() ;
        transform[3] = map.get_utm_pose_y() ;

        return transform;
    }//}}}
    const std::vector< gladys::f_attributes >&  get_attributes() const {//{{{
        return attributes;
    }//}}}
    const gladys::f_attributes& get_goal() const {//{{{
        return goal;
    }//}}}

};//}}}

} // namespace  xares

#endif // XARES_HPP

