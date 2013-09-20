/*
 * xares.cpp
 *
 * eXploration module for Autonomous Robotic Embedded Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-19
 * license: BSD
 */

#include "xares/xares.hpp"

namespace xares {

/*{{{ xares class
 ****************************************************************************/
    /* constructor */
    xares::xares() { }

    xares::xares( const std::string& f_region, const std::string& f_robot_model ) {//{{{
        fd = gladys::frontier_detector( f_region, f_robot_model );
    }//}}}

    /* computing functions */
    void xares::load( const std::string& f_region, const std::string& f_robot_model ) {//{{{
        /* reset internal data */
        frontiers.clear() ;
        attributes.clear() ;
        path.clear() ;
        //goal ;

        /* load new module */
        fd = gladys::frontier_detector( f_region, f_robot_model );

    }//}}}

    void xares::plan( const gladys::points_t &r_pos) {//{{{
        /* compute frontiers and attributes */
        fd.compute_frontiers( r_pos, max_nf, min_size, algo ) ;

        attributes = fd.get_attributes() ;

        /* choose one */
        if ( attributes.size() > 0 ) {
            // sort the frontiers attributes according to a specific decision criteria
            std::sort( attributes.begin(), attributes.end(), decision_making );

            goal = attributes.front().lookout ;
            path = attributes.front().path ;
        }

    }//}}}

} // namespace  xares

