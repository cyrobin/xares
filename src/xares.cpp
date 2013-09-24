/*
 * xares.cpp
 *
 * eXploration module for Autonomous Robotic Embedded Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-19
 * license: BSD
 */

#include <ostream>
#include <exception>

#include "xares/xares.hpp"


namespace xares {

/*{{{ xares class
 ****************************************************************************/
    /* constructor */
    xares::xares( const gladys::weight_map& _wm ) : //{{{
        //wm(_wm), 
        ng ( _wm),
        fd ( ng )
    { }//}}}

    /* computing functions */
//
//    void xares::reload( ) {//{{{
//        /* reset internal data */
//        frontiers.clear() ;
//        attributes.clear() ;
//        path.clear() ;
//        //goal ;
//
//        /* load new module */
//        ng = gladys::nav_graph( wm );
//        fd = gladys::frontier_detector( ng );
//
//    }//}}}
//

    void xares::plan( const gladys::points_t &r_pos) {//{{{
        /* compute frontiers and attributes */
        std::cerr << "[Xares] Computing frontiers... " << std::endl ;
        try {
            fd.compute_frontiers( r_pos, max_nf, min_size, algo ) ;
        } catch (std::exception& e) {
            std::cerr << "[Xares] catch exception : " << e.what() << std::endl ;
            std::cerr << "[Xares] Fail to compute the frontiers: please check your data." << std::endl ;
            return;
        }
        std::cerr << "[Xares] frontiers computed ! :-) " << std::endl ;

        attributes = fd.get_attributes() ;

        std::cerr << "[Xares] I got the attributes. Deciding... " << std::endl ;

        /* choose one */
        if ( attributes.size() > 0 ) {
            // sort the frontiers attributes according to a specific decision criteria
            std::sort( attributes.begin(), attributes.end(), decision_making );

            goal = attributes.front().lookout ;
            path = attributes.front().path ;
        }
        std::cerr << "[Xares] Done." << std::endl ;

    }//}}}

} // namespace  xares

