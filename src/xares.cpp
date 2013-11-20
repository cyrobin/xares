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
#include <limits>       // for numeric_limits::infinity

#include "xares/xares.hpp"

namespace xares {

/*{{{ xares class
 ****************************************************************************/
    /* constructor */
    xares::xares( const gladys::weight_map& _wm, //{{{
                  double x_origin,
                  double y_origin,
                  double height_max,
                  double width_max ) :
        //wm(_wm), 
        ng ( _wm),
        fd ( ng, x_origin, y_origin, height_max, width_max )
    {
      max_nf = 10 ;                                // max nbr of frontiers to consider
      frontier_min_size = 2.0 ;                    // minimal size (meters) of the frontiers to consider
      frontier_max_size = 30.0 ;                   // minimal size (meters) of the frontiers to consider
      algo = gladys::frontier_detector::WFD ;      // algo use to compute frontiers
      min_dist = 1.6 ;                             // minimal cost to the frontiers to consider ( distance in meters * [1-100])
      max_dist = 50.0 ;                            // maximal cost to the frontiers to consider ( distance in meters * [1-100])

      dump = false ;                               // enable/disable dumping 
      dumpdir = "/tmp" ;                           // path where to dump the data

    }//}}}

    /* computing functions */
//
//    void xares::reload( ) {//{{{
//          //TODO
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

    xares::return_value xares::plan( const gladys::points_t &r_pos, double yaw ) {//{{{
        /* compute frontiers and attributes */
        std::cerr << "[Xares] Computing frontiers... " << std::endl ;
        try {
            fd.compute_frontiers( r_pos, yaw, max_nf, frontier_min_size, frontier_max_size, \
                    min_dist, max_dist, algo ) ;
        } catch (std::exception& e) {
            std::cerr << "[Xares] catch exception : " << e.what() << std::endl ;
            std::cerr << "[Xares] Fail to compute the frontiers: please check your data." << std::endl ;
            return XARES_FAILURE;
        }
        std::cerr << "[Xares] frontiers computed ! :-) " << std::endl ;

        attributes = fd.get_attributes() ;

        std::cerr << "[Xares] Got #"<< attributes.size() <<" attributes. Deciding... " << std::endl ;

        // check is there are indeed valuable frontiers
        if ( attributes.size() == 0 )
            return XARES_NO_FRONTIER;
        // TODO check if NO_Frontier is a pb, or if exploration is done
        // TODO return a % of discovery / exploration done.

        /* choose one */
        // sort the frontiers attributes according to a specific decision criteria
        std::sort( attributes.begin(), attributes.end(), decision_making );

        // Verbose : display sorted frontiers attributes
        for ( auto a : attributes )
            std::cerr << a << std::endl;

        goal = attributes.front() ;

        /* transform coordinates into the global references */

        /* the end */
        std::cerr << "[Xares] Goal is : "<< goal << std::endl ;
        std::cerr << "[Xares] Done." << std::endl ;

        return XARES_SUCCESS;

    }//}}}

} // namespace  xares

