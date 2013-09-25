/*
 * test_xares.cpp
 *
 * Test the eXploration module for Autonomous Robotic Embedded Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-19
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <sstream>

#include "gladys/gdal.hpp"          // required to create the models
#include "gladys/weight_map.hpp"    // required to create the models
#include "xares/xares.hpp"

BOOST_AUTO_TEST_SUITE( frontier )

BOOST_AUTO_TEST_CASE( test_frontier )
{
    std::string region_path = "/tmp/test_frontier_detection.tif";
    std::string weight_path = "/tmp/test_frontier_detection_weight.tif";
    std::string robotm_path = "/tmp/robot.json";

    // create a robot model (JSON configuration file)
    std::ofstream robot_cfg(robotm_path);
    robot_cfg<<"{\"robot\":{\"mass\":1.0,\"radius\":1.0,\"velocity\":1.0}}";
    robot_cfg.close();

    /* create a region map (GeoTiff image)
     *
     *  U = unknown
     *  F = flat
     *  O = obstacle
     *  S = seed (flat ; initial position)
     *
     *       1 2 3 4 5 6 7 8 9
     *
     *  1    U U U U U U U U U
     *  2    F F F F F F F F F
     *  3    F F F F F F F F F
     *  4    F F F F F F F F F
     *  5    F F F F S F F F F
     *  6    F F F O O O F F F
     *  7    F F F F F F F F F
     *  8    F F F F F F F F F
     *  9    U U U U O U U U U
     *
     */
    gladys::gdal region;
    region.set_size(gladys::weight_map::N_RASTER, 9, 9);
    region.bands[gladys::weight_map::FLAT    ].assign(9*9, 1);
    // add frontiers (top and bottom)
    for ( int i=0 ; i < 9 ; i++ ) {
        region.bands[gladys::weight_map::FLAT       ][i    ] = 0. ;
        region.bands[gladys::weight_map::NO_3D_CLASS][i    ] = 1  ;
        region.bands[gladys::weight_map::FLAT       ][i+8*9] = 0. ;
        region.bands[gladys::weight_map::NO_3D_CLASS][i+8*9] = 1  ;
    }
    // add ostacle #1 (middle)
    region.bands[gladys::weight_map::FLAT    ][3+5*9] = 0.2 ;
    region.bands[gladys::weight_map::OBSTACLE][3+5*9] = 0.8 ;
    region.bands[gladys::weight_map::FLAT    ][4+5*9] = 0.2 ;
    region.bands[gladys::weight_map::OBSTACLE][4+5*9] = 0.8 ;
    region.bands[gladys::weight_map::FLAT    ][5+5*9] = 0.2 ;
    region.bands[gladys::weight_map::OBSTACLE][5+5*9] = 0.8 ;
    // add ostacle #2 (bottom, centered)
    region.bands[gladys::weight_map::FLAT    ][4+8*9] = 0.2 ;
    region.bands[gladys::weight_map::OBSTACLE][4+8*9] = 0.8 ;

    region.save(region_path);

    // create a frontier exploration module from the map
    // (Create the weight_map, assumed to be good; cf other unit test)
    gladys::weight_map wm( region_path, robotm_path ) ;
    xares::xares xm ( wm ) ;

    // testing frontier detection with defult algorithm
    gladys::point_xy_t r1 {4,4};
    gladys::point_xy_t r2 {4,2};
    gladys::points_t r_pos {r1, r2} ;
    double yaw = 0 ;
    xm.plan( r_pos, yaw ) ;

    std::vector< gladys::f_attributes > attributes = xm.get_attributes() ;

   //Check the number of frontiers
   BOOST_TEST_MESSAGE( "Nbr of frontiers : attributes.size() = " << attributes.size() );
   BOOST_CHECK_EQUAL( attributes.size() , 2 );

   size_t c = 0 ;
   for ( auto& f : attributes )
       c += f.size ;
   BOOST_TEST_MESSAGE( "Nbr of frontier points : c = " << c );
   BOOST_CHECK_EQUAL( c , 18 );

   // Goal should be (4,7) (the bottom frontier) and path size = 7
   gladys::point_xy_t goal = xm.get_goal() ;
   BOOST_TEST_MESSAGE( "Goal is ? " <<  goal[0] << "," << goal[1] );
   BOOST_CHECK_EQUAL( goal[0], 4 );
   BOOST_CHECK_EQUAL( goal[1], 7 );
   BOOST_TEST_MESSAGE( "Path to the goal has size : " << xm.get_path().size() );
   BOOST_CHECK_EQUAL( xm.get_path().size(), 7 );


}


BOOST_AUTO_TEST_SUITE_END();

