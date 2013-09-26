/*
 * replay.cpp
 *
 * eXploration module for Autonomous Robotic Embedded Systems
 *
 * xares replay : replay dump from xares (and xares-genom)
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-19
 * license: BSD
 */

#include <ostream>
#include <fstream>
#include <sys/time.h>

#include "xares/xares.hpp"
#include "gladys/point.hpp"
#include "gladys/weight_map.hpp"

int replay_genom_dump( const std::string& path_to_dump_file) {//{{{

  std::ifstream dump_file;
  dump_file.open (path_to_dump_file) ;

  if ( dump_file.is_open () ) {
    // init
    struct timeval tv0, tv1;

    std::string line, name ;
    gladys::points_t r_pos;
    int n, max_nf ;
    size_t min_size ;
    double x, y, yaw, min_dist, max_dist ;

    //read data
    getline (dump_file,line) ;
    std::istringstream iss1(line);
    iss1 >> name >> n ;
    std::cerr << "[Xares replay] r_pos ["<< n << "] =";
    while ( n-- > 0) {
      iss1 >> x >> y ;
      r_pos.push_back( gladys::point_xy_t {x,y} );
      std::cerr << " ("<< x << "," << y << ")";
    }
    std::cerr << std::endl;

    getline (dump_file,line) ;
    std::istringstream iss2(line);
    iss2 >> name >> yaw ;
    std::cerr << "[Xares replay] yaw = " << yaw << std::endl;

    // load and display internal params
    getline (dump_file,line) ;
    std::istringstream iss3(line);
    iss3 >> name >> max_nf >> min_size >> min_dist >> max_dist ;
    std::cerr << "[Xares replay] internal params "
              << "(max_nf,min_size,min_dist,max_dist) = ("
              << max_nf << "," << min_size << "," << min_dist << "," << max_dist 
              << ")" << std::endl;

    std::cerr << "[Xares replay] max nbr of frontiers = " << max_nf << std::endl;
    getline (dump_file,line) ;
    gladys::weight_map wm ( line );

    dump_file.close();

    // weightmap : origin, size, scale
    std::cerr << "[Xares repay] (x0,y0,xS,yS,W,H) = ("
              << wm.get_utm_pose_x()<< ","
              << wm.get_utm_pose_y()<< ","
              << wm.get_scale_x()   << ","
              << wm.get_scale_y()   << ","
              << wm.get_height()    << ","
              << wm.get_width()     << ")"
              << std::endl;

    // load the planner
    gettimeofday(&tv0, NULL);
    xares::xares xp( wm );
    gettimeofday(&tv1, NULL);

    std::cerr << "[Xares replay] planner loaded ("
              << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
              (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

    // Set internal parameters for xares
    xp.set_params( max_nf, min_size, min_dist, max_dist );
    std::cerr << "[Xares replay] parameters settled. Let's plan !" << std::endl;

    //plan (=replay !)
    gettimeofday(&tv0, NULL);
    xares::xares::return_value rv = xp.plan( r_pos, yaw );
    gettimeofday(&tv1, NULL);

    // parse results
    if ( rv == xares::xares::XARES_FAILURE ) {
      std::cerr << "[Xares replay] #EEE# xares : unexpected data ; unable to compute frontiers :'( " << std::endl;
      return EXIT_SUCCESS;
    }

    if ( rv == xares::xares::XARES_NO_FRONTIER ) {
      std::cerr << "[Xares replay] #EEE# xares : no valuable frontier detected." << std::endl;
      return EXIT_SUCCESS;
    }

    // From here, there should be a valid plan
    std::cerr << "[Xares replay] Plan computed (" 
              << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
              (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)" << std::endl;

    //get planned path
    const gladys::path_t& path = xp.get_goal().path ;

    for (unsigned int i = 0 ; i < path.size() ; i++)
      std::cerr   << "[Xares replay ] ----waypoint #" << i 
                  << " = (" << path[i][0]<< "," << path[i][1] 
                  <<")"<<std::endl;

    // get poster path
    gladys::path_t::const_iterator it;
    double dist = 0.0;

    gladys::point_xy_t curr = path.front() ;

    unsigned int pt = 0 ; 
    for ( it = path.begin(); it != path.end() ; ++it)
    {
      dist += gladys::distance( curr, *it );
      curr = *it;

      if ( dist > 6.0 ) {
        std::cerr   << "[Xares replay ] ---- poster waypoint #" << pt++
                    << " = (" << (*it)[0]<< "," << (*it)[1] 
                    <<")"<<std::endl;
      }
    }

    std::cerr   << "[Xares replay ] ---- poster waypoint #" << pt 
                << " = (" << curr[0]<< "," << curr[1] 
                <<")"<<std::endl;


    // ASCII plot of weightmap, seed & goal
    std::cerr   << "[Xares replay ] ASCII weight map : " << std::endl; 
    // foreach from (origin) to (dim+origin)
    for (int j = wm.get_utm_pose_y(); j < wm.get_width()  + wm.get_utm_pose_y(); j++)
    {
      if ( j < -21 || j > 19 ) continue; //cropping
      for (int i = wm.get_utm_pose_x(); i < wm.get_height() + wm.get_utm_pose_x(); i++)
      {
        if ( i < 0 || i > 60 ) continue; //cropping
        // Special points
        // O = origin
        // S = Seed
        // G = Goal
        if ( i == 0 && j == 0 ) { // origin
            std::cerr << "O"; continue; }
        if ( i == r_pos[0][0] && j == r_pos[0][1] ) { // seed
            std::cerr << "S"; continue; }
        if ( i == curr[0] && j == curr[1] ) { // goal
            std::cerr << "G"; continue;}

        // Common points
        // u = unknown
        // . = known
        // # = obstacle
        double w = wm.get_weight_band()[ wm.idx_utm( gladys::point_xy_t {(double)i,(double)j} )];
        if        (w == 100)  std::cerr << "u";
        else if   (w > 100 )  std::cerr << "#";
        else                  std::cerr << ".";

      }
      std::cerr << std::endl;
    }
  }
  else {
    std::cerr << "[Xares replay] Cannot open provided dump file. :-(" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}//}}}

int replay_lib_dump(char dumpFile[]) {//{{{

  std::cerr << "Not yet implemented..." << std::endl;

  return EXIT_SUCCESS;

}//}}}

int main(int argc, char **argv) {//{{{

  int return_value ;
  return_value = replay_genom_dump( argv[1] ) ;

  std::cerr << "[Xares replay] Done." << std::endl;

  return return_value;

}//}}}

