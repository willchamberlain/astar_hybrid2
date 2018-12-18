#include <Eigen/Dense>
#include <iostream>

namespace vos {
    namespace planning { 
        namespace astar_hybrid {
            class Planner {
                public: 
                    Eigen::ArrayXXf plan(const Eigen::ArrayXXf &a_map_, const Eigen::Array3f &start_posn, const Eigen::Array3f &goal_posn );
                    void verify();
            };

            
            Eigen::ArrayXXf Planner::plan(const Eigen::ArrayXXf &a_map_, const Eigen::Array3f &start_posn, const Eigen::Array3f &goal_posn ) {
                return a_map_;                
            }

            void Planner::verify() {
                std::cout << "OK : vos::planning::astar_hybrid::Planner" << std::endl ;
            }
        }
        namespace display {
            class Map_Display {

            };

            class Plan_Display {

            };
        }
    }

}


