#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace snap_optimizer{

    using namespace std;
    using namespace Eigen;

    struct TrajPoint
    {
        Matrix<double, 6, 1> state;

        Vector3d input;
        double time;
    };

    class snap
    {
        public:
            
            void SnapOptimize(const vector<Matrix<double, 10, 1>> &state_list);

            snap(ros::NodeHandle &nh);
            

        protected:

            /* param */
            int dev_order = 4;
            int p_order;
            int p_num;

            /* Traj data */
            vector<TrajPoint> path_data;     //segment data
            int max_Seg;

            /* snap data */
            MatrixXd PolyCoeff;
            inline void scale();
            int SegNumber;
            MatrixXd Q, M, M_inv, M_inv_t, C_t, C;
            MatrixXd R, R_pp, R_fp; 
            VectorXd d_f, d_p;
            int num_d_f, num_d_p;

            /* snap fun */
            void PolyClosedSolve();
            void getQ();
            void getM();
            void getC_t();
            void getd_f(int index);

            /* helper fun */
            inline int Factorial(int k);

            /* pub */
            ros::Publisher snap_pub;
            string topic = "snap_closed_vis";
            double vis_traj_width;
            visualization_msgs::Marker _traj_vis;
            void Pubsnap();
            Vector3d getPosPoly( int k, double t );
            
    };

    inline void snap::scale()
    {    
       
        for (auto &point : path_data)
        {
            point.state.head(3) = point.state.head(3) * 10;
        }        
        
    }

    inline int snap::Factorial(int k)
    {
        int fac = 1;
        for(int i = k; i > 0; --i)
            fac = fac * i;
        return fac;
    }
}
