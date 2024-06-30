#include "optimizer/snap_closed_form.h"

#include <iostream>

namespace snap_optimizer{

    snap::snap(ros::NodeHandle &nh)
    {
        nh.param("max_Seg", max_Seg, 25);
        nh.param("snap/dev_order", dev_order, 4);
        nh.param("vis_traj_width", vis_traj_width, 0.02);

        p_order = 2 * dev_order -1;
        p_num = p_order + 1;

        snap_pub = nh.advertise<visualization_msgs::Marker>(topic, 1);
    }

    void snap::SnapOptimize(const vector<Matrix<double, 10, 1>> &state_list)
    {
        path_data.clear();
        TrajPoint tmp;
        for (auto &point : state_list)
        {
            tmp.state = point.head(6);
            tmp.input = point.block(6, 0, 3, 1);
            tmp.time = point(9);
            path_data.push_back(tmp);
            // ROS_INFO("position x: %f, y: /%f, t: %f", point(0), point(1), point(9));
            // ROS_INFO("v x: %f, y: /%f, radio_v: %f,  radio_p: %f", point(3), point(4), point(3)/ point(4), point(0)/ point(1));
        }

        ROS_INFO("minSnap optimize %d nodes", path_data.size());            
        SegNumber = path_data.size() -1;

        scale();

        PolyClosedSolve();

        Pubsnap();

    }

    void snap::PolyClosedSolve()
    {
        getQ();

        getM();

        getC_t();

        R = C * M_inv_t * Q * M_inv * C_t;

        num_d_f = 2 * dev_order + SegNumber - 1;
        num_d_p = (SegNumber - 1) * (dev_order - 1);

        MatrixXd R_pp = R.bottomRightCorner(num_d_p, num_d_p);
        MatrixXd R_fp = R.topRightCorner(num_d_f, num_d_p);

        PolyCoeff = MatrixXd::Zero(SegNumber, 3 * p_num);

        /* 3维进行snap优化 */
        for (int i = 0; i < 3; ++i)
        {
            // d_f
            getd_f(i);

            d_p = - 1.0 * R_pp.inverse() * R_fp.transpose() * d_f;

            VectorXd d_tot(d_f.rows() + d_p.rows());
            d_tot << d_f, d_p;
            // cout << "d_tot is:" << endl << d_tot << endl;

            //多项式系数
            VectorXd poly = M_inv * C_t * d_tot;
            // cout << "M_inv :" << endl << M_inv << endl;
            // cout << "poly :" << endl << poly << endl;
            MatrixXd poly_coef_1d_t = poly.transpose();
            for(int k = 0; k < SegNumber; ++k)            
                PolyCoeff.block(k, i * p_num, 1, p_num) = poly_coef_1d_t.block(0,k * p_num, 1, p_num);
            // cout<< "Dimension " << i+1 <<" poly_coef_1d_t: "<< endl << poly_coef_1d_t << endl;
        }
        // cout << "PolyCoeff :" << endl << PolyCoeff << endl;
    }

    /* object function */
    void snap::getQ()
    {
        Q = MatrixXd::Zero(SegNumber * p_num, SegNumber * p_num);
        for (int k = 0; k < SegNumber; ++k)
        {   
            MatrixXd Qk = MatrixXd::Zero(p_num, p_num);  
            for (int i = 0; i < p_num; ++i)
            {
                for (int j = 0; j < p_num; ++j)
                {
                    if(i < dev_order || j < dev_order)
                        continue;
                    Qk(i,j) = (Factorial(i) * Factorial(j))/(Factorial(i - dev_order) * Factorial(j - dev_order) * (i + j - 2 * dev_order + 1));
                }                
            }  
            Q.block(k * p_num, k * p_num, p_num, p_num) = Qk;           
        }   
        // cout << "Matrix Q is:\n"<< endl << Q << endl;
        //  cout<< "Q rows: "<< Q.rows() << " Q cols: "<< Q.cols() << "Q value" << Q.determinant()<<endl;
    }

    /* mapping derivative for numerical stability */
    void snap::getM()
    {
        M = MatrixXd::Zero(SegNumber * p_num, SegNumber * p_num);

        MatrixXd coeff(dev_order, p_num);

        if(dev_order == 4){
            coeff << 1,  1,  1,  1,  1,  1,  1,  1,
                    0,  1,  2,  3,  4,  5,  6,  7,
                    0,  0,  2,  6,  12, 20, 30, 42,
                    0,  0,  0,  6,  24, 60, 120,210;
        }
        else if(dev_order == 3){
            coeff << 1,  1,  1,  1,  1,  1,
                    0,  1,  2,  3,  4,  5,
                    0,  0,  2,  6,  12, 20;
        }
        else{
            ROS_INFO("This derivatieve order is not provided getM!!!");
        }

        for (int k = 0; k < SegNumber; ++k)
        { 
            MatrixXd M_k = MatrixXd::Zero(p_num, p_num);  
            for (int i = 0; i < dev_order; ++i)
            {
                M_k(i, i) = coeff(i, i);
            }

            M_k.block(dev_order, 0, dev_order, p_num) = coeff;

            M.block(k * p_num, k * p_num, p_num, p_num) = M_k;             
        } 

        // cout << "Mapping matrix M is:\n" << endl << M << endl;
        // cout<< "M rows: "<< M.rows() << " M cols: "<< M.cols() << "M value" << M.determinant()<<endl;

        M_inv = M.inverse();

        M_inv_t = M_inv.transpose();  
    }

    /* selection for closed form solution  因为C_t更符合逻辑，所以矩阵更容易写 */
    void snap::getC_t()
    {
        /*先分块，再换行是不可以的。因为换行会干扰其他的状态，只能一个一个值进行指定赋值 */
        int rows = dev_order * 2 * SegNumber;
        int cols = dev_order * 2 * SegNumber - (SegNumber - 1) * dev_order;

        C_t = MatrixXd::Zero(rows, cols);

        /* start point*/
        for (int i = 0; i < dev_order; ++i)
        {
            C_t(i, i) = 1;
        }
        
        /* final point*/
        int Frow_index = rows -dev_order;
        int Fcol_index = cols -dev_order;
        for (int i = 0; i < dev_order; ++i)  //final state 前移, 但随便前移矩阵会乱，所以final state 重新设置
        {
            
            C_t(Frow_index + i, dev_order + SegNumber - 1 + i) = 1;
        }
        // cout <<"Ct: \n"<< endl << C_t << endl;

        for (int i = 0; i < SegNumber - 1; ++i) 
        {
            int row = i * dev_order * 2 + dev_order;
            int col = i + dev_order;
            C_t(row, col) = 1;
            C_t(row + dev_order, col) = 1;
        }
        // cout <<"Ct: \n"<< endl << C_t << endl;

        /*inter v,a.. 后移*/
        for (int i = 0; i < SegNumber - 1; ++i)   
        {
            int row = i * dev_order * 2 + dev_order + 1;
            int col = i * (dev_order - 1) + 2 * dev_order + SegNumber - 1;

            for (int j = 0; j < dev_order - 1; ++j)
            {
                C_t(row + j, col + j) = 1;
                C_t(row + dev_order + j, col + j) = 1;
            }           
            
        }
        // cout <<"Ct: \n"<< endl << C_t << endl;

        C = C_t.transpose();
    }

    /* equal constraint */
    void snap::getd_f(int index)
    {
        d_f = VectorXd::Zero(num_d_f);
        for (int i = 0; i < path_data.size(); ++i)
        {
            //start point
            if(i == 0)
            {   
                d_f(0) = path_data[i].state(index); 
                d_f(1) = path_data[i].state(index + 3); 
                d_f(2) = path_data[i].input(index);

                if (dev_order > 3)
                {
                    d_f(3) = 0.0;
                }                
                
            }

            //中间点
             d_f(i + dev_order - 1) = path_data[i].state(index); 

            //final point
            if(i == path_data.size() - 1)
            {
                d_f(num_d_f - dev_order) = path_data[i].state(index); 
                d_f(num_d_f - dev_order + 1) = path_data[i].state(index + 3); 
                d_f(num_d_f - dev_order + 2) = path_data[i].input(index);                            
                if (dev_order > 3)
                {
                    d_f(3) = 0.0;
                }
            }
        }
        
    }

    void snap::Pubsnap()
    {
        _traj_vis.header.stamp       = ros::Time::now();
        _traj_vis.header.frame_id    = "/world";

        _traj_vis.ns = "snap/trajectory_waypoints";
        _traj_vis.id = 0;
        _traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
        _traj_vis.action = visualization_msgs::Marker::ADD;
        _traj_vis.scale.x = vis_traj_width;
        _traj_vis.scale.y = vis_traj_width;
        _traj_vis.scale.z = vis_traj_width;
        _traj_vis.pose.orientation.x = 0.0;
        _traj_vis.pose.orientation.y = 0.0;
        _traj_vis.pose.orientation.z = 0.0;
        _traj_vis.pose.orientation.w = 1.0;

        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 1.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 0.0;

        _traj_vis.points.clear();
        Vector3d pos;
        geometry_msgs::Point pt;


        for(int i = 0; i < SegNumber; ++i )
        {   
            double t_tot = path_data[i+1].time - path_data[i].time;
            for (double t = 0; t < 10; ++t)
            {   
                double start_t = path_data[i].time;             
                pos = getPosPoly(i, start_t+(t_tot/10)*t);
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                _traj_vis.points.push_back(pt);
            }
        }

        snap_pub.publish(_traj_vis);
    }

    Vector3d snap::getPosPoly(int k, double t )
    {
        Vector3d ret;
        double tmp_t = (t - path_data[k].time)/(path_data[k+1].time - path_data[k].time);
        for ( int dim = 0; dim < 3; ++dim)
        {
            VectorXd coeff = PolyCoeff.row(k).segment( dim * p_num, p_num);
            VectorXd time  = VectorXd::Zero(p_num);

            for(int j = 0; j < p_num; ++j)
            {
                if(j == 0)
                    time(j) = 1.0;
                else
                {                    
                    time(j) = pow(tmp_t, j);
                }
            }
            ret(dim) = coeff.dot(time);
            // cout << "dim:" << dim+1 << " coeff:" << endl << coeff << endl;
            // cout << "time:" << endl << time << endl;
        }
        ret = ret / 10;
        return ret;
    }
}
