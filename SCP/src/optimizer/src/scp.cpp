#include "manager/scp.h"
#include <iostream>

namespace SCP
{

    scp::scp(ros::NodeHandle & nh)
    {
        current_sub = nh.subscribe("/map_generator/current", 1, &scp::currentCallback, this);
        scp_nlp_vis = nh.advertise<visualization_msgs::Marker>("scp_nlp_vis",1);

        nh.param("/map/x_size", x_size, 20.0);
        nh.param("/map/y_size", y_size, 20.0);
        nh.param("vis_traj_width", vis_traj_width, 0.02);


        // Create an instance of  nlp  涉及虚继承 以及 继承的多态性  
        ptop = new nlp::ipopt_interface(nh);

        // Create an instance of the IpoptApplication
        app = IpoptApplicationFactory();

        // 设置MUMPS库使用的内存百分比
        // app->Options()->SetIntegerValue("mumps_mem_percent", 2);

        // 设置MUMPS库的主元选取阈值
        // app->Options()->SetNumericValue("mumps_pivtol", 3);

        //设置迭代的最大次数
        app->Options()->SetIntegerValue("max_iter", 2000);

        //设置收敛容许误差
        app->Options()->SetNumericValue("tol", 5);

        //设置可接受的约束违反容许误差
        app->Options()->SetNumericValue("acceptable_constr_viol_tol",6);

        //设置最小海森矩阵扰动值
        // app->Options()->SetNumericValue("min_hessian_perturbation",7);

        //设置雅可比矩阵正则化值
        // app->Options()->SetNumericValue("jacobian_regularization_value",8);

        //设置Y向量的alpha值
        // app->Options()->SetStringValue("alpha_for_y", "10");

        //设置是否重新计算Y向量
        // app->Options()->SetStringValue("recalc_y", "11");

        //设置初始mu值
        // app->Options()->SetNumericValue("mu_init",12);

        Ipopt::ApplicationReturnStatus status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) 
        {
            std::cout << "*** Distance Approach problem error during initialization!"<<endl;
        }
    }

    void scp::currentCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        if(has_current)
            return;

        // 先将 map 信息存储
        current_info.clear();
        for (int i = 0; i < msg->poses.size(); ++i)
        {
            current tmp;
            Matrix<double, 2, 1> pos;
            tmp.x = pos(0) = msg->poses[i].position.x;
            tmp.y = pos(1) = msg->poses[i].position.y;

            tmp.c_x = msg->poses[i].orientation.x;
            tmp.c_y = msg->poses[i].orientation.y;
        
            current_info.insert(make_pair(pos, tmp));
        }

        // 用  pcl 的 KDT-tree 实现最近位置索引 
        cloudMap.clear();
        pcl::PointXYZ pt;

        for (int i = 0; i < msg->poses.size(); ++i)
        {            
            pt.x = msg->poses[i].position.x;
            pt.y = msg->poses[i].position.y;
            pt.z = 0;
        
            cloudMap.points.push_back(pt);
        }
        kdtreeMap.setInputCloud(cloudMap.makeShared());

        has_current = true;
        
    }

    void scp::OptimizeScp(const vector<Matrix<double, 6, 1>> &state_list)  
    {
        // 因为海流同方向，所以先不用SCP，直接全局范围内解。但怎么跨越障碍物这是个事
        // 对于状态约束，应该在次处线性化，直接传一个线性约束

        state_all = std::move(state_list);
        while (!stop())
        {            
            get_state();

            vector<Matrix<double, 4, 1>> currents;
            get_current(currents);

            vector<Matrix<double, 4, 1>> bounds; //除去起点和终点的位置SCP，状态就再说吧
            get_bounds(bounds);

            ptop->init_path_info(traj_list, bounds, currents);

            Ipopt::SmartPtr<Ipopt::TNLP> nlp_solver = ptop;                   //双层多态应该在初始化完毕后，再传给 TNLP 吧

             // Initialize the IpoptApplication and process the options
            Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(nlp_solver);

            if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level || true) 
            {
                // Retrieve some statistics about the solve
                Ipopt::Index iter_count = app->Statistics()->IterationCount();
                printf("\n\n*** The problem solved in %d iterations!\n", iter_count);

                Ipopt::Number final_obj = app->Statistics()->FinalObjective();
                printf("\n\n*** The final value of the objective function is %e.\n", final_obj);

                pub_traj();

                break;   //只解一次
            }

           
        }
        
    }

    bool scp::stop()
    {

        return false;
    }

    void scp::get_state()
    {
        //先全部解
        traj_list = state_all;
    }

    void scp::get_current(vector<Matrix<double, 4, 1>> &currents)
    {
        for (int i = 0; i < traj_list.size(); ++i)
        {
            pcl::PointXYZ searchPoint(traj_list[i](0), traj_list[i](1), 0);
            vector<int>     pointIdxSearch;
            vector<float>   pointSquaredDistance; 
            if ( kdtreeMap.nearestKSearch (searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0 )
            {
               int nearestNeighborIndex = pointIdxSearch[0]; // 获取第一个最近邻点的索引

                // 通过索引访问点云中的对应点
                pcl::PointXYZ nearestNeighborPoint = cloudMap.points[nearestNeighborIndex];
                Matrix<double, 2, 1> pos(nearestNeighborPoint.x, nearestNeighborPoint.y);
                auto iter = current_info.find(pos);

                Matrix<double, 4, 1> tmp;
                tmp.head(2) = iter->first;
                tmp(2) = iter->second.c_x;
                tmp(3) = iter->second.c_y;
                currents.push_back(tmp);
            }
            else
            {
                cout<<"nearst_current error"<<endl;
            }
        }
    }

    void scp::get_bounds(vector<Matrix<double, 4, 1>> &bounds)
    {
        for (int i = 0; i < traj_list.size() - 2; ++i) //除去起点和终点的位置SCP
        {
             Matrix<double, 4, 1> tmp;
             tmp(0) = - x_size / 2;
             tmp(1) = x_size / 2;
             tmp(2) = - y_size / 2;
             tmp(3) = y_size / 2;
             bounds.push_back(tmp);
        }
    }

    void scp::pub_traj()
    {
        int x_num = traj_list.size();
        int u_num = traj_list.size() - 1;

        MatrixXd state_result_ = MatrixXd::Zero(4, x_num);        
        MatrixXd control_result_ = MatrixXd::Zero(2, u_num);; 
        double time_result_;
        ptop->get_opt_result(state_result_, control_result_, time_result_);


        visualization_msgs::Marker node_vis; 
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();    

        node_vis.ns = "scp/scp_nlp_vis";

        node_vis.type = visualization_msgs::Marker::LINE_STRIP;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        node_vis.color.a = 1.0;    //不透明度为1.0
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0; 
        node_vis.color.b = 0.0;

        node_vis.scale.x = vis_traj_width + 0.03;
        node_vis.scale.y = vis_traj_width + 0.03;
        node_vis.scale.z = vis_traj_width + 0.03;

        
        geometry_msgs::Point pt1;
        for(int i = 0; i < traj_list.size(); ++i)
        {
            pt1.x = state_result_(0, i);            
            pt1.y = state_result_(1, i); 
            pt1.z = 0.0;

            node_vis.points.push_back(pt1);
        }

        scp_nlp_vis.publish(node_vis);

    }
}

