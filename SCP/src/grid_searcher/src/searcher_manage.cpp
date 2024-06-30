#include"grid_searcher/searcher_manage.h"

#include<cmath>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include<tf2/LinearMath/Quaternion.h>
#include <iostream>

searcher_manage::searcher_manage(ros::NodeHandle &nh)
{
    nh.param("resoultion",resolution,0.2);
    nh.param("searcher_flag",searcher_flag,1);
    
    nh.param("use_car_search",use_car_search,true);
    nh.param("/map/x_size",x_size,20.0);  
    nh.param("/map/y_size",y_size,20.0);
    nh.param("/map/z_size",z_size,10.0);
    nh.param("/map/dim_flag",dim_flag,false);  

    nh.param("/map/start_pt_x",start_pt(0),0.0);  
    nh.param("/map/start_pt_y",start_pt(1),0.0);
    nh.param("/map/start_pt_z",start_pt(2),0.0);

    nh.param("/map/start_vel_x",start_vel(0),0.0);  
    nh.param("/map/start_vel_y",start_vel(1),0.0);
    nh.param("/map/start_vel_z",start_vel(2),0.0);

    nh.param("/map/start_acc_x",start_acc(0),0.0);  
    nh.param("/map/start_acc_y",start_acc(1),0.0);
    nh.param("/map/start_acc_z",start_acc(2),0.0);

    nh.param("/map/end_vel_x",end_vel(0),0.0);  
    nh.param("/map/end_vel_y",end_vel(1),0.0);
    nh.param("/map/end_vel_z",end_vel(2),0.0);   

    nh.param("vis_kino_interval", vis_kino_interval ,0.6);  
    nh.param("sam_kino_interval", sam_kino_interval, 0.3);   

    inv_resoultion=1/resolution;

    GLX_SIZE=static_cast<int>(x_size*inv_resoultion);
    GLY_SIZE=static_cast<int>(y_size*inv_resoultion);
    GLZ_SIZE=static_cast<int>(z_size*inv_resoultion);

    gl_xl=-x_size/2.0;
    gl_yl=-y_size/2.0;
    gl_zl=0.0;

    has_map=false;
    has_map_ob=false;    

    XYbounds.emplace_back(gl_xl);
    XYbounds.emplace_back(x_size/2.0);
    XYbounds.emplace_back(gl_yl);
    XYbounds.emplace_back(y_size/2.0);

    if (searcher_flag==1)     //建立kino_astar_finder
    {
        //unique_ptr 不能拷贝或赋值，所以用来指向类对象很适合
        kino_astar_finder_.reset(new kino_astar::kino_astar_finder);   
        kino_astar_finder_->init(nh);     //此时kino_astar_finder的参数位于传入的searcher_manage空间
    }   

    if(use_car_search)
    {
        PlannerOpenSpaceConfig config;
        kino_astar_car.reset(new HybridAStar(config));
        //shared 的构造函数是 explicte 的，只能直接初始化。直接初始化只能在构造函数列表里。不支持复杂对象的默认初始化
        //或者用make_shared 更安全
        result = make_shared<HybridAStartResult>(); 
    }

    GlobalMapSub = nh.subscribe("/map_generator/global_map",10,&searcher_manage::global_mapCallback,this);

    GlobalMapObSub = nh.subscribe("/map_generator/global_map_ob",10,&searcher_manage::global_mapObCallback,this);

    static_obs_sub = nh.subscribe("/map_generator/static_obs", 1,&searcher_manage::ObstaclesCallback, this);

    DisMapPub = nh.advertise<sensor_msgs::PointCloud2>("global_map_vis",1);

    kino_vis = nh.advertise<visualization_msgs::Marker>("kino_vis",1);

    kino_car_vis = nh.advertise<visualization_msgs::Marker>("kino_car_vis",1);

    goal_sub = nh.subscribe("/goal",1,&searcher_manage::goalCallback,this);

}

void searcher_manage::global_mapObCallback(const sensor_msgs::PointCloud2 &GlobalMap)
{
    if(has_map_ob)   
    {
        return;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> CloudMap;
        pcl::fromROSMsg(GlobalMap,CloudMap);
        int i = 0;
        std::vector<common::math::Vec2d> temp;

        //只能处理四边形
        for (auto &pt:CloudMap)
        {  
            temp.emplace_back(pt.x,pt.y);
            ++i;
            if (i == 4)
            {
                obstacles_vertices_vec.emplace_back(temp);
                temp.clear();
                i = 0;
            }                
        }
        has_map_ob = true;
        
    }
    
}

void searcher_manage::ObstaclesCallback(const global_map::ObstaclesConstPtr &msg)
{
    if(has_map_ob)
        return;

    obstacles_vertices_vec.clear();
    for (auto &obstacle: msg->obstacles) 
    {
        std::vector<common::math::Vec2d> points;
        for (auto &pt: obstacle.points) 
        {
            points.emplace_back(pt.x, pt.y);
        }
        obstacles_vertices_vec.emplace_back(points);
    }
    has_map_ob = true;
    has_map = true;
}

void searcher_manage::global_mapCallback(const sensor_msgs::PointCloud2 & GlobalMap)
{    
    if(has_map)   
    {
        // DisMapPub.publish(DisplayMap);//如果要去掉，一定要先开rviz，或添加nh成员以读取param，或订阅信息，否则rviz可能会接受不到
        return;
    }
    else
    {   
        pcl::PointCloud<pcl::PointXYZ> CloudMap;
        pcl::PointCloud<pcl::PointXYZ> CloudMapVis;

        pcl::fromROSMsg(GlobalMap,CloudMap);

        for (auto &pt:CloudMap)
        {
            /* 对ob实际索引乘分辨率后，若不为整数，则只能向上取整以代表一大块grid为ob，所以分辨率越高，也就越贴合实际ob大小，不用膨胀 */
            if (searcher_flag==1)
            {
                // ROS_INFO("org_x: %f, org_y: %f, org_z: %f",pt.x, pt.y, pt.z);
                kino_astar_finder_->SetObs(pt.x, pt.y, pt.z);
            }
            /* map_visulaize 设置成奇数，让ob在方格内，只是为了好看一些，实际map中的ob是grid交点，而不是可视化的点 */
            Eigen::Vector3d cor_round = coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = cor_round(2);
            CloudMapVis.points.push_back(pt);
            // ROS_INFO("vis_x: %f, vis_y: %f, vis_z: %f",pt.x, pt.y, pt.z);        
        }

        CloudMapVis.width    = CloudMapVis.points.size();
        CloudMapVis.height   = 1;
        CloudMapVis.is_dense = true;

        pcl::toROSMsg(CloudMapVis,DisplayMap);
        DisplayMap.header.frame_id="world";
        DisMapPub.publish(DisplayMap);

        has_map = true;
    }

}

void searcher_manage::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    has_path = false;
    has_car_path = false;
    // ROS_INFO("msgs: %s, x: %lf, y: %lf, z: %lf, rotx: %lf",msg->header.frame_id.c_str(),msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,msg->pose.orientation.z);
  
    end_pt(0)=msg->pose.position.x;
    end_pt(1)=msg->pose.position.y;
    end_pt(2)=msg->pose.position.z; 

    end_pt(0) = -9.3135;
    end_pt(1) = 6.1215;

    start_pt(0) = x_size/2.0 - 2;
    start_pt(1) = -y_size/2.0 + 2;

    // 从Pose消息中提取四元数
    Eigen::Quaterniond quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    // 转换为旋转矩阵
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    // 从旋转矩阵转换为欧拉角 (Z-Y-X顺序，即 roll, pitch, yaw)
    Eigen::Vector3d euler = rotationMatrix.eulerAngles(2, 1, 0);

    double pi = euler[0];

    pi = 2.52566;
    

    if(has_map)
    {
        if (searcher_flag==1)     //kino_astar_finder
        {            
            bool search_flag1 = kino_astar_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true); 
            ROS_INFO("start point x: %f, y: %f, z: %f", start_pt(0), start_pt(1), start_pt(2));
            ROS_INFO("end point x: %f, y: %f, z: %f", end_pt(0), end_pt(1), end_pt(2));
            if(search_flag1)
            {
                vector<Vector3d> traj = kino_astar_finder_->GetKinoTraj(vis_kino_interval);
                ROS_INFO("init success, Path_node_num: %d", traj.size());
                has_path = true;
                PubKinoTraj(traj);
            }
            else
            {
                bool search_flag2 = kino_astar_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);                    
                if(search_flag2)
                {
                    vector<Vector3d> traj = kino_astar_finder_->GetKinoTraj(vis_kino_interval); 
                    ROS_INFO("second success, Path_node_num: %d", traj.size());  
                    has_path = true;                  
                    PubKinoTraj(traj);
                }
                else
                {
                    ROS_INFO("path search fail");
                }
                
            }               
        }
        if (use_car_search)     //kino_astar_finder
        { 
            clear_result();  
            //由于函数要求传递一个普通指针，所以只能用 get(), 但此时要小心不能在函数里把内存释放掉，不然shared_ptr的对象也就消失了         
            has_car_path = kino_astar_car->Plan(start_pt(0), start_pt(1), -M_PI, end_pt(0), end_pt(1), pi, XYbounds, obstacles_vertices_vec, result.get()); 
            if(has_car_path)
            { 
                // test();               
                ROS_INFO("init car success, Path_node_num: %d", result->x.size());
                PubCarKinoTraj();
            }
        }
    }
    else
    {
        ROS_INFO("No map, set goal again");
    }
    
}

void searcher_manage::test()    //纯动力学
{
    clear_result();
    int node_num =20;
    double delt = 0.2;
    result->x.emplace_back(start_pt(0));
    result->y.emplace_back(start_pt(1));
    result->phi.emplace_back(M_PI);
    result->v.emplace_back(0);
    result->a.emplace_back(0.1);
    result->steer.emplace_back(0.05);
    for (int i = 0; i < node_num; i++)
    {
        double x = result->x[i] + delt * result->v[i] * cos(result->phi[i]);
        result->x.emplace_back(x);

        double y = result->y[i] + delt * result->v[i] * sin(result->phi[i]);
        result->y.emplace_back(y);

        double phi = result->phi[i] + delt * result->v[i] * tan(result->steer[i]) / 2.8448;
        result->phi.emplace_back(phi);

        double v = result->v[i] + delt * result->a[i];
        result->v.emplace_back(v);

        if(i == node_num - 1)
            continue;
        double a = result->a[i] + 0.05;
        if(v > 2.9)
            a=0;
        result->a.emplace_back(a);
        /* 这里 steer < 0, 是因为 steer 是相对体坐标系的，对于start_phi = pi, 那么 phi 想减小即顺时针转，所以 steer 需要小于0, 即相对体坐标系顺时针是小于0
            但对于 朝向等于 0 处，顺时针小于 0 的话 phi 为负值，总不能一直负值下去吧？？？？？？ 这部分该怎么办？？？？？？ */
        double steer = max(-8.20304748437/16.0,result->steer[i] - 0.02);
        result->steer.emplace_back(steer);
    }
    
}

void searcher_manage::clear_result()
{
    result->x.clear();
    result->y.clear();
    result->phi.clear();
    result->v.clear();
    result->a.clear();
    result->steer.clear();
    result->accumulated_s.clear();
}

void searcher_manage::PubKinoTraj(const vector<Vector3d> &nodes)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();    

    node_vis.ns = "/grid_searcher/kino_vis";

    node_vis.type = visualization_msgs::Marker::LINE_STRIP;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;    //不透明度为1.0
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0; 
    node_vis.color.b = 0.0;

    node_vis.scale.x = resolution/10.0;
    node_vis.scale.y = resolution/10.0;
    node_vis.scale.z = resolution/10.0;

    
    geometry_msgs::Point pt1;
    geometry_msgs::Point pt2;
    for(int i = 0; i < int(nodes.size()-1); ++i)
    {
        Vector3d coord1 = nodes[i];
        pt1.x = coord1(0);
        pt1.y = coord1(1);
        pt1.z = coord1(2);

        Vector3d coord2 = nodes[i+1];
        pt2.x = coord2(0);
        pt2.y = coord2(1);
        pt2.z = coord2(2);
        // ROS_INFO("%f", pt.x);
        node_vis.points.push_back(pt1);
        node_vis.points.push_back(pt2);
    }

    kino_vis.publish(node_vis);
    
}

void searcher_manage::PubCarKinoTraj()
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();    

    node_vis.ns = "/grid_searcher/kino_car_vis";

    node_vis.type = visualization_msgs::Marker::LINE_STRIP;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 5;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;    //不透明度为1.0
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0; 
    node_vis.color.b = 1.0;

    node_vis.scale.x = resolution/7.0;
    node_vis.scale.y = resolution/7.0;
    node_vis.scale.z = resolution/7.0;

    
    geometry_msgs::Point pt1;
    for(int i = 0; i < result->x.size(); ++i)
    {
        pt1.x = result->x[i];
        pt1.y = result->y[i];
        pt1.z = 0;

        node_vis.points.push_back(pt1);
    }

    kino_car_vis.publish(node_vis);
}








