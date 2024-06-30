#include"map/current.h"

namespace map_
{
    void current_map::init(ros::NodeHandle &nh)
    {
        double radius;
        nh.param("vortexes_num", vortexes_num, 1);
        nh.param("/map/dim_flag", current_flag, true);
        nh.param("radius", radius, 500.0);

        nh.param("current/resolution", resolution, 0.5);
        nh.param("/map/x_size",x_size,20.0);  //地图面积是50×50,但搜索算法处理时，不一定要50×50，应该是50*5的大小
        nh.param("/map/y_size",y_size,20.0);
        nh.param("/map/z_size",z_size,10.0);

        current_pub_vis = nh.advertise<geometry_msgs::PoseArray>("current_vis", 1);
        current_pub = nh.advertise<geometry_msgs::PoseArray>("current", 1);

        Vector4d vortex(x_size, y_size, 0.0, radius);        
        vortexes.push_back(vortex);
        // ROS_INFO("radius: %f",vortexes[0](3));
    }

    void current_map::GenCurrentPoint()
    {
        for(double r = - x_size / 2.0; r < x_size/ 2.0;  r += resolution )
        {
            for(double s = - y_size / 2.0; s < y_size/ 2.0;  s += resolution )
            {  
                if(current_flag)
                {
                    lambVortexFun(r, s);
                }
                else
                {
                    for(double t = 0.0; t < z_size;  t += resolution)
                    {
                        lambVortexFun(r, s, t);
                    }
                } 
            }
        }
    }

    void current_map::lambVortexFun(double coordx, double coordy, double coordz)
    {
        if (current_flag)
        {
            //将z方向也算上
        }  
        //还需要传 c_x 和 c_y 。所以需要两套，一套用来vis，一套用来给其他节点解析
        double c_x=0, c_y=0, c_z=0;                    
        for (int i = 0; i < vortexes_num; ++i)
        {
            double dis = Vector3d(coordx - vortexes[i](0),coordx - vortexes[i](1), coordx - vortexes[i](2)).norm();            
            c_x += ((coordy - vortexes[i](1) ) / (2 * M_PI * dis)) * (1 - exp(-dis / pow(vortexes[i](3), 2)));
            c_y += ((coordx - vortexes[i](0) ) / (2 * M_PI * dis)) * (1 - exp(-dis / pow(vortexes[i](3), 2)));
            double p = 10;
            c_x *= p;
            c_y *= p;
            if (fabs(dis) < 0.001)
            {
                c_x = 0.0;
                c_y = 0.0;
            }
            
            if (current_flag)
            {
                //将z方向也算上
            }  
        }

        /* current_info_vis */
        double z_rad = GetRad(c_x, c_y);
        // cout<< "x: "<< c_x <<"y:  "<<c_y<< "z_rad: "<<z_rad<<endl;
        geometry_msgs::Pose tmp_pose;
        tmp_pose.position.x = coordx;
        tmp_pose.position.y = coordy;
        tmp_pose.position.z = coordz;

        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, z_rad);
        tmp_pose.orientation.x = qtn.getX();
        tmp_pose.orientation.y = qtn.getY();
        tmp_pose.orientation.z = qtn.getZ();
        tmp_pose.orientation.w = qtn.getW();
        
        current_info_vis.poses.push_back(tmp_pose);

        /* current_info */
        tmp_pose.position.x = coordx;
        tmp_pose.position.y = coordy;
        tmp_pose.position.z = coordz;

        tmp_pose.orientation.x = c_x;  //传输幅值信息
        tmp_pose.orientation.y = c_y;
        tmp_pose.orientation.z = 0;
        tmp_pose.orientation.w = 0;
        
        current_info.poses.push_back(tmp_pose);
    }


}