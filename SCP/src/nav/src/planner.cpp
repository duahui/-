#include"nav/planner.h"

namespace nav
{


planner::planner(ros::NodeHandle &nh)
{
    grid_searcher.reset(new searcher_manage(nh));

    opt_path.reset(new optimize::opt_manager(nh)); 

    nh.param("searcher_flag",searcher_flag,1);
    
    nh.param("use_car_search",use_car_search,true);   
}

void planner::planning()
{
    if(searcher_flag == 1)  //无人机
    {
        vector<Matrix<double, 10, 1>> state_lsit = grid_searcher->GetTrajPoints();
        if(state_lsit.size() != 0)
        {
            /* 调用优化步骤 */
            // opt_path->OptiPathUAV(state_lsit);
        }
    }
    if(use_car_search)    //无人车
    {
        std::shared_ptr<HybridAStartResult> result = grid_searcher->get_car_Traj();  
         
        if(result != nullptr)
        {
            // for(int i = 0; i< result->x.size(); ++i)
            // {
            //     cout<<"x: "<< result->x[i] <<"y: "<< result->y[i] <<"phi: "<< result->phi[i]<<endl;
            //     if(i == result->x.size() -1 )
            //         continue;
            //     cout<<"v: "<< result->v[i]<<"a: "<< result->a[i]<<"steer: "<< result->steer[i]<<endl;
            //     // cout<<"accumulated_s: "<< result->accumulated_s[i]<<endl;

            // } 
            /* 调用优化步骤 */
            vector<Matrix<double, 6, 1>> state_lsit_car = get_points(result.get());
            opt_path->OptiPathCAR(state_lsit_car);
        }
    }

}

vector<Matrix<double, 6, 1>> planner::get_points(const HybridAStartResult *result)
{
    vector<Matrix<double, 6, 1>> points;
    for (int i = 0; i < result->x.size(); ++i)
    {
        Matrix<double, 6, 1> temp;
        temp(0) = result->x[i];
        temp(1) = result->y[i];
        temp(2) = result->phi[i];
        temp(3) = result->v[i];
        if (i == result->x.size() - 1)
        {
            temp(4) = 0;
            temp(5) = 0;
        }
        else
        {
            temp(4) = result->steer[i];
            temp(5) = result->a[i];
        }
        points.emplace_back(temp);
    }
    return std::move(points);
}

}