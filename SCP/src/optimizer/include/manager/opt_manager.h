#include "optimizer/snap_closed_form.h"
#include "manager/scp.h"
#include "qp/qp_state.h"
#include "corridor/corridor.h"
#include "manager/nlp_manager.h"
#include "nlp/contingence.h"
#include <unordered_map>

namespace optimize
{
using namespace std;
using namespace Eigen;
class opt_manager
{

    public:
        opt_manager(ros::NodeHandle &nh);
        void OptiPathUAV(const vector<Matrix<double, 10, 1>> &state_list);

        void OptiPathCAR(const vector<Matrix<double, 6, 1>> &state_list);  //x,y,phi,v,steer,a

    private:
        bool use_snap_closed = true;
        bool use_scp_nlp = true;

        unique_ptr<snap_optimizer::snap> min_snap;

        unique_ptr<SCP::scp> scp_nlp;

        unique_ptr<qp::FindStateProblem> speed_plan;

        unique_ptr<corridor::construct_corridor> generate_corridor;

        unique_ptr<nlp_manager::nlp_solver> contingence_opt;

        void vis_traj(const vector<Matrix<double, 7, 1>> &new_state);
        std::unordered_map<int, cartesian_planner::math::Polygon2d> find_dynamic_obs(double time, std::vector<corridor::DynamicObstacle_> &dynamic_obs);

        void vis_path(const vector<Matrix<double, 7, 1>> &new_state, string name, cartesian_planner::visualization::Color color);
};
}