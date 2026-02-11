#ifndef STC_GEN_HPP
#define STC_GEN_HPP

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/SimpleSetup.h>

#include <memory>
#include <Eigen/Eigen>

#include <decomp_util/ellipsoid_decomp.h>
#include <plan_env/edt_environment.h>
namespace stc_gen{
    class STCGen{
    public:
        // RRTConnect
        // validity_check_rate: normal is 1/grid_num
        static inline double PlanPath(const Eigen::Vector2d &start,
                            const Eigen::Vector2d &goal,
                            const Eigen::Vector2d &lb,
                            const Eigen::Vector2d &hb,
                            const double &validity_check_rate,
                            const double &timeout,
                            const std::function<bool(const ompl::base::State*)> &valid_checker_func,
                            std::vector<Eigen::Vector2d> &path
                            ){
            auto space = std::make_shared<ompl::base::RealVectorStateSpace>();
            space->addDimension(lb(0), hb(0));
            space->addDimension(lb(1), hb(1));

            auto setup = std::make_shared<ompl::geometric::SimpleSetup>(space);
            setup->setStateValidityChecker(valid_checker_func);
            space->setup();

            // printf("res %f\n",validity_check_rate);
            setup->getSpaceInformation()->setStateValidityCheckingResolution(validity_check_rate);
            // printf("max %f\n",space->getMaximumExtent());
            setup->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(setup->getSpaceInformation()));

            printf("%f %f %f %f\n",lb(0),hb(0),lb(1),hb(1));

            ompl::base::ScopedState<> s(setup->getStateSpace()), g(setup->getStateSpace());
            s[0] = start[0];
            s[1] = start[1];
            g[0] = goal[0];
            g[1] = goal[1];
            printf("RRTConnect planning, start(%f %f), goal(%f %f)\n",start(0),start(1),goal(0),goal(1));
            setup->setStartAndGoalStates(s, g);

            double cost = INFINITY;
            setup->getPlanner()->clear();
            auto solved = setup->solve();   

            const std::size_t ns = setup->getProblemDefinition()->getSolutionCount();
            // OMPL_INFORM("Found %d solutions", (int)ns);
            if (setup->haveSolutionPath())
            {
                ompl::geometric::PathGeometric &path_t = setup->getSolutionPath();
                path_t.interpolate(20);
                for (size_t i = 0; i < path_t.getStateCount(); i++){
                    const auto state = path_t.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                    path.emplace_back(state[0], state[1]);
                }
                printf("path searched, points: %d\n",path.size());
                cost = path_t.length();
            }
            return cost;
        }     
        
        // hpoly = [A,b]
        // line_segment = [p1,p2]
        static inline void ConvexHull(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &line_segment,
                                const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &point_cloud,
                                Eigen::MatrixX3d &hpoly,
                                const double max_aaxis = 8.0,
                                const double max_baxis = 8.0){

            auto line = std::make_shared<LineSegment<2>>(line_segment[0], line_segment[1]);
            line->set_local_bbox(Eigen::Vector2d(max_aaxis,max_baxis));
            line->set_obs(point_cloud);
            line->dilate(0);
            
            auto lc2d = LinearConstraint2D((line_segment[0]+line_segment[1])/2,line->get_polyhedron().hyperplanes());
            hpoly.resize(lc2d.A_.rows(),3);
            hpoly << lc2d.A_,lc2d.b_;
        }

        // hpoly = [A,b]
        // line_segment = [p1,p2]
        // poly_vis
        static inline void ConvexHull(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &line_segment,
                                const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &point_cloud,
                                Eigen::MatrixX3d &hpoly,Polyhedron<2> &poly_vis,
                                const double max_aaxis = 8.0,
                                const double max_baxis = 8.0){

            auto line = std::make_shared<LineSegment<2>>(line_segment[0], line_segment[1]);
            line->set_local_bbox(Eigen::Vector2d(max_aaxis,max_baxis));
            line->set_obs(point_cloud);
            line->dilate(0.0);
            
            poly_vis = line->get_polyhedron();

            auto lc2d = LinearConstraint2D((line_segment[0]+line_segment[1])/2,line->get_polyhedron().hyperplanes());
            hpoly.resize(lc2d.A_.rows(),3);
            hpoly << lc2d.A_,lc2d.b_;
        }

struct SafeCorridor {
    Eigen::MatrixX4d hpoly;  // 走廊约束矩阵 [A | b]
    Eigen::Vector2d center;  // 走廊中心
    double width;           // x方向半宽
    double height;          // y方向半高
};

static std::vector<SafeCorridor> generateCorridors(
    const std::vector<Eigen::Vector2d>& path,
    fast_planner:: EDTEnvironment::Ptr edt_env,
    double init_expand = 0.05,
    double max_expand = 1.0,
    double step = 0.02) {

    std::vector<SafeCorridor> corridors;

    for (size_t i = 1; i < path.size(); i++) {
        const auto& p_prev = path[i-1];
        const auto& p_curr = path[i];
        
        SafeCorridor corridor;
        corridor.center = (p_prev + p_curr) * 0.5;

        // 初始化各方向扩展范围
        double left = init_expand;   // x负方向
        double right = init_expand;  // x正方向 
        double down = init_expand;   // y负方向
        double up = init_expand;     // y正方向

        #pragma omp parallel for num_threads(16)
        // 独立扩展四个方向
        auto expandDirection = [&](double& extend, Eigen::Vector2d dir) {
            for (double e = extend + step; e <= max_expand; e += step) {
                Eigen::Vector2d new_min = corridor.center - Eigen::Vector2d(left, down) + dir * e;
                Eigen::Vector2d new_max = corridor.center + Eigen::Vector2d(right, up) + dir * e;
                
                // bool collision = std::any_of(obstacles.begin(), obstacles.end(),
                //     [&](const Eigen::Vector2d& obs) {
                //         return (obs.x() >= new_min.x() && obs.x() <= new_max.x()) &&
                //                (obs.y() >= new_min.y() && obs.y() <= new_max.y());
                //     });
                
                bool collision_edt = false;
                Eigen::Vector3d min_point,max_point;
                min_point << new_min.x(), new_min.y(), 0.3;
                max_point << new_max.x(), new_max.y(), 0.3;

                double dist_min = edt_env->evaluateCoarseEDT(
                    min_point, -1.0);
                if (dist_min < 0.1) 
                    collision_edt = true;

                double dist_max = edt_env->evaluateCoarseEDT(
                    max_point, -1.0);
                if (dist_max < 0.1) 
                    collision_edt = true;

                // 其他点的距离检查
                for (double x = new_min.x(); x <= new_max.x(); x += 0.2) {
                    for (double y = new_min.y(); y <= new_max.y(); y += 0.2) {
                        Eigen::Vector3d query_pt(x, y, 0.025);
                        double dist = edt_env->evaluateCoarseEDT(query_pt, -1.0);
                        if (dist < 0.1) {
                            collision_edt = true;
                            break;
                        }
                    }
                    if (collision_edt) break;
                }

                if (!collision_edt) {
                    extend = e;
                } else {
                    break;
                }
            }
        };

        // 分别扩展四个方向
        expandDirection(left, Eigen::Vector2d(-1, 0));  // 向左扩展
        expandDirection(right, Eigen::Vector2d(1, 0));  // 向右扩展
        expandDirection(down, Eigen::Vector2d(0, -1));  // 向下扩展 
        expandDirection(up, Eigen::Vector2d(0, 1));     // 向上扩展

        // 计算最终边界
        Eigen::Vector2d min_pt = corridor.center - Eigen::Vector2d(left, down);
        Eigen::Vector2d max_pt = corridor.center + Eigen::Vector2d(right, up);

        // 构建半空间约束矩阵
        Eigen::MatrixX4d A_b(6, 4);
        A_b << 1, 0, 0, -max_pt.x(),   // x <= max_x
               0, -1, 0, min_pt.y(),  // y >= min_y
               0, 0, 1, -0.05,
               0, 0, -1, 0,
               0, 1, 0, -max_pt.y(),   // y <= max_y
              -1, 0, 0, min_pt.x();  // x >= min_x

        corridor.hpoly = A_b;
        corridor.width = (max_pt.x() - min_pt.x()) / 2.0;
        corridor.height = (max_pt.y() - min_pt.y()) / 2.0;
        corridors.push_back(corridor);
    }

    return corridors;
}

static std::vector<SafeCorridor> generateEDTCorridors(
    const std::vector<Eigen::Vector2d>& path,
    fast_planner::EDTEnvironment::Ptr edt_env,
    double init_expand = 0.15,
    double max_expand = 1.0,
    double safety_margin = 0.2) 
{
    std::vector<SafeCorridor> corridors;

    #pragma omp parallel for num_threads(16)
    for (size_t i = 1; i < path.size(); i ++) {
        const auto& p_prev = path[i-1];
        const auto& p_curr = path[i];
        
        SafeCorridor corridor;
        corridor.center = (p_prev + p_curr) * 0.5;


        // 射线投射查询最大扩展距离
        auto rayCast = [&](const Eigen::Vector2d& dir) -> double {
            double max_e = init_expand;
            const double resolution = edt_env->sdf_map_->getResolution();
            Eigen::Vector2d current = corridor.center;
            
            for (double e = init_expand; e <= max_expand; e += resolution) {
                Eigen::Vector3d query_pt(current.x(), current.y(), 0.0);
                double dist = edt_env->evaluateCoarseEDT(query_pt, 0.0);
                
                // 安全条件：距离障碍物至少保持安全余量
                if (dist > (e + safety_margin)) {
                    max_e = e;
                    current += dir * resolution; // 继续沿方向移动
                } else {
                    break;
                }
            }
            return std::min(max_e, max_expand);
        };

        // 四方向独立查询
        const double left  = rayCast(Eigen::Vector2d(-1, 0));
        const double right = rayCast(Eigen::Vector2d(1, 0));
        const double down  = rayCast(Eigen::Vector2d(0, -1));
        const double up    = rayCast(Eigen::Vector2d(0, 1));

        // 构建走廊边界
        const Eigen::Vector2d min_pt = corridor.center - Eigen::Vector2d(left, down);
        const Eigen::Vector2d max_pt = corridor.center + Eigen::Vector2d(right, up);
        
        // 边界安全验证（可选）
        auto validateCorners = [&]() -> bool {
            const std::vector<Eigen::Vector2d> corners = {
                min_pt, max_pt, 
                {min_pt.x(), max_pt.y()}, 
                {max_pt.x(), min_pt.y()}
            };
            
            return std::all_of(corners.begin(), corners.end(), [&](const auto& pt) {
                Eigen::Vector3d pt3d(pt.x(), pt.y(), 0.0);
                return edt_env->evaluateCoarseEDT(pt3d, 0.0) > safety_margin;
            });
        };

        if (validateCorners()) {
            // 生成半空间约束（保持原格式）
            Eigen::MatrixX4d A_b(6, 4);
            A_b << 1, 0, 0, -max_pt.x(),
                   0, -1, 0, min_pt.y(),
                   0, 0, 1, -0.05,
                   0, 0, -1, 0,
                   0, 1, 0, -max_pt.y(),
                  -1, 0, 0, min_pt.x();
            
            corridor.hpoly = A_b;
            corridor.width = (max_pt.x() - min_pt.x()) / 2.0;
            corridor.height = (max_pt.y() - min_pt.y()) / 2.0;
            corridors.push_back(corridor);
        }
    }
    return corridors;
}

    };
}; // namespace stc_gen
#endif