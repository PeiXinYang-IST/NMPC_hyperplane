/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>  // 修改为 RRTstar
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <deque>
#include <memory>
#include <Eigen/Eigen>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <vector>
#include <queue>
#include <cmath>
#include <utility>
#include <algorithm>
    #include <vector>
#include <Eigen/Dense>
#include <omp.h> // OpenMP头文件
namespace sfc_gen
{


template <typename Map>
inline double planPath_Djistra(const Eigen::Vector3d &s,
                       const Eigen::Vector3d &g,
                       const Eigen::Vector3d &lb,
                       const Eigen::Vector3d &hb,
                       const Map *mapPtr,const Map *dynamic_mapPtr,
                       const Eigen::Vector3d &odom_pos,
                       const double &timeout,
                       std::vector<Eigen::Vector3d> &p) {

    auto start_time = std::chrono::high_resolution_clock::now(); // 开始计时

    // 设置网格分辨率
    const double dx = 0.05; // x方向的分辨率
    const double dy = 0.05; // y方向的分辨率

    // 计算网格的列数和行数
    const int cols = static_cast<int>((hb(0) - lb(0)) / dx);
    const int rows = static_cast<int>((hb(1) - lb(1)) / dy);

    // 创建网格并标记障碍物
    std::vector<std::vector<bool>> grid(cols, std::vector<bool>(rows, false));
    for (int i = 0; i < cols; ++i) {
        for (int j = 0; j < rows; ++j) {
            const double x = lb(0) + i * dx + dx/2; // 网格中心坐标
            const double y = lb(1) + j * dy + dy/2;
            const Eigen::Vector3d pos(x, y, 0.05); // 使用与状态检查相同的z值
            if ((pos - odom_pos).squaredNorm() <= 0.3 * 0.3) {
                grid[i][j] = true;
                continue;
            }
            else
            grid[i][j] = (mapPtr->query(pos) == 0) && (dynamic_mapPtr->query(pos) == 0);
        }
    }

    // 转换起点和目标点到网格索引
    const int start_i = static_cast<int>((s(0) - lb(0)) / dx);
    const int start_j = static_cast<int>((s(1) - lb(1)) / dy);
    const int goal_i = static_cast<int>((g(0) - lb(0)) / dx);
    const int goal_j = static_cast<int>((g(1) - lb(1)) / dy);

    // 检查起点和终点是否有效
    if (start_i < 0 || start_i >= cols || start_j < 0 || start_j >= rows || !grid[start_i][start_j]) {
        return INFINITY;
    }
    if (goal_i < 0 || goal_i >= cols || goal_j < 0 || goal_j >= rows || !grid[goal_i][goal_j]) {
        return INFINITY;
    }

    // Dijkstra算法实现
    using Node = std::pair<int, int>;
    const std::vector<std::pair<int, int>> dirs = {{-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {-1,1}, {1,-1}, {1,1}}; // 8邻域

    std::vector<std::vector<double>> dist(cols, std::vector<double>(rows, INFINITY));
    std::vector<std::vector<Node>> prev(cols, std::vector<Node>(rows, {-1, -1}));
    std::priority_queue<std::pair<double, Node>, std::vector<std::pair<double, Node>>, std::greater<>> pq;

    dist[start_i][start_j] = 0.0;
    pq.emplace(0.0, Node(start_i, start_j));

    bool found = false;
    while (!pq.empty()) {
        const auto [current_dist, current] = pq.top();
        pq.pop();

        if (current.first == goal_i && current.second == goal_j) {
            found = true;
            break;
        }

        if (current_dist > dist[current.first][current.second]) continue;

        for (const auto& dir : dirs) {
            int ni = current.first + dir.first;
            int nj = current.second + dir.second;
            if (ni < 0 || ni >= cols || nj < 0 || nj >= rows || !grid[ni][nj]) continue;

            // 计算移动代价，考虑对角线距离
            const bool is_cardinal = (dir.first == 0 || dir.second == 0);
            const double step_cost = is_cardinal ? 
                                    (dir.first != 0 ? dx : dy) :  // 直线移动取对应轴长度
                                    std::hypot(dx, dy);           // 对角线取欧氏距离
            const double new_dist = current_dist + 10*step_cost;

            if (new_dist < dist[ni][nj]) {
                dist[ni][nj] = new_dist;
                prev[ni][nj] = current;
                pq.emplace(new_dist, Node(ni, nj));
            }
        }
    }

    if (!found) {
        std::cerr << "Error: No path found using Dijkstra's algorithm." << std::endl;
        return INFINITY;
    }

    // 回溯路径
    std::vector<Node> path_nodes;
    Node current = {goal_i, goal_j};
    while (current.first != -1 && current.second != -1) {
        path_nodes.push_back(current);
        current = prev[current.first][current.second];
    }
    std::reverse(path_nodes.begin(), path_nodes.end());

    // 转换为连续坐标
    p.clear();
    for (const auto& node : path_nodes) {
        const double x = lb(0) + node.first * dx + dx/2;
        const double y = lb(1) + node.second * dy + dy/2;
        p.emplace_back(x, y, 0.05); // 路径点高度设为0.05
    }
    
    // 计算路径总长度
    double cost = 0.0;
    for (size_t i = 1; i < p.size(); ++i) {
        cost += (p[i] - p[i-1]).norm();
    }

    auto end_time = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Dijkstra's algorithm execution time: " << elapsed.count() << " seconds" << std::endl;

    return cost;
}


    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, 0.05);
        space->setBounds(bounds);

        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                auto start_time = std::chrono::high_resolution_clock::now();
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               0.2);
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end_time - start_time;
                // std::cout << "State validity check time: " << elapsed.count() << " seconds" << std::endl;

                return mapPtr->query(position) == 0;
            });

        si->setup();

        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] =  0.05;
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] =  0.05;

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        // planner->setRange(0.2);
        planner->setup();

        // auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
        // planner->setProblemDefinition(pdef);
        // planner->setup();

        // auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
        // planner->setProblemDefinition(pdef);
        // planner->setRange(1);
        // planner->setup();

        // 创建LazyPRM规划器
        // auto planner(std::make_shared<ompl::geometric::LazyPRM>(si));
        // planner->setProblemDefinition(pdef);
        // planner->setup();

        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;
        if (solved)
        {
            p.clear();
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], 0.05);
            }
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }
        return cost;
    }

//这里进行并行化处理之后加速？
inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        
        // #pragma omp parallel for num_threads(16)

        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, 1.5);
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

        //固定Z轴为路径段平均高度（或指定平面高度）
        const double z_plane = (a(2) + b(2)) * 0.5; // 使用路径段平均高度
        Eigen::Vector3d a_plane(a(0), a(1), z_plane);
        Eigen::Vector3d b_plane(b(0), b(1), z_plane);

            firi::firi(bd, pc, a_plane, b_plane, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), 0.05, 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }
            hpolys.emplace_back(hp);
        }
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }



}

#endif
