#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include <vector>
#include <cmath>
#include <map>

struct Point { double x, y; };

class DBSCAN {
public:
    DBSCAN(double eps, int min_pts) : eps_(eps), min_pts_(min_pts) {}

    std::vector<int> cluster(const std::vector<Point>& points) {
        int n = points.size();
        std::vector<int> labels(n, 0); // 0: 未访问, -1: 噪声
        int cluster_id = 0;
        for (int i = 0; i < n; i++) {
            if (labels[i] != 0) continue;
            std::vector<int> neighbors = find_neighbors(points, i);
            if (neighbors.size() < (size_t)min_pts_) {
                labels[i] = -1;
            } else {
                cluster_id++;
                expand_cluster(points, labels, i, neighbors, cluster_id);
            }
        }
        return labels;
    }

private:
    double eps_; int min_pts_;
    std::vector<int> find_neighbors(const std::vector<Point>& pts, int idx) {
        std::vector<int> nb;
        for (int i = 0; i < (int)pts.size(); i++) {
            if (std::hypot(pts[idx].x - pts[i].x, pts[idx].y - pts[i].y) < eps_) nb.push_back(i);
        }
        return nb;
    }
    void expand_cluster(const std::vector<Point>& pts, std::vector<int>& labels, int idx, std::vector<int>& nb, int cid) {
        labels[idx] = cid;
        for (size_t i = 0; i < nb.size(); i++) {
            int n_idx = nb[i];
            if (labels[n_idx] == -1) labels[n_idx] = cid;
            if (labels[n_idx] != 0) continue;
            labels[n_idx] = cid;
            std::vector<int> next_nb = find_neighbors(pts, n_idx);
            if (next_nb.size() >= (size_t)min_pts_) nb.insert(nb.end(), next_nb.begin(), next_nb.end());
        }
    }
};
#endif