#pragma once 

#include <vector>
#include <geometry_msgs/Point.h>

struct Cluster {
    geometry_msgs::Point mode;
    std::vector<geometry_msgs::Point> original_points;
    std::vector<geometry_msgs::Point> shifted_points;
};

class MeanShift {
public:
    typedef geometry_msgs::Point Point;

    MeanShift() { set_kernel(NULL); }
    MeanShift(double (*_kernel_func)(double,double)) { set_kernel(kernel_func); }
    std::vector<Point> meanshift(const std::vector<Point> & points,
                                                double kernel_bandwidth,
                                                double EPSILON = 0.3);
    std::vector<Cluster> cluster(const std::vector<Point> &, double);

private:
    double (*kernel_func)(double,double);
    void set_kernel(double (*_kernel_func)(double,double));
    void shift_point(const Point&, const std::vector<Point> &, double, Point&);
    std::vector<Cluster> cluster(const std::vector<Point> &, const std::vector<Point> &);
};
