#include <stdio.h>
#include <math.h>
#include "../include/MeanShiftAlgorithm.h"

using namespace std;

#define CLUSTER_EPSILON 2.0

double euclidean_distance(const geometry_msgs::Point &point_a, const geometry_msgs::Point &point_b){
    double total = 0;
    const double temp = pow((point_a.x - point_b.x), 2) +
        pow((point_a.y - point_b.y), 2) +
         pow((point_a.z - point_b.z), 2);
    total = temp;
    return sqrt(total);
}

double euclidean_distance_sqr(const geometry_msgs::Point &point_a, const geometry_msgs::Point &point_b){
    double total = 0;
    const double temp = pow((point_a.x - point_b.x), 2) +
        pow((point_a.y - point_b.y), 2) +
         pow((point_a.z - point_b.z), 2);
    total = temp;
    return (total);
}

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0 / 2.0 * (distance * distance) / (kernel_bandwidth * kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double, double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

void MeanShift::shift_point(const Point &point,
                            const std::vector<Point> &points,
                            double kernel_bandwidth,
                            Point &shifted_point) {
    shifted_point.x = 0;
    shifted_point.y = 0;
    shifted_point.z = 0;
    double total_weight = 0;
    for(int i = 0; i < points.size(); i++){
        const Point& temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);
        shifted_point.x += temp_point.x * weight;
        shifted_point.y += temp_point.y * weight;
        shifted_point.z += temp_point.z * weight;
        total_weight += weight;
    }
    const double total_weight_inv = 1.0 / total_weight;
    shifted_point.x *= total_weight_inv;
    shifted_point.y *= total_weight_inv;
    shifted_point.z *= total_weight_inv;
}

std::vector<MeanShift::Point> MeanShift::meanshift(const std::vector<Point> &points,
                                             double kernel_bandwidth,
                                             double EPSILON){
    ros::WallTime startTime_evaluation = ros::WallTime::now();
    const double EPSILON_SQR = EPSILON * EPSILON;
    vector<bool> stop_moving(points.size(), false);
    vector<Point> shifted_points = points;
    double max_shift_distance;
    Point point_new;
    do {
        max_shift_distance = 0;
        for(int i=0; i<points.size(); i++){
            if (!stop_moving[i]) {
                shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
                double shift_distance_sqr = euclidean_distance_sqr(point_new, shifted_points[i]);
                if(shift_distance_sqr > max_shift_distance){
                    max_shift_distance = shift_distance_sqr;
                }
                if(shift_distance_sqr <= EPSILON_SQR) {
                    stop_moving[i] = true;
                }
                shifted_points[i] = point_new;
            }
        }
    } while (max_shift_distance > EPSILON_SQR);
    double total_time_evaluation = (ros::WallTime::now() - startTime_evaluation).toSec();
	// cout <<"Mean shift function "<< total_time_evaluation <<" sec"<<endl;
    return shifted_points;
}

vector<Cluster> MeanShift::cluster(const std::vector<Point> &points,
    const std::vector<Point> &shifted_points)
{
    ros::WallTime startTime_evaluation = ros::WallTime::now();
    vector<Cluster> clusters;
    for (int i = 0; i < shifted_points.size(); i++) {

        int c = 0;
        for (; c < clusters.size(); c++) {
            if (euclidean_distance(shifted_points[i], clusters[c].mode) <= CLUSTER_EPSILON) {
                break;
            }
        }

        if (c == clusters.size()) {
            Cluster clus;
            clus.mode = shifted_points[i];
            clusters.push_back(clus);
        }

        clusters[c].original_points.push_back(points[i]);
        clusters[c].shifted_points.push_back(shifted_points[i]);
    }
    double total_time_evaluation = (ros::WallTime::now() - startTime_evaluation).toSec();
	// cout <<"Cluster function "<< total_time_evaluation <<" sec"<<endl;
    return clusters;
}

vector<Cluster> MeanShift::cluster(const std::vector<Point> &points, double kernel_bandwidth){
    vector<Point> shifted_points = meanshift(points, kernel_bandwidth);
    return cluster(points, shifted_points);
}
