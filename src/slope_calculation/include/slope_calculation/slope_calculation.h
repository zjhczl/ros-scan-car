#ifndef SLOPE_CALCULATION_H
#define SLOPE_CALCULATION_H

#include "common_point_types.h"
#include "least_square.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <list>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

struct SlopeCalculationInitOptions {
};

typedef ouster_ros::PointXYZIR PointT;
class SlopeCalculation
{
public:
    SlopeCalculation();
    ~SlopeCalculation(){};

    void createRosPubSub();
    void init(const SlopeCalculationInitOptions = SlopeCalculationInitOptions());
    bool pointPreProcess(const pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud);
    bool fitLine(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud);
    void displaySlopePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr slope_points);
    void displaySlopeMsg(pcl::PointXYZI point_top);
    void smoothResults();
    void obtainDealRingCloud(pcl::PointCloud<PointT>::Ptr input);
    void saveFailData(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud, int index, float line_slope, float line_intercept, int fit_num);
    void linearRegression(pcl::PointCloud<pcl::PointXYZI>::Ptr fit_cloud, float &line_slope, float &line_intercept);
    void rtkCallBack(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

private:
    void pointsCallBack(const sensor_msgs::PointCloud2 &msg);
    void pointT2PointXYZI(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output);

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;
    ros::Publisher slope_result_pub_;
    ros::Subscriber rtk_sub_;

    LeastSquare lesq_;
    Eigen::VectorXd x_data_;
    Eigen::VectorXd y_data_;
    Eigen::VectorXd coeffs_;
    std::queue<pcl::PointXYZI> point_queue;
    int seeds_num_ = 30;                                // 拟合直线的种子
    int seeds_thresh_ = 10;                             // 种子进行筛选
    int degree_ = 1;                                    // 拟合一维平面。
    int ring_id_ = -1;                                  // 机关雷达第几个ring.
    float height_ = 0;                                  // 挡墙的高度
    float slope_angle_ = 0;                             // 坡度
    bool is_right_ = false;                             // 朝前摆放的激光雷达 true.
    float heigt_thresh_ = 0.1;                          // 点到直线的高度阈值
    float slope_dis_ = 0;                               // 挡墙到车的距离
    int over_thresh_num_ = 4;                           // 连续几个点超过阈值
    pcl::PointXYZI point_top_;                          // 当前位置顶点的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr slope_points_; // 当前的slope上的点云
    bool do_debug_ = false;
    int current_save_id_ = 0;
    float over_num_scale_ = 0.5; // 使用超过多少个进行直线再拟合。
    float average_x_ = 0;
    float slope_thresh_ = 0; // slope 过滤阈值
    float green_height_ = 0;

    // for smooth
    float height_results_scale_ = 0.9;      // 选择链表中的比例。
    float slope_angle_results_scale_ = 0.9; // 选择链表中的比例。
    float slope_dis_results_scale_ = 0.9;   // 选择链表中的比例。
    std::list<float> height_list_;
    std::list<float> slope_angle_list_;
    std::list<float> slope_dis_list_;
    float height_pre_ = 0;
    float slope_angle_pre_ = 0;
    float slope_dis_pre_ = 0;
    int smooth_data_number_ = 15;
    float height_pub_data_ = 0;
    float slope_dis_pub_data_ = 0;
    float slope_angle_pub_data_ = 0;
    // for rtk
    std::list<float> r_list_;
    float r_use_ = 0;
    float r_pre_ = 0;
    float rtk_scale_ = 0.5;
    // for debug
    ros::Publisher slope_points_pub_;
    ros::Publisher slope_text_pub_;
};

#endif