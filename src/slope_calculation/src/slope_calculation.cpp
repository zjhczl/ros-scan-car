#include "slope_calculation.h"

SlopeCalculation::SlopeCalculation()
    : nh_("~")
{
}

void SlopeCalculation::init(const SlopeCalculationInitOptions)
{
    nh_.getParam("ring_id", ring_id_);
    nh_.getParam("do_debug", do_debug_);
    nh_.getParam("height_results_scale", height_results_scale_);
    nh_.getParam("slope_angle_results_scale", slope_angle_results_scale_);
    nh_.getParam("slope_dis_results_scale", slope_dis_results_scale_);
    nh_.getParam("slope_thresh", slope_thresh_);
    x_data_.resize(seeds_num_);
    y_data_.resize(seeds_num_);
    coeffs_.resize(degree_ + 1);
    lesq_.init(seeds_num_, degree_);
    slope_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout
        << " ring_id " << ring_id_ << " " << do_debug_ << " " << height_results_scale_ << " " << slope_angle_results_scale_
        << " " << slope_dis_results_scale_ << " " << slope_thresh_ << std::endl;
}

// 仅仅对ring进行更新。
void SlopeCalculation::obtainDealRingCloud(pcl::PointCloud<PointT>::Ptr input)
{
    // obtain the ring filter the data
    std::vector<pcl::PointCloud<pcl::PointXYZI>> ring_vec; // 每个ring的点云
    std::vector<int> ring_number_vec;
    std::vector<float> ring_dis_sum;
    std::vector<float> points_fabs_y;
    pcl::PointCloud<pcl::PointXYZI> ring_single;
    int current_ring_id = input->points[0].ring; // 当前ring的id
    int inter_gap = 3;                           // 使用离中间ring的距离
    float dis_sum = 0;                           // 每个ring 临时距离的总和。
    float dis_sum_min = 0;                       // 最小距离总和的日ring
    int dis_sum_min_ring_id = 0;                 // 最小ring的id.
    int index = 0;                               // 每个ring对应的id
    int vec_index = 0;                           // 最小距离ring在vec的id
    float fabs_y_tem = 0;                        // 每个ring当前最大的y值
    for (auto &point : input->points) {
        if (64 - inter_gap < point.ring && point.ring < 64 + inter_gap) {
            pcl::PointXYZI out_point;
            out_point.x = point.x;
            out_point.y = point.y;
            out_point.z = point.z;
            out_point.intensity = point.intensity;
            float dis_tem = sqrt(out_point.x * out_point.x + out_point.y * out_point.y + out_point.z * out_point.z);
            if (point.ring == current_ring_id) { // 这里会少掉最后一个ring,因为到了下一个才取出上一个来。
                if (fabs(point.y) > 1) {
                    ring_single.points.push_back(out_point);
                    dis_sum += dis_tem;
                    if (fabs(point.y) > fabs_y_tem) {
                        fabs_y_tem = fabs(point.y);
                    }
                }
            } else {
                if (0 != ring_single.points.size()) {
                    ring_vec.push_back(ring_single);
                    ring_number_vec.push_back(current_ring_id);
                    ring_dis_sum.push_back(dis_sum);
                    points_fabs_y.push_back(fabs_y_tem);
                    ring_single.points.clear();
                    if (dis_sum > dis_sum_min) {
                        dis_sum_min = dis_sum;
                        dis_sum_min_ring_id = current_ring_id;
                        vec_index = index;
                    }
                    dis_sum = 0;
                    fabs_y_tem = 0;
                    index++;
                }
                current_ring_id = point.ring;
            }
        }
    }
    // std::cout << "before the point fabs y ";
    // for (int i = 0; i < points_fabs_y.size(); i++) {
    //     std::cout << ring_number_vec[i] << " " << points_fabs_y[i] << " ";
    // }
    // std::cout << std::endl;

    for (int i = 0; i < points_fabs_y.size() - 1; i++) {
        for (int j = i + 1; j < points_fabs_y.size(); j++) {
            if (points_fabs_y[i] < points_fabs_y[j]) {
                float tem = points_fabs_y[i];
                points_fabs_y[i] = points_fabs_y[j];
                points_fabs_y[j] = tem;

                int tem_id = ring_number_vec[i];
                ring_number_vec[i] = ring_number_vec[j];
                ring_number_vec[j] = tem_id;
            }
        }
    }
    // std::cout << "after the point fabs y ";
    // for (int i = 0; i < points_fabs_y.size(); i++) {
    //     std::cout << ring_number_vec[i] << " " << points_fabs_y[i] << " ";
    // }
    // std::cout << std::endl;
    ring_id_ = ring_number_vec[0];
    std::cout << " change ring_id_  to " << ring_id_ << std::endl;
}

void SlopeCalculation::pointT2PointXYZI(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    if (is_right_) {
        // 只取出右侧的点云, 并且滤除1米范围内的点云。(还能过滤非零点)
        int num = 0;
        output->header = input->header;
        for (auto &point : input->points) {
            if (point.ring == ring_id_) {
                if (num > 512 && fabs(point.y) > 1.0 && fabs(point.x) < 10 && fabs(point.y) < 20) {
                    pcl::PointXYZI out_point;
                    out_point.x = point.x;
                    out_point.y = point.y;
                    out_point.z = point.z;
                    out_point.intensity = point.intensity;
                    output->points.push_back(out_point);
                }
                num++;
            }
        }
    } else {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_tem(new pcl::PointCloud<pcl::PointXYZI>);
        // 只取出右侧的点云, 并且滤除1米范围内的点云。(还能过滤非零点)
        int num = 0;
        // pcl::PointXYZI point_tem;
        output->header = input->header;
        for (auto &point : input->points) {
            if (point.ring == ring_id_) {
                if (num < 512 && fabs(point.y) > 1.0 && fabs(point.x) < 10 && fabs(point.y) < 20) {
                    pcl::PointXYZI out_point;
                    out_point.x = point.x;
                    out_point.y = point.y;
                    out_point.z = point.z;
                    out_point.intensity = point.intensity;
                    out_tem->points.push_back(out_point);
                }
                num++;
            }
        }
        int len = out_tem->points.size();
        for (int i = 0; i < len; i++) {
            output->points.push_back(out_tem->points[len - i - 1]);
        }
    }
}

void SlopeCalculation::displaySlopeMsg(pcl::PointXYZI point_top)
{
    visualization_msgs::Marker slope_msg_maker;
    slope_msg_maker.header.frame_id = "os_sensor";
    slope_msg_maker.header.stamp = ros::Time();
    slope_msg_maker.ns = "slope_msg_maker";
    slope_msg_maker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    slope_msg_maker.action = visualization_msgs::Marker::ADD;

    slope_msg_maker.color.a = 1.0;
    slope_msg_maker.color.b = 1.0;
    slope_msg_maker.color.g = 1.0;
    slope_msg_maker.color.r = 1.0;

    slope_msg_maker.scale.x = 0.5;
    slope_msg_maker.scale.y = 0.5;
    slope_msg_maker.scale.z = 0.5;

    slope_msg_maker.pose.orientation.x = 0.0;
    slope_msg_maker.pose.orientation.y = 0.0;
    slope_msg_maker.pose.orientation.z = 0.0;
    slope_msg_maker.pose.orientation.w = 1.0;
    slope_msg_maker.id = 0;
    slope_msg_maker.pose.position.x = point_top.x - 0.2;
    slope_msg_maker.pose.position.y = point_top.y;
    slope_msg_maker.pose.position.z = point_top.z;

    std::stringstream height_stream;
    height_stream << std::fixed << std::setprecision(2) << height_pub_data_;
    std::stringstream slope_stream;
    slope_stream << std::fixed << std::setprecision(2) << slope_angle_pub_data_;
    std::string results_string = height_stream.str() + "m " + slope_stream.str() + "%";
    std::stringstream slope_dis_stream;
    slope_dis_stream << std::fixed << std::setprecision(2) << slope_dis_pub_data_;
    slope_msg_maker.text = results_string;
    slope_text_pub_.publish(slope_msg_maker);

    // pub the reusuts to web
    results_string = "right:" + height_stream.str() + "," + slope_stream.str() + "," + slope_dis_stream.str();
    std_msgs::String msg;
    msg.data = results_string;
    slope_result_pub_.publish(msg);
}

void SlopeCalculation::displaySlopePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr slope_points)
{
    visualization_msgs::Marker slope_maker;
    slope_maker.header.frame_id = "os_sensor";
    slope_maker.header.stamp = ros::Time();
    slope_maker.ns = "slope_maker";
    slope_maker.type = visualization_msgs::Marker::POINTS;
    slope_maker.action = visualization_msgs::Marker::ADD;

    slope_maker.color.a = 1.0;
    slope_maker.color.b = 1.0;
    slope_maker.color.g = 1.0;
    slope_maker.color.r = 1.0;

    slope_maker.scale.x = 0.1;
    slope_maker.scale.y = 0.1;
    slope_maker.scale.z = 0.1;

    slope_maker.pose.orientation.x = 0.0;
    slope_maker.pose.orientation.y = 0.0;
    slope_maker.pose.orientation.z = 0.0;
    slope_maker.pose.orientation.w = 1.0;
    slope_maker.id = 0;
    slope_maker.points.clear();
    geometry_msgs::Point gem_point;
    for (auto &point : slope_points->points) {
        gem_point.x = point.x;
        gem_point.y = point.y;
        gem_point.z = point.z;
        slope_maker.points.push_back(gem_point);
    }
    slope_points_pub_.publish(slope_maker);
}

void SlopeCalculation::smoothResults()
{
    // for height
    if (smooth_data_number_ != height_list_.size()) {
        if (height_ != height_pre_) {
            height_list_.push_back(height_);
            height_pre_ = height_;
        }
    } else {
        std::vector<float> height_tem_data;
        std::list<float>::iterator it;
        for (it = height_list_.begin(); it != height_list_.end(); it++) {
            height_tem_data.push_back(*it);
        }
        std::sort(height_tem_data.begin(), height_tem_data.end());

        if (height_ != height_pre_) {
            height_list_.push_back(height_);
            height_pre_ = height_;
            height_pub_data_ = height_tem_data[int(smooth_data_number_ * height_results_scale_) - 1];
            height_list_.pop_front();
        } else {
            height_pub_data_ = height_tem_data[int(smooth_data_number_ * height_results_scale_) - 1];
        }
    }
    // for slope_angle
    if (smooth_data_number_ != slope_angle_list_.size()) {
        if (slope_angle_ != slope_angle_pre_) {
            slope_angle_list_.push_back(slope_angle_);
            slope_angle_pre_ = slope_angle_;
        }
    } else {
        std::vector<float> slope_angle_tem_data;
        std::list<float>::iterator it;
        for (it = slope_angle_list_.begin(); it != slope_angle_list_.end(); it++) {
            slope_angle_tem_data.push_back(*it);
        }
        std::sort(slope_angle_tem_data.begin(), slope_angle_tem_data.end());

        if (slope_angle_ != slope_angle_pre_) {
            slope_angle_list_.push_back(slope_angle_);
            slope_angle_pre_ = slope_angle_;
            slope_angle_pub_data_ = slope_angle_tem_data[int(smooth_data_number_ * slope_angle_results_scale_) - 1];
            slope_angle_list_.pop_front();
        } else {
            slope_angle_pub_data_ = slope_angle_tem_data[int(smooth_data_number_ * slope_angle_results_scale_) - 1];
        }
    }
    // for slope dis
    if (smooth_data_number_ != slope_dis_list_.size()) {
        if (slope_dis_pre_ != slope_dis_) {
            slope_dis_list_.push_back(slope_dis_);
            slope_dis_pre_ = slope_dis_;
        }
    } else {
        std::vector<float> slope_dis_tem_data;
        std::list<float>::iterator it;
        for (it = slope_dis_list_.begin(); it != slope_dis_list_.end(); it++) {
            slope_dis_tem_data.push_back(*it);
        }
        std::sort(slope_dis_tem_data.begin(), slope_dis_tem_data.end());

        if (slope_dis_ != slope_dis_pre_) {
            slope_dis_list_.push_back(slope_dis_);
            slope_dis_pre_ = slope_dis_;
            slope_dis_pub_data_ = slope_dis_tem_data[int(smooth_data_number_ * slope_dis_results_scale_) - 1];
            slope_dis_list_.pop_front();
        } else {
            slope_dis_pub_data_ = slope_dis_tem_data[int(smooth_data_number_ * slope_dis_results_scale_) - 1];
        }
    }
}
void SlopeCalculation::saveFailData(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud, int index, float line_slope, float line_intercept, int fit_num)
{
    std::string out_path = "/home/fupeng/fupeng/Ros_Project/catkin_ws/" + std::to_string(index) + ".csv";

    std::ofstream fp(out_path);
    if (fp.is_open()) {
        fp << line_intercept << "," << line_slope << "," << fit_num << "," << r_use_ << "\n";
        for (auto &point : right_cloud->points) {
            fp << point.x << "," << point.y << "\n";
        }

        fp.close();
    }
}

// 最小二乘法直线拟合
void SlopeCalculation::linearRegression(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud, float &line_slope, float &line_intercept)
{

    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

    int n = seeds_num_;

    for (int i = 0; i < n; i++) {
        auto &point = right_cloud->points[i];
        sum_x += point.y;
        sum_y += point.x;
        sum_xy += point.y * point.x;
        sum_xx += point.y * point.y;
    }
    average_x_ = sum_y / n;
    double numerator = n * sum_xy - sum_x * sum_y;
    double denominator = n * sum_xx - sum_x * sum_x;

    // 计算斜率 m
    line_slope = numerator / denominator;

    // 计算截距 b
    line_intercept = (sum_y - line_slope * sum_x) / n;
}

bool SlopeCalculation::fitLine(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud)
{
    if (right_cloud->points.size() < 40) {
        seeds_num_ = 14;
    } else {
        seeds_num_ = right_cloud->points.size() / 2;
    }

    float line_slope = 0;
    float line_intercept = 0;

    linearRegression(right_cloud, line_slope, line_intercept);

    // 异常数据过滤
    if (fabs(line_slope) < slope_thresh_ || fabs(line_slope) > 0.1) {
        return false;
    }
    if (do_debug_) {

        saveFailData(right_cloud, current_save_id_, line_slope, line_intercept, seeds_num_);
    }
    current_save_id_++;
    // 找拐点
    while (0 != point_queue.size()) {
        point_queue.pop();
    }

    int over_num = 0;
    int inter_val = 2;
    int current_id = 0;
    for (int i = seeds_num_; i < right_cloud->points.size() - inter_val; i += inter_val) {
        auto point = right_cloud->points[i];
        float dis = fabs(line_intercept + line_slope * point.y - point.x) / sqrt(1 + line_slope * line_slope);
        if (dis > heigt_thresh_) {
            over_num++;
            point_queue.push(point);
            if (over_thresh_num_ == over_num) {
                current_id = i;
                // std::cout << " current id " << current_id << " " << dis << std::endl;
                break;
            }
        } else {
            // 回零
            over_num = 0;
            while (0 != point_queue.size()) {
                point_queue.pop();
            }
        }
    }
    if (0 == current_id || 0 == point_queue.size()) {
        // can't get the corner
        return false;
    }

    pcl::PointXYZI corner_point = point_queue.front();
    slope_points_->points.clear();
    // 获取顶点。
    point_top_.x = 0;
    point_top_.y = 0;
    for (int i = current_id; i < right_cloud->points.size(); i++) {
        auto point = right_cloud->points[i];
        if (fabs(point.y) > point_top_.y) {
            point_top_ = point;
        }
        slope_points_->points.push_back(point);
    }

    slope_dis_ = fabs(point_top_.y);

    if (slope_dis_ < 6.0) {
        return false;
    }
    // 挡墙的高度。认为其为直角边。非斜边height
    // use average x
    // float tem_x =
    height_ = fabs(point_top_.x - average_x_);

    float corner_top_dis = std::sqrt((point_top_.x - corner_point.x) * (point_top_.x - corner_point.x) + (point_top_.y - corner_point.y) * (point_top_.y - corner_point.y));
    float dis_l = fabs(point_top_.y - corner_point.y);
    float cos_value = (1 / std::sqrt(1 + line_slope * line_slope));
    // green_height_ = height * cos + (dis_l - height * tan)*sin + diff
    green_height_ = height_ * cos_value + (dis_l - height_ * cos_value) * cos_value * line_slope + fabs(corner_point.x - average_x_);
    height_pub_data_ = height_;
    slope_dis_pub_data_ = slope_dis_;
    slope_angle_pub_data_ = fabs(line_slope) * 100;

    //     if (do_debug_) {
    // std::cout
    //     << "current_id the height and slope angle " << right_cloud->points.size() << "  " << current_id << " " << height_ << " " << green_height_ << "  " << slope_angle_pub_data_ << " " << slope_dis_ << " " << average_x_ << std::endl;
    //     }
    return true;
}

// bool SlopeCalculation::fitLine(pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud)
// {
//     // fit line
//     int num = 0;
//     std::vector<float> points_x_data;
//     // average_x for the low point x
//     float average_x = 0;
//     float average_y = 0;
//     for (int i = 0; i < seeds_num_; i++) {
//         auto point = right_cloud->points[i];
//         if (point.x < 1.5) {
//             num++;
//         }
//         x_data_(i) = point.y;
//         y_data_(i) = point.x;
//         average_x += point.x;
//         average_y += point.y;
//         points_x_data.push_back(point.x);
//     }
//     average_x = average_x / seeds_num_;
//     average_y = average_y / seeds_num_;
//     if (num > seeds_thresh_) {
//         return false;
//     }

//     lesq_.doSolve(x_data_, y_data_);
//     coeffs_ = lesq_.getCoeffs();

//     // find the corner 拐点
//     // 清零
//     while (0 != point_queue.size()) {
//         point_queue.pop();
//     }

//     int over_num = 0;
//     int inter_val = 2;
//     int current_id = 0;
//     for (int i = seeds_num_; i < right_cloud->points.size() - inter_val; i += inter_val) {
//         auto point = right_cloud->points[i];
//         float dis = fabs(coeffs_(0) + coeffs_(1) * point.y - point.x) / sqrt(1 + coeffs_(1) * coeffs_(1));
//         if (dis > heigt_thresh_) {
//             over_num++;
//             point_queue.push(point);
//             if (over_thresh_num_ == over_num) {
//                 current_id = i;
//                 // std::cout << " current id " << current_id << " " << dis << std::endl;
//                 break;
//             }
//         } else {
//             // 回零
//             over_num = 0;
//             while (0 != point_queue.size()) {
//                 point_queue.pop();
//             }
//         }
//     }
//     if (0 == current_id || 0 == point_queue.size()) {
//         // can't get the corner
//         return false;
//     }

//     // if (do_debug_) {
//     //     saveFailData(right_cloud, current_save_id_, coeffs_(1), coeffs_(0), seeds_num_);
//     // }
//     // current_save_id_++;

//     // fit once more time
//     pcl::PointCloud<pcl::PointXYZI>::Ptr fit_cloud(new pcl::PointCloud<pcl::PointXYZI>);

//     for (int i = 0; i < current_id - over_thresh_num_ * over_num_scale_; i++) {
//         auto point = right_cloud->points[i];
//         fit_cloud->points.push_back(point);
//     }
//     float line_slope = 0;
//     float line_intercept = 0;

//     line_slope = coeffs_(1);
//     line_intercept = coeffs_(0);

//     // if (fabs(coeffs_(1)) > 0.05) {
//     //     line_slope = coeffs_(1);
//     //     line_intercept = coeffs_(0);
//     // } else {
//     //     linearRegression(fit_cloud, line_slope, line_intercept);
//     // }
//     // 异常数据过滤
//     if (fabs(line_slope) < 0.01 || fabs(line_slope) > 0.1) {
//         return false;
//     }
//     if (do_debug_ || fabs(line_slope) > 0.05) {
//         std::cout << " before after " << coeffs_(1) << " " << coeffs_(0) << " " << line_slope << " " << line_intercept << std::endl;
//         saveFailData(right_cloud, current_save_id_, line_slope, line_intercept, fit_cloud->points.size());
//     }
//     current_save_id_++;

//     pcl::PointXYZI corner_point = point_queue.front();
//     pcl::PointXYZI first_point = right_cloud->points[0];
//     slope_points_->points.clear();
//     // 获取顶点。
//     point_top_.x = 0;
//     point_top_.y = 0;
//     for (int i = current_id; i < right_cloud->points.size(); i++) {
//         auto point = right_cloud->points[i];
//         if (fabs(point.y) > point_top_.y) {
//             point_top_ = point;
//         }
//         slope_points_->points.push_back(point);
//     }

//     slope_dis_ = fabs(point_top_.y);
//     // 挡墙的高度。认为其为直角边。非斜边height
//     // use average x
//     // float tem_x =
//     height_ = fabs(point_top_.x - average_x);

//     height_pub_data_ = height_;
//     slope_dis_pub_data_ = slope_dis_;
//     slope_angle_pub_data_ = fabs(line_slope) * 100;
//     // if (fabs(line_slope) < 0.03) {
//     //     if ((fabs(line_slope) + fabs(r_use_)) < 0.02 || (fabs(line_slope) + fabs(r_use_)) > 0.1) {
//     //         return false;
//     //     } else {
//     //         slope_angle_pub_data_ = fabs(line_slope) * 100 + fabs(r_use_) * 100;
//     //     }
//     // } else {
//     //     slope_angle_pub_data_ = fabs(line_slope) * 100;
//     // }

//     if (do_debug_) {
//         std::cout
//             << "current_id the height and slope angle " << current_id << " " << height_ << " " << slope_angle_pub_data_ << " " << slope_dis_ << " " << average_x << std::endl;
//     }

//     return true;
// }

bool SlopeCalculation::pointPreProcess(const pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud)
{
    // TODO: filter the points
    // 1、前面的点云需要进行滤波。（kalman）
    // 2+1、 后面的点云需要过滤。(distance 远的， 近的(使用其一坐标轴))
    if (right_cloud->points.size() < 30) {
        std::cout << " the points size " << right_cloud->points.size() << std::endl;
        return false;
    }

    bool success = false;
    if (fitLine(right_cloud)) {
        success = true;
    }

    // pub results
    if (0 != slope_points_->points.size()) {
        smoothResults();
        displaySlopeMsg(point_top_);
        displaySlopePoints(slope_points_);
    }

    return success;
}

void SlopeCalculation::pointsCallBack(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pointT2PointXYZI(cloud, out_cloud);

    if (!pointPreProcess(out_cloud)) {
        obtainDealRingCloud(cloud);
    }
    // // pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // out_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // std::cout << " the out_cloud size " << out_cloud->points.size() << std::endl;

    // pointT2PointXYZI(cloud, out_cloud);
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*out_cloud, output); // point cloud msg -> ROS msg
    // output.header = msg.header;
    // point_pub_.publish(output); // publish
    // std::cout << " pub " << out_cloud->points.size() << std::endl;
}

// 定义回调函数，用于处理接收到的Vector3Stamped消息
void SlopeCalculation::rtkCallBack(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    // 获取消息中的时间戳
    ros::Time timestamp = msg->header.stamp;
    // 获取消息中的向量数据
    float y = msg->vector.x;
    float p = msg->vector.y;
    float r = msg->vector.z;

    // smooth
    if (smooth_data_number_ != r_list_.size()) {
        if (r != r_pre_) {
            r_list_.push_back(r);
            r_pre_ = r;
        }
    } else {
        std::vector<float> r_data;
        std::list<float>::iterator it;
        for (it = r_list_.begin(); it != r_list_.end(); it++) {
            r_data.push_back(*it);
        }
        std::sort(r_data.begin(), r_data.end());
        r_use_ = r_data[r_data.size() * rtk_scale_ - 1];

        if (smooth_data_number_ == r_data.size()) {
            r_list_.pop_front();
            r_list_.push_back(r);
        }
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(r, p, y);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    // ROS_INFO("Received Vector3Stamped message at time %f: [%.2f, %.2f, %.2f]",
    //          timestamp.toSec(), y, p, r);
}

void SlopeCalculation::createRosPubSub()
{
    point_sub_ = nh_.subscribe("/ouster/points", 100, &SlopeCalculation::pointsCallBack, this);
    rtk_sub_ = nh_.subscribe("/fixposition/ypr", 10, &SlopeCalculation::rtkCallBack, this);

    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output/pointclouds1", 1);
    slope_result_pub_ = nh_.advertise<std_msgs::String>("/slope_results", 1, true);
    slope_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/slope_point", 1, true);
    slope_text_pub_ = nh_.advertise<visualization_msgs::Marker>("/slope_maker_msgs", 1, true);
}
