#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


// クォータニオンから回転行列を計算する関数.
Matrix3d RotMatFromQuat(const Vector4d quat){
    Matrix3d Rot;
        Rot << 2*(quat(0)*quat(0) + quat(3)*quat(3))-1 , 2*(quat(0)*quat(1) - quat(2)*quat(3)), 2*(quat(0)*quat(2) + quat(1)*quat(3)) ,
        2*(quat(0)*quat(1) + quat(2)*quat(3)) , 2*(quat(1)*quat(1) + quat(3)*quat(3))-1 , 2*(quat(1)*quat(2) - quat(0)*quat(3)) ,
        2*(quat(0)*quat(2) - quat(1)*quat(3)) , 2*(quat(1)*quat(2) + quat(0)*quat(3)) , 2*(quat(2)*quat(2) + quat(3)*quat(3))-1 ;

    return Rot;
}

// 回転行列からクォータニオンを計算する関数.
Vector4d QuatFromRotMat(const Matrix3d Rot){
    Vector4d quat;
    quat << sqrt( Rot(0,0) - Rot(1,1) - Rot(2,2) + 1) / 2.0,
            0.5 * ( Rot(1,0) + Rot(0,1) ) / sqrt( Rot(0,0) - Rot(1,1) - Rot(2,2) + 1),
            0.5 * ( Rot(2,0) + Rot(0,2) ) / sqrt( Rot(0,0) - Rot(1,1) - Rot(2,2) + 1),
            0.5 * ( Rot(2,1) - Rot(1,2) ) / sqrt( Rot(0,0) - Rot(1,1) - Rot(2,2) + 1);
    return quat;
}

// 回転角度と回転軸からクォータニオンを計算する関数.
Vector4d QuatFromRadAndAxis(const double rad, const Vector3d axis){
    double sin_half = sin(0.5*rad);
    double cos_half = cos(0.5*rad);
    Vector4d quaternion;
    quaternion(0) = sin_half * axis(0);
    quaternion(1) = sin_half * axis(1);
    quaternion(2) = sin_half * axis(2);
    quaternion(3) = cos_half;
    return quaternion;
}

// 回転行列から回転ベクトルを計算する関数.
Vector3d RotVecFromRotMat(const Matrix3d& input_rot_mat){
    Matrix3d rot_mat = input_rot_mat;
    double trace = rot_mat.trace();
    double theta;

    Vector3d rot_axis;

    if(trace <= -1.0){
        rot_mat = rot_mat / abs(trace);
        theta = acos(-1);
        rot_axis << sqrt(0.5*(rot_mat(0,0)+1.0)),sqrt(0.5*(rot_mat(1,1)+1.0)),sqrt(0.5*(rot_mat(2,2)+1.0));
    }
    else if(trace >= 3.0){
        rot_mat = rot_mat / abs(trace) * 3.0;
        theta = acos(1);
        rot_axis << 0,0,0;
    }
    else{
        theta = acos( 0.5 * (trace - 1) );
        double sin_theta_inv = 1/sin(theta);
        rot_axis << 0.5*sin_theta_inv * (rot_mat(2,1)-rot_mat(1,2)) , 0.5*sin_theta_inv * (rot_mat(0,2)-rot_mat(2,0)) , 0.5*sin_theta_inv * (rot_mat(1,0)-rot_mat(0,1));
    }
    
    Vector3d rot_vec = theta * rot_axis;
    return rot_vec;
}

// クォータニオンからクォータニオン行列を計算する関数.
Matrix4d QuatMatFromQuat(const Vector4d quat){
    Matrix4d Mat_Q;
    Mat_Q << quat(3) , -quat(2) , quat(1) , quat(0) ,
        quat(2) , quat(3) , -quat(0) , quat(1) ,
        -quat(1) , quat(0) , quat(3) , quat(2) ,
        -quat(0) , -quat(1) , -quat(2) , quat(3) ;

    return Mat_Q;
}

Matrix3d RotCovToMat(const std::array<double, 36>& covariance){
    Matrix3d ret_mat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_mat(i, j) = covariance[6 * (j+3) + (i+3)];
        }
    }
    return ret_mat;
}

std::array<double, 36> MatToRotCov(const Matrix3d& cov_mat){
    std::array<double, 36> ret_msg; 
    for (int i = 0; i < 36; i++)ret_msg[i] = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_msg[6 * (j+3) + (i+3)] = cov_mat(i, j);
        }
    }
    return ret_msg ;
}

std::array<double, 36> MatToRotCov(const Matrix3d& cov_mat, const std::array<double, 36>& orig_msg){
    std::array<double, 36> ret_msg = orig_msg;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_msg[6 * (j+3) + (i+3)] = cov_mat(i, j);
        }
    }
    return ret_msg ;
}

Matrix3d CovToMatrix(const std::array<double, 36>& covariance){
    Matrix3d ret_mat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_mat(i,j) = covariance[6 * j + i];
        }
    }
    return ret_mat;
}

std::array<double, 36> MatrixToCov(const Matrix3d& cov_mat){
    std::array<double, 36> ret_msg; 
    for (int i=0; i<36; i++) ret_msg[i] = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_msg[6 * j + i] = cov_mat(i, j);
        }
    }
    return ret_msg ;
}

std::array<double, 36> MatrixToCov(const Matrix3d& cov_mat , const std::array<double, 36>& orig_msg){
    std::array<double, 36> ret_msg = orig_msg;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret_msg[6 * j + i] = cov_mat(i, j);
        }
    }
    return ret_msg ;
}

geometry_msgs::msg::Point VectorToPosition(const Vector3d& vec){
    geometry_msgs::msg::Point ret_msg;
    ret_msg.x = vec(0);
    ret_msg.y = vec(1);
    ret_msg.z = vec(2);
    return ret_msg;
}

Vector3d PositionToVector(const geometry_msgs::msg::Point& msg){
    Vector3d ret_vec;
    ret_vec(0) = msg.x;
    ret_vec(1) = msg.y;
    ret_vec(2) = msg.z;
    return ret_vec;
}

geometry_msgs::msg::Quaternion QuatToOrientation(const Vector4d& quat){
    geometry_msgs::msg::Quaternion ret_msg;
    ret_msg.x = quat(0);
    ret_msg.y = quat(1);
    ret_msg.z = quat(2);
    ret_msg.w = quat(3);
    return ret_msg;
}

Vector4d OrientationToQuat(const geometry_msgs::msg::Quaternion& msg){
    Vector4d ret_quat;
    ret_quat(0) = msg.x;
    ret_quat(1) = msg.y;
    ret_quat(2) = msg.z;
    ret_quat(3) = msg.w;
    return ret_quat;
}


class OdomFrameChange : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string sub_topic_name_;
    std::string pub_topic_name_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool tf_en_;
    bool initial_tf_en_;
    bool odom_en_;
    bool mode_2d_;
    bool use_init_R_;
    int sub_count_;
    int init_num_;

    std::vector<Vector3d> vel_log_;
    std::vector<Vector3d> pos_log_;
    std::vector<Vector4d> quat_log_;
    Vector3d vel_sum_;
    Vector4d quat_sum_;
    Vector3d vel_ave_;
    Vector4d quat_ave_;
    std::vector<double> time_log_;

    Matrix3d R_;
    Matrix3d init_R_;
    Matrix4d Q_;
    std::vector<double> R_arr_;
    std::vector<double> position_arr_;
    Vector3d init_pos_;

public:
    OdomFrameChange() : Node("odom_frame_changer") {
        // Declare and get parameters
        R_arr_.resize(9);
        R_arr_={1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
        position_arr_.resize(3);
        position_arr_={0.0,0.0,0.0};

        this->declare_parameter<std::string>("sub_topic_name", "hokuyo_lio/lidar_odom");
        this->declare_parameter<std::string>("pub_topic_name", "hokuyo_lio/trans_odom");
        this->declare_parameter<std::string>("frame_id", "body");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<bool>("tf_en", true);
        this->declare_parameter<bool>("initial_tf_en", true);
        this->declare_parameter<bool>("odom_en", true);
        this->declare_parameter<bool>("mode_2d", false);
        this->declare_parameter<bool>("use_init_R", false);
        this->declare_parameter<int>("init_num", 5);
        this->declare_parameter<std::vector<double>>("R_arr", R_arr_);
        this->declare_parameter<std::vector<double>>("position_arr", position_arr_);

        this->get_parameter("sub_topic_name", sub_topic_name_);
        this->get_parameter("pub_topic_name", pub_topic_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);
        this->get_parameter("tf_en", tf_en_);
        this->get_parameter("initial_tf_en", initial_tf_en_);
        this->get_parameter("odom_en", odom_en_);
        this->get_parameter("mode_2d", mode_2d_);
        this->get_parameter("use_init_R", use_init_R_);
        this->get_parameter("init_num", init_num_);
        this->get_parameter("R_arr", R_arr_);
        this->get_parameter("position_arr", position_arr_);


        vel_sum_ = Vector3d::Zero();
        quat_sum_ = Vector4d::Zero();
        sub_count_ = 0;
        R_ << R_arr_[0], R_arr_[1], R_arr_[2],
              R_arr_[3], R_arr_[4], R_arr_[5],
              R_arr_[6], R_arr_[7], R_arr_[8];

        init_pos_ << position_arr_[0], position_arr_[1], position_arr_[2];

        Vector3d rot_vec_tmp = RotVecFromRotMat(R_);
        Vector4d quat_tmp = QuatFromRadAndAxis(rot_vec_tmp.norm(), rot_vec_tmp.normalized());
        Q_ = QuatMatFromQuat(quat_tmp);

        publisher_ = create_publisher<nav_msgs::msg::Odometry>(pub_topic_name_, 10);
        subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            sub_topic_name_, 10,
            std::bind(&OdomFrameChange::callback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        if (tf_en_ && initial_tf_en_) {
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = this->now();
            odom_trans.header.frame_id = frame_id_;
            odom_trans.child_frame_id = child_frame_id_;
            odom_trans.transform.translation.x = 0.0;
            odom_trans.transform.translation.y = 0.0;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation.x = 0.0;
            odom_trans.transform.rotation.y = 0.0;
            odom_trans.transform.rotation.z = 0.0;
            odom_trans.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(odom_trans);
        }
    }

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        sub_count_++;
        auto pub_msg = *msg;
        pub_msg.header.frame_id = frame_id_;
        pub_msg.child_frame_id = child_frame_id_;

        Vector3d odom_tmp = PositionToVector(pub_msg.pose.pose.position);
        Vector4d quat_tmp = OrientationToQuat(pub_msg.pose.pose.orientation);

        if(use_init_R_){
            if(sub_count_ < init_num_)return;
            
            else if(sub_count_ == init_num_){
                init_R_ = RotMatFromQuat(quat_tmp);
                R_ = R_ * init_R_.transpose();
                Vector3d rot_vec_tmp = RotVecFromRotMat(R_);
                Vector4d quat_tmp = QuatFromRadAndAxis(rot_vec_tmp.norm(),rot_vec_tmp.normalized());
                Q_ = QuatMatFromQuat(quat_tmp);
            }
        }

        Vector3d ret_odom = R_ * odom_tmp + init_pos_;
        Vector4d ret_quat = Q_ * quat_tmp;
        
        if(mode_2d_){
            ret_odom(2) = 0.0;
            ret_quat(0) = 0.0;
            ret_quat(1) = 0.0;
        }
        ret_quat.normalize();

        pub_msg.pose.pose.position = VectorToPosition(ret_odom);
        pub_msg.pose.pose.orientation = QuatToOrientation(ret_quat);

        if (odom_en_) {
            publisher_->publish(pub_msg);
        }

        if (tf_en_) {
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header = pub_msg.header;
            odom_trans.child_frame_id = child_frame_id_;
            odom_trans.transform.translation.x = pub_msg.pose.pose.position.x;
            odom_trans.transform.translation.y = pub_msg.pose.pose.position.y;
            odom_trans.transform.translation.z = pub_msg.pose.pose.position.z;
            odom_trans.transform.rotation = pub_msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(odom_trans);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomFrameChange>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
