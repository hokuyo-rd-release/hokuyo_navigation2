#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
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

// Vector3d を geometry_msgs::Vector3  に変換する関数.
geometry_msgs::Vector3 VectorToTwistLinear(const Vector3d vec){
    geometry_msgs::Vector3 ret_msg;
    ret_msg.x = vec(0);
    ret_msg.y = vec(1);
    ret_msg.z = vec(2);
    return ret_msg;
}

// geometry_msgs::Vector3 を Vector3d に変換する関数.
Vector3d TwistLinearToVector(const geometry_msgs::Vector3 msg){
    Vector3d ret_vec;
    ret_vec(0) = msg.x;
    ret_vec(1) = msg.y;
    ret_vec(2) = msg.z;
    return ret_vec;
}

// Vector3d を geometry_msgs::Point に変換する関数.
geometry_msgs::Point VectorToPosition(const Vector3d vec){
    geometry_msgs::Point ret_msg;
    ret_msg.x = vec(0);
    ret_msg.y = vec(1);
    ret_msg.z = vec(2);
    return ret_msg;
}

// geometry_msgs::Point を Vector3d に変換する関数.
Vector3d PositionToVector(const geometry_msgs::Point msg){
    Vector3d ret_vec;
    ret_vec(0) = msg.x;
    ret_vec(1) = msg.y;
    ret_vec(2) = msg.z;
    return ret_vec;
}

// Vector4d を geometry_msgs::Quaternion に変換する関数.
geometry_msgs::Quaternion QuatToOrientation(const Vector4d quat){
    geometry_msgs::Quaternion ret_msg;
    ret_msg.x = quat(0);
    ret_msg.y = quat(1);
    ret_msg.z = quat(2);
    ret_msg.w = quat(3);
    return ret_msg;
}

// geometry_msgs::Quaternion を Vector4d に変換する関数.
Vector4d OrientationToQuat(const geometry_msgs::Quaternion msg){
    Vector4d ret_quat;
    ret_quat(0) = msg.x;
    ret_quat(1) = msg.y;
    ret_quat(2) = msg.z;
    ret_quat(3) = msg.w;
    return ret_quat;
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

class PoseToTF{
private:    

    // ROSの設定.
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    tf::TransformBroadcaster odom_broadcaster;

    std::string sub_pose_topic;
    std::string pub_odom_topic;
    std::string frame_id;
    std::string child_frame_id;

    bool tf_en;
    bool odom_en;
    bool mode_2d;
    bool use_init_R;
    int sub_count;
    int init_num;

    std::vector<Vector3d> vel_log;
    std::vector<Vector3d> pos_log;
    std::vector<Vector4d> quat_log;
    Vector3d vel_sum;
    Vector4d quat_sum;
    Vector3d vel_ave;
    Vector4d quat_ave;
    std::vector<double> time_log;

    Matrix3d R;
    Matrix3d init_R;
    Matrix4d Q;
    std::vector<double> R_arr;
    std::vector<double> position_arr;
    Vector3d init_pos;

public:

    void Callback(const geometry_msgs::PoseStamped& msg){
        sub_count++;
        nav_msgs::Odometry pub_msg;
        pub_msg.header = msg.header;
        pub_msg.header.frame_id = frame_id;
        pub_msg.child_frame_id = child_frame_id;
        Vector3d odom_tmp = PositionToVector(msg.pose.position);
        Vector4d quat_tmp = OrientationToQuat(msg.pose.orientation);

        if(use_init_R){
            if(sub_count < init_num)return;
            
            else if(sub_count == init_num){
                init_R = RotMatFromQuat(quat_tmp);
                R = R * init_R.transpose();
                Vector3d rot_vec_tmp = RotVecFromRotMat(R);
                Vector4d quat_tmp = QuatFromRadAndAxis(rot_vec_tmp.norm(),rot_vec_tmp.normalized());
                Q = QuatMatFromQuat(quat_tmp);
            }
        }

        Vector3d ret_odom = R * odom_tmp + init_pos;
        Vector4d ret_quat = Q * quat_tmp;
        
        if(mode_2d){
            ret_odom(2) = 0.0;
            ret_quat(0) = 0.0;
            ret_quat(1) = 0.0;
        }
        ret_quat.normalize();

        pub_msg.pose.pose.position = VectorToPosition(ret_odom);
        pub_msg.pose.pose.orientation = QuatToOrientation(ret_quat);

        if(odom_en){
            publisher.publish(pub_msg);
        }
        if(tf_en){
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header = pub_msg.header;
            odom_trans.header.seq = 0;
            odom_trans.child_frame_id = child_frame_id;
            odom_trans.transform.translation = VectorToTwistLinear(ret_odom);
            odom_trans.transform.rotation = pub_msg.pose.pose.orientation;
            odom_broadcaster.sendTransform(odom_trans);
        }
    }

    //初期化処理.
    PoseToTF() : nh(),pnh("~"){
        //各rosparamのデフォルト値.
        sub_pose_topic = "pose";
        pub_odom_topic = "odometry_fixframe";
        frame_id = "body";

        child_frame_id = "base_link";
        tf_en = true;
        odom_en = true;
        mode_2d = false;
        sub_count = 0;
        quat_sum << 0.0,0.0,0.0,0.0;
        init_num = 5;

        //rosparamの取得.
        pnh.getParam("sub_pose_topic", sub_pose_topic);
        pnh.getParam("pub_odom_topic", pub_odom_topic);
        pnh.getParam("frame_id", frame_id);
        pnh.getParam("child_frame_id", child_frame_id);
        pnh.getParam("tf_en", tf_en);
        pnh.getParam("odom_en", odom_en);
        pnh.getParam("mode_2d", mode_2d);
        pnh.getParam("use_init_R", use_init_R);
        pnh.getParam("init_num", init_num);

        pnh.param<vector<double>>("R_arr", R_arr, vector<double>());
        pnh.param<vector<double>>("position_arr", position_arr, vector<double>());

        R << R_arr[0] , R_arr[1] , R_arr[2],
            R_arr[3] , R_arr[4] , R_arr[5],
            R_arr[6] , R_arr[7] , R_arr[8];

        init_pos << position_arr[0] , position_arr[1] , position_arr[2];
        
        Vector3d rot_vec_tmp = RotVecFromRotMat(R);
        Vector4d quat_tmp = QuatFromRadAndAxis(rot_vec_tmp.norm(),rot_vec_tmp.normalized());
        Q = QuatMatFromQuat(quat_tmp);

        //publisher,subscriberの設定.
        subscriber = nh.subscribe(sub_pose_topic,10,&PoseToTF::Callback, this);
        
        publisher = nh.advertise<nav_msgs::Odometry>(pub_odom_topic,10);

    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "pose_to_tf");
    PoseToTF pose_to_tf;

    ros::spin();
    return 0;
}
