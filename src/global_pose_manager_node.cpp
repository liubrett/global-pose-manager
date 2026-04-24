#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int8.h>

class GlobalPoseManager {
private:
    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber estop_sub_;
    ros::Publisher  smoothed_pose_pub_;
    ros::Publisher  initial_pose_pub_;
    ros::ServiceServer rescue_srv_;
    ros::ServiceServer toggle_srv_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // --- TF 时空同步 ---
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // --- 状态机 ---
    enum State {
        STATE_0_INIT,        
        STATE_1_TRACKING,    
        STATE_2_BLIND_ODOM   
    };
    State current_state_;
    std::mutex state_mutex_;

    // --- 位姿变换矩阵 ---
    tf2::Transform T_map_odom_target_;  
    tf2::Transform T_map_odom_current_; 
    
    double max_trans_speed_;
    double max_rot_speed_;   
    ros::Time last_odom_time_;
    ros::Time last_rescue_time_;
    
    // --- 滞回防抖计数器 ---
    int confident_frames_count_;
    int lost_frames_count_;

    // --- 外部控制阀门 ---
    bool is_algorithm_enabled_; // 逻辑总闸（当人工介入或地图切换时）
    bool is_estop_active_;      // 物理急停总闸

    // ==========================================
    // 内部引擎：系统状态彻底重置 (清除历史包袱)
    // ==========================================
    void resetSystemState() {
        current_state_ = STATE_0_INIT;
        confident_frames_count_ = 0;
        lost_frames_count_ = 0;
        last_rescue_time_ = ros::Time(0); 
        last_odom_time_ = ros::Time(0);   
        T_map_odom_target_.setIdentity();
        T_map_odom_current_.setIdentity();
        ROS_INFO("[GlobalPoseManager] System state has been reset to INIT.");
    }

    // --- 自动强制拉回 AMCL ---
    void executeAutoRescue(const ros::Time& current_stamp) {
        // 【核心防线】：冷却时间 (Cooldown) 检测！
        if (!last_rescue_time_.isZero() && (ros::Time::now() - last_rescue_time_).toSec() < 5.0) {
            ROS_INFO_THROTTLE(1.0, "[GlobalPoseManager] auto-saving is freezing...");
            return;
        }

        // 1. 查出此时此刻最新的纯 Odom 位置
        geometry_msgs::TransformStamped odom_msg;
        try {
            odom_msg = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_INFO("[GlobalPoseManager] auto-save get Odom message failed! reason: %s", ex.what());
            return;
        }

        geometry_msgs::PoseWithCovarianceStamped rescue_msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // 2. 算出当前的真实全局位置
            tf2::Transform P_odom;
            tf2::fromMsg(odom_msg.transform, P_odom);
            tf2::Transform current_real_pose = T_map_odom_current_ * P_odom;

            // 3. 包装成 AMCL 能够听懂的“神谕” (/initialpose)
            
            rescue_msg.header.stamp = current_stamp; // 使用触发时的时间戳
            rescue_msg.header.frame_id = "map";
            tf2::toMsg(current_real_pose, rescue_msg.pose.pose);

            // 赋予科学协方差
            for (int i = 0; i < 36; i++) rescue_msg.pose.covariance[i] = 0.0;
            rescue_msg.pose.covariance[0]  = 0.25;  
            rescue_msg.pose.covariance[7]  = 0.25;  
            rescue_msg.pose.covariance[35] = 0.06;  
            rescue_msg.pose.covariance[14] = 1e-9;
            rescue_msg.pose.covariance[21] = 1e-9;
            rescue_msg.pose.covariance[28] = 1e-9;

            last_rescue_time_ = ros::Time::now(); // 重置冷却时间
        }
        
        // 4. 发射重置信号，并记录冷却时间
        initial_pose_pub_.publish(rescue_msg);
        
        ROS_INFO("[GlobalPoseManager] Auto-Rescued! AMCL has been forced to align with Odom tracking.");
    }


public:
    GlobalPoseManager() : nh_("~"), current_state_(STATE_0_INIT), tf_listener_(tf_buffer_),
                          confident_frames_count_(0), lost_frames_count_(0) {
        
        max_trans_speed_ = 0.05; // 5cm/s 平滑速度
        max_rot_speed_   = 0.05; // 约 3度/s

        is_algorithm_enabled_ = true;
        is_estop_active_ = false;

        last_rescue_time_ = ros::Time(0);

        T_map_odom_target_.setIdentity();
        T_map_odom_current_.setIdentity();

        // 订阅高频底层里程计 
        odom_sub_ = nh_.subscribe("/odom_filtered", 100, &GlobalPoseManager::odomCallback, this);
        // 订阅低频全局雷达距离场观测
        pose_sub_ = nh_.subscribe("/amcl_pose", 10, &GlobalPoseManager::PoseCallback, this);
        // 急停话题订阅
        estop_sub_ = nh_.subscribe("/emergency_stop", 1, &GlobalPoseManager::eStopCallback, this);
        
        smoothed_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot/location1", 10);

        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        rescue_srv_ = nh_.advertiseService("rescue_amcl", &GlobalPoseManager::rescueCallback, this);

        toggle_srv_ = nh_.advertiseService("toggle_algorithm", &GlobalPoseManager::toggleAlgorithmCallback, this);
        
        ROS_INFO("Global Pose Manager Initialized.");
    }

    // 急停话题回调
    void eStopCallback(const std_msgs::Int8::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 解析 Int8 数据：0 代表正常，非 0 (通常是 1) 代表急停按下
        bool current_estop_state = (msg->data != 0);

        if (current_estop_state && !is_estop_active_) {
            ROS_INFO("[GlobalPoseManager] emergency stop on!");
            is_estop_active_ = true;
            resetSystemState(); 
        } else if (!current_estop_state && is_estop_active_) {
            ROS_INFO("[GlobalPoseManager] emergency stop off");
            is_estop_active_ = false;
        }
    }

    // 切图or人为校准位姿服务回调
    bool toggleAlgorithmCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 【防反击装甲】：如果刚刚执行过自动救援，无视外部的关闭指令！
        /* if (!req.data && !last_rescue_time_.isZero() && (ros::Time::now() - last_rescue_time_).toSec() < 5.0) {
            ROS_INFO("[GlobalPoseManager] Reject toggle(false)! System is recovering from auto-rescue!");
            res.message = "Toggle rejected due to recent auto-rescue.";
            res.success = false;
            return true;
        } */

        is_algorithm_enabled_ = req.data;
        
        if (is_algorithm_enabled_) {
            ROS_INFO("[GlobalPoseManager] map_cleaning completed.");
            res.message = "Manager Enabled.";
        } else {
            ROS_INFO("[GlobalPoseManager] map cleaning");
            resetSystemState(); 
            res.message = "Manager Disabled and Reset.";
        }
        res.success = true;
        return true;
    }


    // --- 大脑：处理低频、带延迟的全局雷达观测 ---
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

        // 【第一道防线】：物理与逻辑总闸拦截
        if (!is_algorithm_enabled_ || is_estop_active_) return;

        double cov_x = msg->pose.covariance[0];
        double cov_y = msg->pose.covariance[7];
        double cov_yaw = msg->pose.covariance[35];

        // 木桶原理求最大协方差
        double max_cov = std::max({cov_x, cov_y, cov_yaw});
        bool is_confident = (max_cov < 0.8);  // 严格高置信度
        bool is_degraded  = (max_cov > 1.5);  // 轻微退化
        bool is_fatal = (max_cov > 5.0);    // 已崩溃

        // 时空回溯查找历史 Odom 位姿
        geometry_msgs::TransformStamped odom_to_base_msg;
        try {
            // 精确查找雷达采集那一刻 (msg->header.stamp) 的 odom -> base_link
            odom_to_base_msg = tf_buffer_.lookupTransform("odom", "base_link", msg->header.stamp, ros::Duration(0.1));
        } catch (tf2::TransformException &ex) {
            ROS_INFO_THROTTLE(1.0, "TF Sync Warning: Waiting for historical odom... %s", ex.what());
            return; // 找不到历史对齐数据，坚决不进行错误修正
        }

        tf2::Transform P_odom_hist;
        tf2::fromMsg(odom_to_base_msg.transform, P_odom_hist);
        tf2::Transform P_map_obs;
        tf2::fromMsg(msg->pose.pose, P_map_obs);

        // ==== 错误位姿拦截 =====
        tf2::Transform new_T_target = P_map_obs * P_odom_hist.inverse();

        std::unique_lock<std::mutex> lock(state_mutex_);

        if (!is_algorithm_enabled_ || is_estop_active_) return;

        // 空间门控，飞车拦截
        if(current_state_ != STATE_0_INIT && is_confident) {
            // 计算帧之间的位移
            double dx = new_T_target.getOrigin().x() - T_map_odom_target_.getOrigin().x();
            double dy = new_T_target.getOrigin().y() - T_map_odom_target_.getOrigin().y();
            double jump_dist = std::hypot(dx, dy);

            // 计算帧之间的角度偏差
            tf2::Quaternion q_new = new_T_target.getRotation();
            tf2::Quaternion q_old = T_map_odom_target_.getRotation();
            q_new.normalize();
            q_old.normalize();
            double jump_angle_rad = q_new.angleShortestPath(q_old);
            // 两帧之间amcl经历了多少时间
            double dt = (msg->header.stamp - last_odom_time_).toSec();
            if(dt < 0.01) dt = 0.1; // 防止除以零

            if(jump_dist > 0.3 || jump_angle_rad > 0.17 || (jump_angle_rad / dt) > 0.80) {
                ROS_INFO_THROTTLE(1.0, "[GlobalPoseManager] Spatial gate triggered! jump_dist: %.2fm, jump_angle_rad: %.2fdegree, anuglar_velocity: %.2f rad/s",
                jump_dist, jump_angle_rad * 180.0 / M_PI, jump_angle_rad / dt);

                // 强行判定为退化位姿
                is_confident = false;
                is_degraded = true;
                is_fatal = true;
            }
        }

        if(is_fatal) {
           // 【断头台触发】：无视任何计数器，立刻强行切入盲走自保！
            if (current_state_ != STATE_2_BLIND_ODOM) {
                current_state_ = STATE_2_BLIND_ODOM;
                ROS_INFO("[GlobalPoseManager] FATAL ERROR: AMCL crashed, switch to blind odom mode.");
            }

            lost_frames_count_ = 0;
            confident_frames_count_ = 0;

            lock.unlock();

            executeAutoRescue(msg->header.stamp);
            return; // 直接返回，绝对不允许后面的代码碰到 T_map_odom_target_ ！！！ 
        }

        if (is_confident) {
            confident_frames_count_++;
            lost_frames_count_ = 0;
        } else if (is_degraded) {
            lost_frames_count_++;
            confident_frames_count_ = 0;
        }
        

        // ----- 状态切换 -----
        if (current_state_ == STATE_0_INIT && confident_frames_count_ >= 3) {
            T_map_odom_target_ = new_T_target;
            T_map_odom_current_ = new_T_target;
            confident_frames_count_ = 0;
            lost_frames_count_ = 0;
            current_state_ = STATE_1_TRACKING;
            ROS_INFO("State: INIT -> TRACKING. System Online.");
        } 
        else if (current_state_ == STATE_1_TRACKING) {
            if (lost_frames_count_ >= 3) { // 3帧恶化就判定丢锁
                lost_frames_count_ = 0;
                confident_frames_count_ = 0;
                current_state_ = STATE_2_BLIND_ODOM;
                executeAutoRescue(msg->header.stamp);
                ROS_INFO("State: TRACKING -> BLIND_ODOM. Freezing TF target.");
            } else if(!is_degraded) {
                T_map_odom_target_ = new_T_target; // 正常刷新目标
            } else {
                ROS_INFO("An abnormal frame has been detected. A total of %d frames have been accumulated so far.", lost_frames_count_);
            }
        } 
        else if (current_state_ == STATE_2_BLIND_ODOM) {
            if (confident_frames_count_ >= 3) { // 极度严苛：连续3帧优质观测才允许恢复
                T_map_odom_target_ = new_T_target;
                confident_frames_count_ = 0;
                lost_frames_count_ = 0;
                current_state_ = STATE_1_TRACKING;
                ROS_INFO("State: BLIND_ODOM -> TRACKING. Target locked, smoothing initiated.");
            }
        }
    }

    // --- 心脏：被高频 Odom 驱动的同步插值与发布 ---
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 【防线】：总闸拦截
        if (!is_algorithm_enabled_ || is_estop_active_) {
            last_odom_time_ = msg->header.stamp; // 更新时间防跳变
            return; 
        }
        
        if (current_state_ == STATE_0_INIT) return; // 未初始化前保持静默

        ros::Time current_time = msg->header.stamp;
        if (last_odom_time_.isZero()) {
            last_odom_time_ = current_time;
            return;
        }
        
        double dt = (current_time - last_odom_time_).toSec();
        last_odom_time_ = current_time;
        if (dt <= 0.0) return;

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            // A. 取代异步定时器，在同频回调中执行平滑插值
            if (current_state_ == STATE_1_TRACKING) {
                /* tf2::Vector3 current_pos = T_map_odom_current_.getOrigin();
                tf2::Vector3 target_pos  = T_map_odom_target_.getOrigin();
                tf2::Quaternion current_q = T_map_odom_current_.getRotation();
                tf2::Quaternion target_q  = T_map_odom_target_.getRotation();

                double dist_error = current_pos.distance(target_pos);
                double angle_error = current_q.angleShortestPath(target_q);

                // 只有误差大于极小阈值才插值，防止浮点抖动
                if (dist_error > 0.001 || angle_error > 0.001) {
                    double trans_step = std::min(1.0, (max_trans_speed_ * dt) / (dist_error + 1e-6));
                    double rot_step   = std::min(1.0, (max_rot_speed_ * dt) / (angle_error + 1e-6));

                    T_map_odom_current_.setOrigin(current_pos.lerp(target_pos, trans_step));
                    T_map_odom_current_.setRotation(current_q.slerp(target_q, rot_step));
                }  */
                T_map_odom_current_ = T_map_odom_target_;
            }

            // B. 同频发布 TF 树 (保证 map -> odom 时间戳与 Odom 严格一致)
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = current_time;
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "odom";
            transformStamped.transform = tf2::toMsg(T_map_odom_current_);
            //tf_broadcaster_.sendTransform(transformStamped);

            // C. 发布带协方差的最终基座系全局位姿
            tf2::Transform current_odom_pose;
            tf2::fromMsg(msg->pose.pose, current_odom_pose);
            tf2::Transform final_pose = T_map_odom_current_ * current_odom_pose;
            
            
            pose_msg.header.stamp = current_time;
            pose_msg.header.frame_id = "map";
            tf2::toMsg(final_pose, pose_msg.pose.pose);
        }
        
        // 此处可结合当前状态机状态动态填充 pose_msg.pose.covariance，供下游规划器参考
        smoothed_pose_pub_.publish(pose_msg);
    }


    // --- 绝杀技能：将后台正确的盲走位姿，反向喂给 AMCL 进行强行拉回 ---
    bool rescueCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 1. 查出此时此刻最新的纯 Odom 位置
        geometry_msgs::TransformStamped odom_msg;
        try {
            odom_msg = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        } catch (...) {
            ROS_ERROR("[GlobalPoseManager] Failed to obtain the latest odometry. Recovery unsuccessful!");
            return false;
        }

        // 2. 利用我们后台默默维护的 T_map_odom_current_ 算出当前的真实全局位置
        tf2::Transform P_odom;
        tf2::fromMsg(odom_msg.transform, P_odom);
        tf2::Transform current_real_pose = T_map_odom_current_ * P_odom;

        // 3. 包装成 AMCL 能够听懂的“神谕” (/initialpose)
        geometry_msgs::PoseWithCovarianceStamped rescue_msg;
        rescue_msg.header.stamp = odom_msg.header.stamp;
        rescue_msg.header.frame_id = "map";
        tf2::toMsg(current_real_pose, rescue_msg.pose.pose);

        // 赋予它一个科学且健康的初始协方差 (让 AMCL 无条件信任)
        for (int i = 0; i < 36; i++) rescue_msg.pose.covariance[i] = 0.0;
        rescue_msg.pose.covariance[0]  = 0.25;  // X 的方差
        rescue_msg.pose.covariance[7]  = 0.25;  // Y 的方差
        rescue_msg.pose.covariance[35] = 0.06;  // Yaw 的方差
        rescue_msg.pose.covariance[14] = 1e-9; // Z
        rescue_msg.pose.covariance[21] = 1e-9; // Roll
        rescue_msg.pose.covariance[28] = 1e-9; // Pitch

        // 4. 发射重置信号
        initial_pose_pub_.publish(rescue_msg);
        
        ROS_INFO("[GlobalPoseManager] Successfully published a recovery pose! AMCL has been forcibly brought back on track. ");
        return true;
    }
};

int main(int argc, char** argv) {
    // 强制关闭linux标准输出缓冲
    // 1. 击穿标准输出 (搞定 INFO, DEBUG)
    setvbuf(stdout, NULL, _IONBF, 0);
    // 2. 击穿标准错误 (搞定 WARN, ERROR, FATAL)
    setvbuf(stderr, NULL, _IONBF, 0);

    ros::init(argc, argv, "global_pose_manager");
    // 启用多线程异步调度，保障高频 Odom 回调不会被阻塞
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    
    GlobalPoseManager manager;
    ros::waitForShutdown();
    return 0;
}