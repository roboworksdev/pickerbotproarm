#include "unitree_arm_sdk/control/unitreeArm.h"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/int32.hpp"



using namespace UNITREE_ARM;

class Z1ArmROSNode : public rclcpp::Node {
public:
    Z1ArmROSNode() : Node("z1_arm_ros_node"), arm(true) {

        //Timer for end effector relative position broadcaster
        timer_pose_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&Z1ArmROSNode::armBroadcaster,this));
        timer_routine_ = create_wall_timer(std::chrono::milliseconds(5000), std::bind(&Z1ArmROSNode::routineCallback, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        

        //Initalise the arm communication
        std::cout << "STARTING ARM CALIBRATION" << std::endl;
        arm.sendRecvThread->start();
        arm.calibration();
        arm.backToStart();
        std::cout << "FINISHED ARM CALIBRATION" << std::endl;

    }


    ~Z1ArmROSNode() {
        arm.backToStart();
        arm.setFsm(ArmFSMState::PASSIVE);
        arm.sendRecvThread->shutdown();
    }

    
    

private:

    void printState(){
      std::cout<<"------ joint State ------"<<std::endl;
      std::cout<<"qState: "<<arm.lowstate->getQ().transpose()<<std::endl;
      std::cout<<"qdState: "<<arm.lowstate->getQd().transpose()<<std::endl;
      std::cout<<"tauState: "<<arm.lowstate->getTau().transpose()<<std::endl;

      std::cout<<"------ Endeffector Cartesian Posture ------"<<std::endl;
      std::cout<<"roll pitch yaw x y z"<<std::endl;
      std::cout<<arm.lowstate->endPosture.transpose()<<std::endl;
    }




    void armBroadcaster() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "arm_base_link";
        t.child_frame_id = "end_effector";

        t.transform.translation.x = arm.lowstate->endPosture.transpose()[3];
        t.transform.translation.y = arm.lowstate->endPosture.transpose()[4];
        t.transform.translation.z = arm.lowstate->endPosture.transpose()[5];

        tf2::Quaternion q;
        q.setRPY(arm.lowstate->endPosture.transpose()[0], arm.lowstate->endPosture.transpose()[1], arm.lowstate->endPosture.transpose()[2]);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }



    void armMoveToJointAngle(Vec6 jointAngles, double duration) {

        arm.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);
        Vec6 targetPos, lastPos;
        lastPos = arm.lowstate->getQ();
        targetPos << jointAngles;

        UNITREE_ARM::Timer timer(arm._ctrlComp->dt);
        for(int i=0; i<duration; i++)
        {
            arm.q = lastPos*(1-i/duration) + targetPos*(i/duration);
            arm.qd = (targetPos-lastPos)/(duration*arm._ctrlComp->dt);
            arm.setArmCmd(arm.q, arm.qd);
            timer.sleep();
        }
        arm.setFsm(ArmFSMState::JOINTCTRL);
    }


    int move_to_transformation(std::string transformation_frame) {

        std::string to_frame = "arm_base_link";
        std::string from_frame = transformation_frame;
            
        try {
            t = tf_buffer_->lookupTransform(
            to_frame, from_frame,
            tf2::TimePointZero);
            std::cout << "able to transform to " << transformation_frame  << std::endl;
        } catch (const tf2::TransformException & ex) {
            std::cout <<"Could not transform to " << transformation_frame << std::endl;
            return 0;
        }
        
        tf2::Quaternion q(  t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Vec6 posture[2];
        posture[0] << 0, M_PI/2, yaw, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
        auto output = arm.MoveJ(posture[0], 0, 2.0);

        return 1;

    }



    void moveToObservation() {
        arm.setFsm(ArmFSMState::JOINTCTRL);
        Vec6 posture[2];
        posture[0] << 0, 1.2, -1.4, 1.57, 0, 0;
        armMoveToJointAngle(posture[0],1000.0 );
    }


    // Simple routine which moves between three positions. Uses joint position, set position, and move J command. 
    void routineCallback() {
        if (state == 0){
            moveToObservation();
            state = 1;

        } else if (state == 1) {
            state = 2;
            move_to_transformation("lower_position"); // Transformation is defined in static_arm_transformation.cpp

        } else {
            state = 0;
            arm.backToStart();
        }
    }


    geometry_msgs::msg::TransformStamped t;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_pose_;
    rclcpp::TimerBase::SharedPtr timer_routine_;
    rclcpp::TimerBase::SharedPtr timer_wait_;

    double joint_speed = 2.0;
    double cartesian_speed = 0.5;
    int state = 0;

    unitreeArm arm;
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Z1ArmROSNode>());
    rclcpp::shutdown();
    return 0;
}
