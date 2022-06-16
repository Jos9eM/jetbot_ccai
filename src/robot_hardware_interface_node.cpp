#include <jetbot_hw_interface/robot_hardware_interface.h>

//namesapce i2c_ros

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    pidLeft.SetMode(AUTOMATIC);
    pidLeft.SetSampleTime(1);
    pidLeft.SetOutputLimits(-115, 115);

    pidRight.SetMode(AUTOMATIC);
    pidRight.SetSampleTime(1);
    pidRight.SetOutputLimits(-115, 115);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=30;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
	
	for(int i=0; i<2; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    pidLeft.Compute();
    pidRight.Compute();
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {
    uint8_t rbuff[1];
    int x;

    encoder_left.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    inputLeft = x;
    left_motor_pos+=angles::from_degrees((double)x);
    joint_position_[0]=left_motor_pos;

    encoder_right.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    inputRight = x;
    right_motor_pos+=angles::from_degrees((double)x);
    joint_position_[1]=right_motor_pos;

    //ROS_INFO("pos=%.2f x=%d ",pos,x);
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

	uint8_t wbuff[2];

    int velocity,result;
    
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(left_prev_cmd!=velocity)
    {
        setPointL = velocity;
        if(auto motor { hat.getMotor (1) }){
            motor->setSpeed (abs(outPWML));
            if(outPWML > 0){
                motor->run (AdafruitDCMotor::kForward);
            } else if (outPWML <0){
                motor->run (AdafruitDCMotor::kBackward);
            }
            // release the motor after use
            motor->run (AdafruitDCMotor::kRelease);
        }
	    //result = hat.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
        setPointR = velocity;

        if(auto motor { hat.getMotor (2) }){
            motor->setSpeed (abs(outPWMR));
            if(outPWMR > 0){
                motor->run (AdafruitDCMotor::kForward);
            } else if (outPWMR < 0){
                motor->run (AdafruitDCMotor::kBackward);
            }
            // release the motor after use
            motor->run (AdafruitDCMotor::kRelease);
        }
	    //result = right_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
    }


		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "jetbot_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
