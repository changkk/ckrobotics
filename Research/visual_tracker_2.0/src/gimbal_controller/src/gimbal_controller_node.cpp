#include "SBGC.h"
#include "SBGC_cmd_helpers.cpp"
#include <termios.h> /* POSIX terminal control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/ioctl.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace tf2;



int fd, delay = 5000, port = 0;
double pitch = 0, roll = 0, yaw = 0, roll_s = 0, pitch_s = 0, yaw_s = 0;
bool angle_control_flag = false, speed_control_flag = false;
SBGC_Parser par;
static SBGC_cmd_realtime_data_t rt_data;


// Class to interface with AlexMos gimbal driver
class SBGC_ComObj_derived : public SBGC_ComObj
{
  public:
    virtual uint16_t getBytesAvailable()
    {
        int bytes_available;
        ioctl(fd, FIONREAD, &bytes_available);
        return bytes_available;
    }
    virtual uint8_t readByte()
    {
        uint8_t reby;
        char red;
        int err = read(fd, &red, sizeof red);
        memcpy(&reby, &red, sizeof red);
        // std::cout << "Read:" << reby << std::endl;

        if (err < 0)
        {
            return err;
        }
        else
        {
            return reby;
        }
    }
    virtual void writeByte(uint8_t b)
    {
        char wry;
        memcpy(&wry, &b, sizeof b);
        // std::cout << "Written:" << wry << std::endl;
        write(fd, &wry, sizeof wry);
    }
    // Arduino com port is not buffered, so empty space is unknown.
    virtual uint16_t getOutEmptySpace()
    {
        return 0xFFFF;
    }
};

// Open serial port to gimbal
int open_port(void)
{
    //Open port
    fd = open("/dev/gimbal", O_RDWR | O_NOCTTY | O_NDELAY);/* File descriptor for the port */

    // Set attributes
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd,TCSANOW,&options);

    // Error handling
    if (fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0");
        port = 0;
    }
    else
        fcntl(fd, F_SETFL, FNDELAY);
        port = 1;

    return (fd);
}

// Read incoming desired angles
void gimbal_target_angles_cb(const geometry_msgs::PoseStamped mesg1)
{
    Quaternion q(mesg1.pose.orientation.x, mesg1.pose.orientation.y, mesg1.pose.orientation.z, mesg1.pose.orientation.w);
    Matrix3x3 m(q);
    // m.getRPY(roll, pitch, yaw);


    m.getRPY(pitch, yaw, roll);

    //Limit pitch angles
    if(pitch > 70*M_PI/180)
    {
        pitch = 70*M_PI/180;
    }
    else if(pitch < -35*M_PI/180)
    {
        pitch = -35*M_PI/180;
    }
    //Fix roll to zero, camera should not move in roll
    roll = 0;

    //Hard fix for yaw, ugh. Should use tf properly.
    yaw = -yaw;

    angle_control_flag = true;
    speed_control_flag = false;
}

// Read incoming desired speeds
void gimbal_target_speed_cb(const geometry_msgs::TwistStamped mesg1)
{
    //REP 103 - Roll, Pitch, Yaw
    roll_s = mesg1.twist.angular.x;
    pitch_s = mesg1.twist.angular.y;
    yaw_s = mesg1.twist.angular.z;

    angle_control_flag = false;
    speed_control_flag = true;


}


// Process incoming gimbal data
void process_in_queue() {
	while(par.read_cmd()) {
        // std::cout << "foo" << std::endl;
		SerialCommand &cmd = par.in_cmd;
		
				
		uint8_t error = 0;
		
		switch(cmd.id) {
		// Receive realtime data
		case SBGC_CMD_REALTIME_DATA_3:
            error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
        break;
		case SBGC_CMD_REALTIME_DATA_4:
			error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
        break;
		}
	}
}

// void zero_gimbal_IMU()
// {

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_node");
    
    ros::NodeHandle n;

    ros::Publisher pub_imu_angle = n.advertise<geometry_msgs::PoseStamped>("gimbal_imu_angles", 1);
    ros::Subscriber sub_ang = n.subscribe("gimbal_target_orientation", 1, gimbal_target_angles_cb);
    ros::Subscriber sub_spd = n.subscribe("gimbal_target_speed", 1, gimbal_target_speed_cb);


    ros::Rate loop_rate(30);

    open_port();

    SBGC_ComObj *com_obj;
    SBGC_ComObj_derived d;
    com_obj = &d;

    // SBGC_Parser par;
    par.init(com_obj);
    
    SBGC_cmd_control_t c = {0, 0, 0, 0, 0, 0, 0};

    SBGC_cmd_realtime_data_t p;
    // SerialCommand sc_obj;
    // sc_obj.init(SBGC_CMD_REALTIME_DATA_4);
    // sc_obj.writeWord(50);
    // par.send_cmd(sc_obj, 0);
    
    double imu_angles[3] = {0, 0, 0};
    double enc_angles[3] = {0, 0, 0};
    double yaw_drift = 0;
    geometry_msgs::PoseStamped gm_msg;

    c.mode = SBGC_CONTROL_MODE_ANGLE;
    c.speedROLL = c.speedPITCH = 45*SBGC_SPEED_SCALE;
    c.speedYAW = 60*SBGC_SPEED_SCALE;
    SBGC_cmd_control_send(c,par);
    usleep(delay);

    //Start with Gimbal at zero (partly to confirm it hasnt drifted too much)
    c.mode = SBGC_CONTROL_MODE_ANGLE;
	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(0);
    c.angleROLL = SBGC_DEGREE_TO_ANGLE(0);
	c.angleYAW = SBGC_DEGREE_TO_ANGLE(0);
    SBGC_cmd_control_send(c,par);
    usleep(delay);

    // int var = 8.5;
    double mag_dec = 0;

    tf2::Quaternion q;
    // zero_gimbal_IMU();

    while(ros::ok() && port == 1)
    {
        
        // Update Angles in message and send to gimbal

        //Prioritize speed control over angle, since yolo_detection is more accurate. But both messages will not come in at the same time if the governer_node is working right.
        if (angle_control_flag && !speed_control_flag)
        {
            //For gimbal angle control
            c.mode = SBGC_CONTROL_MODE_ANGLE;
            c.anglePITCH = SBGC_DEGREE_TO_ANGLE(pitch * 180 / M_PI);
            c.angleROLL = SBGC_DEGREE_TO_ANGLE(0);
            c.angleYAW = SBGC_DEGREE_TO_ANGLE(yaw * 180 / M_PI - yaw_drift * 180 / M_PI + mag_dec);
            SBGC_cmd_control_send(c, par);
            usleep(delay);
        }

        if (speed_control_flag)
        {
            //For gimbal speed control
            c.mode = SBGC_CONTROL_MODE_SPEED;
            c.speedROLL = roll_s * SBGC_SPEED_SCALE;
            c.speedPITCH = pitch_s * SBGC_SPEED_SCALE;
            c.speedYAW = yaw_s * SBGC_SPEED_SCALE;
            SBGC_cmd_control_send(c, par);
            usleep(delay);
        }

        // // For fixed gimbal
        // c.mode = SBGC_CONTROL_MODE_ANGLE;
        // c.anglePITCH = SBGC_DEGREE_TO_ANGLE(10);
        // c.angleROLL = SBGC_DEGREE_TO_ANGLE(0);
        // c.angleYAW = SBGC_DEGREE_TO_ANGLE(60-yaw_drift*180/M_PI);
        // // Send angles to gimbal  
        // SBGC_cmd_control_send(c, par);
        // usleep(delay);


        // Retrive Angles from Gimbal IMU
        SerialCommand sc_obj;
        sc_obj.init(SBGC_CMD_REALTIME_DATA_4);      //Send realtime data request
        par.send_cmd(sc_obj, 0);
        usleep(delay/5);
        process_in_queue();                         //Process requested data

        // Angles from gimbal are in ROLL,PITCH,YAW order, in integer values. Converting to radians first and scale as per SBGC encoding
        imu_angles[0] = rt_data.imu_angle[0]*0.02197265625*M_PI/180;//Roll
        imu_angles[1] = rt_data.imu_angle[1]*0.02197265625*M_PI/180;//Pitch
        imu_angles[2] = rt_data.imu_angle[2]*0.02197265625*M_PI/180;//Yaw

        enc_angles[0] = rt_data.rotor_angle[0]*0.02197265625*M_PI/180;//Roll
        enc_angles[1] = rt_data.rotor_angle[1]*0.02197265625*M_PI/180;//Pitch
        enc_angles[2] = rt_data.rotor_angle[2]*0.02197265625*M_PI/180;//Yaw

        yaw_drift = (-enc_angles[2] - imu_angles[2]);

        //Convert to quaternion to publish, Using -enc_angles[2] for yaw due to drift issues.
        // Quaternion q(-enc_angles[YAW], imu_angles[PITCH], imu_angles[ROLL]);
        q.setRPY(imu_angles[PITCH], imu_angles[ROLL], enc_angles[YAW] );

        gm_msg.pose.orientation.w = q.getW();
        gm_msg.pose.orientation.x = q.getX();
        gm_msg.pose.orientation.y = q.getY();
        gm_msg.pose.orientation.z = q.getZ();

        pub_imu_angle.publish(gm_msg);
        

        // std::cout << "Target Angles:" << std::endl << "Roll: " << roll * 180 / M_PI << "    Pitch: " << pitch * 180 / M_PI << " Yaw: " << yaw * 180 / M_PI << std::endl << std::endl;
        // std::cout << "Actual IMU Angles:" << std::endl << "Roll: " << imu_angles[0] * 180 / M_PI << "  Pitch: " << imu_angles[1] * 180 / M_PI << "  Yaw: " << imu_angles[2] * 180 / M_PI << std::endl << std::endl;
        // std::cout << "Actual ENC Angles:" << std::endl << "Roll: " << enc_angles[0] * 180 / M_PI << "  Pitch: " << enc_angles[1] * 180 / M_PI << "  Yaw: " << enc_angles[2] * 180 / M_PI << std::endl << std::endl;
        // std::cout <<  "Yaw_drift: " << yaw_drift* 180 / M_PI << "  Target_Yaw_corrected: " << (-yaw_drift+yaw) * 180 / M_PI << std::endl;
        // std::cout << "------------------" << std::endl;
        
        ros::spinOnce();    
        loop_rate.sleep();

    }
    // std::cout << "Command Sent again, closing" << std::endl;
    //For gimbal speed control
    c.mode = SBGC_CONTROL_MODE_SPEED;
    c.speedROLL = 0 * SBGC_SPEED_SCALE;
    c.speedPITCH = 0 * SBGC_SPEED_SCALE;
    c.speedYAW = 0 * SBGC_SPEED_SCALE;
    SBGC_cmd_control_send(c, par);
    usleep(delay);

    close(fd);
    return(0);
}   
