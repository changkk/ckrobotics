#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "contact_sensor/jetsonGPIO.h"
#include "jetsonGPIO.c"

int main(int argc, char **argv)
{
        ros::init(argc, argv, "contact_node");

        ros::NodeHandle n;



        ros::Publisher pub = n.advertise<std_msgs::Bool>("contact_sensors", 100);

        ros::Rate loop_rate(20);


        jetsonTX1GPIONumber pushButton = gpio38; // Input


        // Make the button and led available in user space
        gpioExport(pushButton);

        gpioSetDirection(pushButton,inputPin);

        unsigned int value = low;
        int counter = 0, threshold = 5;
        //ros::Subscriber sub0 = n.subscribe("/hexacopter0/perception/targets", 1, hex0_targs);

        while (ros::ok()){

                gpioGetValue(pushButton, &value);

                std_msgs::Bool msg;

                if (value == high)
                {
                    counter++;
                    if( counter >= threshold)
                    {
                        msg.data = true;
                        counter = threshold;
                    }

                }
                else
                {
                    counter = 0;
                    msg.data = false;
                }

                pub.publish(msg);



                //ROS_INFO("Target Database Refreshed");

                ros::spinOnce();
                loop_rate.sleep();


        }


        return 0;
}




