#include <iostream>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <neato_node/Bump.h>


// Keyboard Interrupt Handler
volatile sig_atomic_t flag = 0;
void interrupt(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool run()
  {
    signal(SIGINT, interrupt); 
    std::cout << "Woof! I'm awake.\nUse keyboard interrupt ";
    std::cout << "when you want me to stop." << std::endl;

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    // char cmd[50];
    while(nh_.ok()){
      // std::cout << ".";                                            // DEVEL
      if(flag)
      {
        std::cout << "Caught! Keyboard interrupt.";
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        return false;
      }

      // Call function that searches for audio signal
      // func

      // Once audio signal has been found and saved to file, 
      // load the file, run prosody script


      // Prosody script returns a command
      // Depending on the prosody result, execute a movement command.


      // Wait until command is entered
      // std::cin.getline(cmd, 50);
      // cmd[0] = 'x';
      

      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      // //move forward
      // if(cmd[0]=='+'){
      //   base_cmd.linear.x = 0.25;
      // } 
      // //turn left (yaw) and drive forward at the same time
      // else if(cmd[0]=='l'){
      //   base_cmd.angular.z = 0.75;
      //   base_cmd.linear.x = 0.25;
      // } 
      // //turn right (yaw) and drive forward at the same time
      // else if(cmd[0]=='r'){
      //   base_cmd.angular.z = -0.75;
      //   base_cmd.linear.x = 0.25;
      // } 
      // //quit
      // else if(cmd[0]=='.'){
      //   break;
      // }

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
    }
    return true;
  }


  int run_prosody_analysis()
  {
    std::string filename = "hello_world.py";
    std::string command = "python ";
    command += filename;
    system(command.c_str());
    FILE* in = popen(command.c_str(), "r");
    // fscanf(in, ... // or some other method of reading
    pclose(in);
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "audio_dog");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.run();
}