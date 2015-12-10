#include <python2.7/Python.h> 
#include <iostream>
#include <stdio.h>
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

      // Catch keyboard interrupts to stop node.
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
      run_prosody_analysis();

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

  int call_python_method(int argc, char *argv[])
  {
    std::cout << "call_python_method" << std::endl;
    PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pArgs;

    if (argc < 3) 
    {
      printf("Usage: exe_name python_source function_name\n");
      return 1;
    }

    // Initialize the Python Interpreter
    Py_Initialize();

    // Build the name object
    pName = PyString_FromString(argv[1]);
    std::cout << "pName: " << argv[1] << std::endl;


    // Load the module object
    pModule = PyImport_Import(pName);

    // pDict is a borrowed reference 
    pDict = PyModule_GetDict(pModule);

    // pFunc is also a borrowed reference 
    pFunc = PyDict_GetItemString(pDict, argv[2]);

    if (PyCallable_Check(pFunc)) 
    {
      // Prepare the argument list for the call
      if( argc > 3 )
      {
        pArgs = PyTuple_New(argc - 3);
        for (int i = 0; i < argc - 3; i++)
        {
          pValue = PyInt_FromLong(atoi(argv[i + 3]));
          if (!pValue)
          {
            PyErr_Print();
            return 1;
          }
          PyTuple_SetItem(pArgs, i, pValue);    
        }

        pValue = PyObject_CallObject(pFunc, pArgs);

        if (pArgs != NULL)
        {
          Py_DECREF(pArgs);
        }
      } 
      else
      {
        pValue = PyObject_CallObject(pFunc, NULL);
      }

      if (pValue != NULL) 
      {
        std::cout << "Return of call" << PyInt_AsLong(pValue) << std::endl;
        Py_DECREF(pValue);
      }
      else 
      {
        PyErr_Print();
      }

    // some code omitted...
    }

    // Clean up
    Py_DECREF(pModule);
    Py_DECREF(pName);

    // Finish the Python Interpreter
    Py_Finalize();
  }


  int run_prosody_analysis()
  {
    std::cout << "Running prosody script." << std::endl;

    // Configure python script related strings
    std::string path = "~/catkin_ws/src/comprobo15/AudioDog/src/hello_world.py";
    std::ostringstream python_path;
    python_path << "sys.path.append(\"" << path << "\")";

    // http://www.codeproject.com/Articles/11805/Embedding-Python-in-C-C-Part-I
    int nargs = 3;
    char* args[] = {"", "hello_world", "main"};
    call_python_method(nargs, args);

    // Py_Initialize();  // start python interpreter

    // PyRun_SimpleString("import sys");
    // PyRun_SimpleString(python_path.str().c_str());

    std::cout << "    Read:"<< std::endl;
    std::cout << "    Finished" << std::endl;
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