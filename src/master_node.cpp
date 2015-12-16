#include <python2.7/Python.h> 
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <neato_node/Bump.h>


//http://stackoverflow.com/questions/7998816/do-pyimport-importmodule-and-import-statement-load-into-different-namespace
//http://www.tutorialspoint.com/python/python_further_extensions.htm
//http://members.gamedev.net/sicrane/articles/EmbeddingPythonPart1.html
//https://docs.python.org/2/extending/embedding.html


// Keyboard Interrupt Handler
volatile sig_atomic_t flag = 0;
void interrupt(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

class RobotDriver
{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;

  bool cmd_found = false;   // true: command found, acting; false: listening
  int cmd_state = 0;        // 0: stop, 1: follow, 2: good boy
  int current_angle, start_angle, target_angle;

public:
  // ROS Node Initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  // Main loop
  bool run()
  {
    int result_detect, result_pros, result_loc;
    signal(SIGINT, interrupt); 
    std::cout << std::endl << "Woof! I'm awake.\nUse keyboard interrupt ";
    std::cout << "when you want me to stop." << std::endl << std::endl;
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while(nh_.ok()){
      std::cout << std::endl;

      // Default twist commands: set stationary
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      // Catch keyboard interrupts to stop node.
      if(flag)
      {
        std::cout << "Caught! Keyboard interrupt.";
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        return false;
      }

      // Call function that searches for audio signal, saves to wav if found
      result_detect = detect_command();

      // If an audio signal has been found, process it
      if (result_detect != -1){

        // Once audio signal has been found and saved to file, 
        // load the file, run prosody script, determine command
        result_pros = analyze_prosody();

        // If the prosody analysis did not fail:
        if (result_pros != -1){

          // Set the state to the command was found
          cmd_state = result_pros;

          // If the command was to "follow", find the angle relative to src
          if(cmd_state == 1){
            target_angle = determine_src_dir();
          }
        }
      }

      
      //Follow
      if(cmd_state == 1){
        std::cout <<  "   STATE: FOLLOW" << std::endl;
        twist_cmds = follow()
      }

      // Good Boy
      else if(cmd_state == 2){
        std::cout <<  "   STATE: GOODBOY" << std::endl;
      }

      // Stop (no command)
      else if(cmd_state == 0){
        std::cout <<  "   STATE: 0 (STOP)" << std::endl;
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
      }
    

      //Publish the twist command
      cmd_vel_pub_.publish(base_cmd);
    }
    return true;
  }

  int* follow(){
    int angle_err;
    int twist_cmds[2];          // [x, z]
    twist_cmds = {0, 0};

    // determine difference between current angle and target angle.
    angle_err = calc_angle_error()

    // determine turn speed (proportional)
    long turn_speed;
    turn_speed = (long)angle_err / 90.0; // Scales 0 - 90 to 0 - 1

    

    return twist_cmds
  }

  int calc_angle_error(){
    int angle_err;
    angle_err = 0;

    current_angle = calc_current_angle()
    angle_err = target_angle = current_angle;

    return err;
  }

  int calc_current_angle(){

  }


  int detect_command()
  {
    // To be completed by IAN
    return 0;
  }

  /* 
  analyze_prosody()
  Calls the python prosody analysis script using call_python_method().
  */ 
  int analyze_prosody()
  {
    int res;
    std::cout << "Running prosody script." << std::endl;

    // Configure input arguments for call_python_method()
    int nargs = 3;
    char* args[] = {"", "hello_world", "run"};
    res = call_python_method(nargs, args);
    return res;
  }

  int determine_src_dir()
  {
    int res;
    std::cout << "Running localization script." << std::endl;

    // Configure input arguments for call_python_method()
    int nargs = 3;
    char* args[] = {"", "cross_correlation", "run"};
    res = call_python_method(nargs, args);
    return res;
  }

  /* 
  call_python_module()
  Attempts to import a python module and call one of its functions.
  Returns -1 if it cannot find the module specified.
  Returns an int x otherwise.
  */ 
  int call_python_method(int argc, char *argv[])
  {
    std::cout << "  call_python_method" << std::endl;
    PyObject *sysPath, *programName, *pName, *pModule, *pDict, *pFunc, *pValue, *pArgs;
    int res;

    if (argc < 3) 
    {
      printf("    Usage: exe_name python_source function_name\n");
      return 1;
    }

    Py_SetProgramName(argv[0]);
    PySys_SetArgv(argc, argv);


    // Load the module object
    // pModule = PyImport_Import(pName);
    std::cout << "    Module: " << argv[1] << std::endl;
    pModule = PyImport_ImportModule(argv[1]);
    // std::cout << "    pModule:  " << pModule << std::endl;

    if(pModule == 0){
      std::cout << "    Could not find the python module." << std::endl;
      std::cout << "    Please run this node from the module's directory." << std::endl;
      // Clean up
      Py_DECREF(pFunc);
      Py_DECREF(pDict);
      Py_DECREF(pModule);

      return -1;
    }

    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, argv[2]);
    std::cout << "    Calling \"" << argv[2] << "()\":" << std::endl;
    if (PyCallable_Check(pFunc)) 
    {
      // Prepare the argument list for the call
      if( argc > 3 )
      {
        pArgs = PyTuple_New(argc - 3);
        for (int i = 0; i < argc - 3; i++)
        {
          pValue = PyLong_FromLong(atof(argv[i + 3]));
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
      } else
      {
        pValue = PyObject_CallObject(pFunc, NULL);
      }

      if (pValue != NULL) 
      {
        res = PyLong_AsLong(pValue);
        std::cout << "    pValue: " << pValue << std::endl;
        std::cout << "    Result: " << res << std::endl;
        Py_DECREF(pValue);
      }
      else 
      {
        PyErr_Print();
      }

    }

    // Clean up
    // Py_DECREF(pFunc);
    // Py_DECREF(pDict);
    Py_DECREF(pModule);
    // Finish the Python Interpreter
    // Py_Finalize();

    return res;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "audio_dog");
  ros::NodeHandle nh;

  // Initialize the Python Interpreter
  std::cout << "    Initializing Python Interpreter" << std::endl;
  Py_Initialize();

  RobotDriver driver(nh);
  driver.run();

  Py_Finalize();
}