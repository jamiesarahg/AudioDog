#include <python2.7/Python.h> 
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <neato_node/Bump.h>


//http://stackoverflow.com/questions/7998816/do-pyimport-importmodule-and-import-statement-load-into-different-namespace
//http://www.tutorialspoint.com/python/python_further_extensions.htm
//http://members.gamedev.net/sicrane/articles/EmbeddingPythonPart1.html
//https://docs.python.org/2/extending/embedding.html
//https://books.google.com/books?id=n11lCgAAQBAJ&pg=PA140&lpg=PA140&dq=ros+laserscan+ranges&source=bl&ots=zWvUw5jUCO&sig=DNNW87ol3By0hyFbKaxTF89wYQY&hl=en&sa=X&ved=0ahUKEwjajsL_5N_JAhVHOiYKHcMhCWAQ6AEIWzAJ#v=onepage&q=ros%20laserscan%20ranges&f=false
//http://ros-users.122217.n3.nabble.com/SICK-LMS-Subscriber-Node-td970412.html

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
  ros::Subscriber scanSub;

  bool awaiting_cmd;             // true: command found, acting; false: listening
  float twist_cmds[2];          // movement commands {forward, angular}
  int cmd_state;              // 0: stop, 1: follow, 2: good boy
  float current_angle, start_angle, target_angle;

  unsigned long loop_start_time, loop_current_time, loop_dt;

  std::string cmd_file;

public:
  // ROS Node Initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cmd_file = "../wav/jamieClose1.wav";  // to be renamed
  }

  // Main loop
  bool run()
  {
    int result_detect, result_pros, result_dir;
    long loop_current_time;
    signal(SIGINT, interrupt); 
    geometry_msgs::Twist base_cmd;
    awaiting_cmd = true;
    cmd_state = 0;

    // Time tracking variables
    long CLOCKS_PER_MS = CLOCKS_PER_SEC/1000.0;
    loop_start_time = clock();

    std::cout << std::endl << "Woof! I'm awake.\nUse keyboard interrupt ";
    std::cout << "when you want me to stop." << std::endl << std::endl;
    
    // create model dictionary # TODO
        // create_models.py
        // predict.py -> dictionary  created from create_models
          // pass in wave filename


    while(nh_.ok()){
      loop_current_time = clock();
      loop_dt = (loop_current_time - loop_start_time) / CLOCKS_PER_MS; 
      
      // Only run loop once every 10 ms or slower.
      if (loop_dt < 10){
        continue;
      }

      loop_start_time = loop_current_time;
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
        // updates "test.wav"
      if (awaiting_cmd){
        std::cout << "AWAITING CMD" << std::endl;
        result_detect = detect_command();

        // If an audio signal has been found, process it
        if (result_detect != -1){

          // Once audio signal has been found and saved to file, 
          // load the file, run prosody script, determine command
          result_pros = analyze_prosody(); // UPDATE TO RUN PROSODY

          // If the prosody analysis did not fail:
          if (result_pros != -1){

            // Set the state to the command was found
            cmd_state = result_pros;
            awaiting_cmd = false;

            // If the command was to "follow", find the angle relative to src
            if(cmd_state == 1){
              result_dir = determine_src_dir(); // TODO  - DOWNSAMPLE
              target_angle = result_dir / 1000.0;
            }
          }
        }
      }
      else{
        // std::cout << "STATE:" << cmd_state << std::endl;
        //Follow
        if(cmd_state == 1){
          std::cout <<  "   STATE: FOLLOW" << std::endl;
          std::cout <<  "   TARGET: " <<  target_angle << std::endl;
          follow();
        }

        // Good Boy
        else if(cmd_state == 2){
          // std::cout <<  "   STATE: GOODBOY" << std::endl;
          good_boy(); // TODO : to be completed by prosody team
          
        }

        // Stop (no command)
        else if(cmd_state == 0){
          // std::cout <<  "   STATE: 0 (STOP)" << std::endl;
          twist_cmds[0] = 0.0;
          twist_cmds[1] = 0.0;
          awaiting_cmd = true;
        }
      }


      base_cmd.linear.x = twist_cmds[0];
      base_cmd.angular.z = twist_cmds[1];

      //Publish the twist command
      cmd_vel_pub_.publish(base_cmd);
    }


    return true;
  }

  void follow(){
    float angle_err;
    float new_twist_cmds[2];

    // determine difference between current angle and target angle.
    angle_err = calc_angle_error();

    // determine turn speed (proportional)
    long turn_speed;
    turn_speed = (long)angle_err / 90.0; // Scales 0 - 90 to 0 - 1
    

    // std::copy(new_twist_cmds, new_twist_cmds+2, twist_cmds);
    twist_cmds[0] = 0.0;
    twist_cmds[1] = turn_speed;
  }

  void good_boy(){
    // TODO : to be completed by prosody team
    // run this function on every iteration of loop
    // for example: if "Goodboy" makes the neato turn for three seconds
    // check the time elapsed, if there is still time, keep turning
    // if time is up, stop turning

    //if the command is finished, let the neato listen for another command
    /*
    if (... enter finished conditional here){
      cmd_state = 0;
      awaiting_cmd = true;
    }
    */


    twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
    twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
  }

  int calc_angle_error(){
    int angle_err;
    angle_err = 0;

    current_angle = calc_current_angle();
    angle_err = target_angle - current_angle;

    return angle_err;
  }

  int calc_current_angle(){

  }


  int detect_command()
  {
    // To be completed by Ian
    return 0;
  }

  /* 
  analyze_prosody()
  Calls the python prosody analysis script using call_python_method().
  */ 
  int analyze_prosody()
  {
    int res;
    std::cout << "    Running prosody script." << std::endl;

    // Configure input arguments for call_python_method()
    int nargs = 3;
    char* args[] = {"", "hello_world", "run"};
    res = call_python_method(nargs, args);
    return res;
  }


  /* 
  determine_src_dir()
  Calls the python angle calculation script using call_python_method().
  */ 
  int determine_src_dir()
  {
    int res;
    std::cout << "    Running angle calculation script." << std::endl;

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