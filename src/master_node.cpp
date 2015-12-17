/*
Computational Robotics 2015
Project AudiodDog

Authors:
  Antonia Elsen
  Jamie Gorson
  Susie Grimshaw
  Ian Hill

master_node.cpp
  C++ script containing ROS Node
  Calls python scripts to detect spoken commands, analyze the prosody of the 
  oral command, and determine the direction of the source audio signal.
*/

#include <python2.7/Python.h> 
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <signal.h>
#include <sndfile.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <neato_node/Bump.h>
#include "circular_buffer.h"

// Keyboard Interrupt Handler
volatile sig_atomic_t flag = 0;
void interrupt(int sig){ // can be called asynchronously
  flag = 1; // set flag
}



// Main Class - ROS Node
class RobotDriver
{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber scanSub;

  // State variables
  std::string cmd_file;       // Filename of audio file to process
  float twist_cmds[2];        // movement commands {forward, angular}
  bool awaiting_cmd;          // true: command found, acting; false: listening
  int cmd_state;              // 0: still, 1: follow, 2: "stop", 
                              // 3: good boy, 4: fetch
 
  // Time variables
  long CLOCKS_PER_MS;
  unsigned long loop_start_time, loop_current_time, loop_dt, cmd_start_time;

  // Direction variables
  float current_angle, start_angle, target_angle;
  PyObject *pModels;

  

public:
  // ROS Node Initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cmd_file = "../wav/sample.wav";
  }

  /*
  run()
  Main method.

  Creates prosody analysis model dictionary, 
  Runs main loop.

  Loop contains:
   - audio detection script, 
   - prosody analysis script, 
   - source direction calculation script.
  */
  bool run()
  { 
    // State variables
    cmd_state = 0;
    awaiting_cmd = true;
    geometry_msgs::Twist base_cmd;

    // Time variables
    CLOCKS_PER_MS = CLOCKS_PER_SEC/1000.0;
    loop_start_time = clock();

    // Misc variables
    int result_detect, result_pros, result_dir;
    signal(SIGINT, interrupt); 
    // --------------------------------------------------------------
    
    // std::cout << "Creating models for prosody analysis." << std::endl;
    // create_models();

    std::cout << std::endl << "Woof! I'm awake.\nUse keyboard interrupt ";
    std::cout << "when you want me to stop." << std::endl << std::endl;


    while(nh_.ok()){
      loop_current_time = clock();
      loop_dt = (loop_current_time - loop_start_time) / CLOCKS_PER_MS; 
      
      // Only run loop once every 10 ms or slower.
      if (loop_dt < 10){
        continue;
      }

      loop_start_time = loop_current_time;
      

      // Default twist commands: set stationary
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      // Catch keyboard interrupts to still node.
      if(flag)
      {
        std::cout << "Caught! Keyboard interrupt.";
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        return false;
      }

      
      // Call function that searches for audio signal, saves to wav if found
        // updates "sample.wav"
      if (awaiting_cmd){
        std::cout << "AWAITING CMD" << std::endl;
        result_detect = detect_command();
        // result_detect = 1;

        // If an audio signal has been found, process it
        if (result_detect != -1){
          // std::cout << "    Prosody-based command detected." << std::endl;

          // Once audio signal has been found and saved to file, 
          // load the file, run prosody script, determine command
          result_pros = analyze_prosody(); // UPDATE TO RUN PROSODY

          // If the prosody analysis did not fail:
          if (result_pros != -1){

            // Set the state to the command was found
            cmd_state = result_pros;
            awaiting_cmd = false;
            cmd_start_time = clock();
            std::cout << "STATE: " << cmd_state << std::endl;

            // If the command was to "follow", find the angle relative to src
            if(cmd_state == 1){
              result_dir = determine_src_dir();
              target_angle = result_dir / 1000.0;
            }
          }
        }
      }
      else{
        // std::cout << "STATE:" << cmd_state << std::endl;
        //Follow
        if(cmd_state == 1){
          // std::cout <<  "   STATE: FOLLOW" << std::endl;
          // std::cout <<  "   TARGET: " <<  target_angle << std::endl;
          follow(cmd_start_time);
        }

        // Stop
        else if(cmd_state == 2){
          // std::cout <<  "   STATE: STOP" << std::endl;
          stop(cmd_start_time);
        }

        // Good Boy
        else if(cmd_state == 3){
          // std::cout <<  "   STATE: GOODBOY" << std::endl;
          good_boy(cmd_start_time);
        }

        // Fetch
        else if(cmd_state == 4){
          // std::cout <<  "   STATE: FETCH" << std::endl;
          fetch(cmd_start_time);
        }

        // Still (no command)
        else if(cmd_state == 0){
          // std::cout <<  "   STATE: 0 (STILL)" << std::endl;
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

  void follow(int cmd_start_time){
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

  void good_boy(int cmd_start_time){
    // Function to run during good_boy state

    float dt, mod; // difference of time from start selection of good boy to now

    dt = (clock() - cmd_start_time)/CLOCKS_PER_MS; //calculating diff_time

    // check to see if in good_boy for more than four seconds. If so, exit loop
    if (dt > 4000) {
      std::cout << "    Now awaiting cmd" << std::endl;
      //CHANGE STATUS
      twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
      twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
      cmd_state = 0;
      awaiting_cmd = true;
    }
    else {
      
      mod = (int)dt % 1000;
      // std::cout << "    dt: " << dt;
      // std::cout << "    mod: " << mod;
      // good boy switches direction every .5 seconds. 
      // Check to see if it should be going left or right.
      if (mod > 500) {
            // std::cout << "    NEG" << std::endl;
            twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
            twist_cmds[1] = -0.8;          // the z (angular) speed (0 - 1)
      }
      else {
            // std::cout << "    POS" << std::endl;
            twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
            twist_cmds[1] = 0.8;          // the z (angular) speed (0 - 1)
      }
    }
  }

  void fetch(int cmd_start_time){
    // Function to run during fetch state

    float dt;                       // elapsed time
    dt = (clock() - cmd_start_time)/CLOCKS_PER_MS; //calculating diff_time

    // check to see if in fetch for more than four seconds. If so, exit loop
    if (dt > 4000) {
      std::cout << "    Now awaiting cmd" << std::endl;
      //CHANGE STATUS
      twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
      twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
      cmd_state = 0;
      awaiting_cmd = true;
    }
    else {
      // fetch turns direction for 4 seconds. Check to see if it should be going left or right.
      twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
      twist_cmds[1] = 0.8;          // the z (angular) speed (0 - 1)
    }
  }

  void stop(int cmd_start_times){
    // Function to run during stop state

    float dt; // difference of time from start selection of stop to now
    float isOdd;

    dt = (clock() - cmd_start_time)/CLOCKS_PER_MS; //calculating diff_time

    // check to see if in stop for more than four seconds. If so, exit loop
    if (dt > 4000) {
      std::cout << "    Now awaiting cmd" << std::endl;
      //CHANGE STATUS
      twist_cmds[0] = 0.0;          // the x (forward) speed (between 0 - 1)
      twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
      cmd_state = 0;
      awaiting_cmd = true;
    }

    else {
      isOdd = (int)floor(dt/1000) % 2;
      if (!isOdd){
        twist_cmds[0] = 0.4;          // the x (forward) speed (between 0 - 1)
        twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
      } 
      else {
        twist_cmds[0] = -0.4;          // the x (forward) speed (between 0 - 1)
        twist_cmds[1] = 0.0;          // the z (angular) speed (0 - 1)
      }
    }
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
    SNDFILE *sf;
    SF_INFO info;
    int num_channels;
    int num, num_items;
    num = 1;
    int f,sr,c;
    int i,j;
    SNDFILE *out;
    
    /* Open stdin to capture WAV data. */
    info.format = SF_FORMAT_WAV;
    sf = sf_open_fd(0, SFM_READ, &info, true);
    if (sf == NULL)
      {
      std::cout << "Failed to read stdin." << std::endl;
      exit(-1);
      }
    else{
      std::cout << sf << std::endl;
      }
    /* Print some of the info, and figure out how much data to read. */
    sf_count_t frames = 176400*2;
    int item_goal = 176400*4;
    int item_count = 0;
    sr = info.samplerate;
    info.frames = frames;
    info.seekable = 1;
    num_items = (int) round(sr / 100);
    float* incoming_section = new float[num_items];
    float* outgoing_section = new float[num_items];
    int out_index = 0;
    FloatCircularBuffer buffer (352800, 88200, 0.18, 0.1);
    bool record = false;
    /*
    Load raw data into the circular buffer and write the data to temp.out
    when appropriate.
    */

    out = sf_open("../wav/sample.wav",SFM_WRITE, &info);
    std::cout << "Saved file to '../wav/sample.wav'" << std::endl;
    while (((num = sf_read_float (sf, incoming_section, num_items)) > 0) &&
        (item_count < item_goal)) {
      for (int in_index = 0; in_index < num; in_index++) {
        record = (buffer.push(incoming_section[in_index])) ? true : record;
        if (record) {
          buffer.pop(outgoing_section[out_index]);
          out_index++;
          item_count++;
        }
        if (out_index == num_items) {
          sf_write_float(out, outgoing_section, out_index);
          out_index = 0;
        }
        if (item_count == item_goal) {
          sf_write_float(out, outgoing_section, out_index);
          break;
        }
      }
    }
    sf_close(sf);
    sf_close(out);
    delete [] incoming_section;
    delete [] outgoing_section;
    printf("done\n");
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
    char* args[] = {"", "predict", "predict_wrapper"};
    res = call_python_method(nargs, args);
    return res;
  }

  int analyze_prosody_with_models()
  {
    std::cout << "    analyze_prosody_with_models()" << std::endl;
    int nargs = 3;
    char* args[] = {"", "predict", "predict_wrapper"};
    std::cout << "  call_python_method" << std::endl;
    PyObject *sysPath, *programName, *pName, *pModule, *pDict, *pFunc, *pValue, *pArgs;
    int res;

    PySys_SetArgv(nargs, args);

    // Load the module object
    // pModule = PyImport_Import(pName);
    std::cout << "    Module: " << args[1] << std::endl;
    pModule = PyImport_ImportModule(args[1]);
    std::cout << "    pModule:  " << pModule << std::endl;

    if(pModule == 0){
      std::cout << "    Could not find the python module." << std::endl;
      std::cout << "    Please run this node from the module's directory." << std::endl;
      // Clean up
      Py_DECREF(pModule);

      return -1;
    }

    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, args[2]);
    std::cout << "    Calling \"" << args[2] << "()\":" << std::endl;
    if (PyCallable_Check(pFunc)) 
    {
        pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 1, pModels);    

      // Prepare the argument list for the call
      pValue = PyObject_CallObject(pFunc, pArgs);
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
    return res;
  }

  // int create_models(){
  //   std::cout << "    create_models()" << std::endl;
  //   int nargs = 3;
  //   char* args[] = {"", "createModels", "createModels"};
  //   std::cout << "  call_python_method" << std::endl;
  //   PyObject *sysPath, *programName, *pName, *pModule, *pDict, *pFunc, *pValue, *pArgs;
  //   int res;

  //   PySys_SetArgv(nargs, args);

  //   // Load the module object
  //   // pModule = PyImport_Import(pName);
  //   std::cout << "    Module: " << args[1] << std::endl;
  //   pModule = PyImport_ImportModule(args[1]);
  //   std::cout << "    pModule:  " << pModule << std::endl;

  //   if(pModule == 0){
  //     std::cout << "    Could not find the python module." << std::endl;
  //     std::cout << "    Please run this node from the module's directory." << std::endl;
  //     // Clean up
  //     Py_DECREF(pModule);

  //     return -1;
  //   }

  //   pDict = PyModule_GetDict(pModule);
  //   pFunc = PyDict_GetItemString(pDict, args[2]);
  //   std::cout << "    Calling \"" << args[2] << "()\":" << std::endl;
  //   if (PyCallable_Check(pFunc)) 
  //   {
  //     // Prepare the argument list for the call
  //     pModels = PyObject_CallObject(pFunc, NULL);
  //   }

  //   // Clean up
  //   // Py_DECREF(pFunc);
  //   // Py_DECREF(pDict);
  //   Py_DECREF(pModule);
  //   return 1;
  // }

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
    std::cout << "Direction:" << std::endl;
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
    // std::cout << "  call_python_method" << std::endl;
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
    // std::cout << "    Module: " << argv[1] << std::endl;
    pModule = PyImport_ImportModule(argv[1]);
    // std::cout << "    pModule:  " << pModule << std::endl;

    if(pModule == 0){
      std::cout << "    Could not find the python module." << std::endl;
      std::cout << "    Please run this node from the module's directory." << std::endl;
      // Clean up
      Py_DECREF(pModule);

      return -1;
    }

    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, argv[2]);
    // std::cout << "    Calling \"" << argv[2] << "()\":" << std::endl;
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
        // std::cout << "    pValue: " << pValue << std::endl;
        // std::cout << "    Result: " << res << std::endl;
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
  Py_Initialize();

  RobotDriver driver(nh);
  driver.run();

  Py_Finalize();
}