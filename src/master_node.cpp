#include <python2.7/Python.h> 
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <sndfile.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <neato_node/Bump.h>
#include "circular_buffer.h"


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
    int res_listen, res_pros, res_loc;
    signal(SIGINT, interrupt); 
    std::cout << std::endl << "Woof! I'm awake.\nUse keyboard interrupt ";
    std::cout << "when you want me to stop." << std::endl << std::endl;
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while(nh_.ok()){

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
      res_listen = run_audio_processor();

      // Once audio signal has been found and saved to file, 
      // load the file, run prosody script
      res_pros = run_prosody_analysis();

      // If the prosody analysis failed due to cwd issues, exit.
      if (res_pros == -1){
        return false;
      }
      // Depending on the prosody result, execute a movement command.
      else if(res_pros == 1){
        std::cout <<  "Following!" << std::endl;
        // execute cross_correlation script here
      }
      else if(res_pros == 2){
        std::cout <<  "Stopping!" << std::endl;
      }
      else if(res_pros == 3){
        std::cout <<  "etc." << std::endl;
      }
      

      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;

      // Old keyboard-controlled movement code
      // std::cin.getline(cmd, 50);
      // cmd[0] = 'x';
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

  int run_audio_processor()
  {
    SNDFILE *sf;
    SF_INFO info;
    int num_channels;
    int num, num_items;
    int f,sr,c;
    int i,j;
    SNDFILE *out;
    
    /* Open stdin to capture WAV data. */
    info.format = SF_FORMAT_WAV;
    sf = sf_open_fd(0, SFM_READ, &info, true);
    if (sf == NULL)
      {
      printf("Failed to read stdin.\n");
      exit(-1);
      }
    /* Print some of the info, and figure out how much data to read. */
    f = info.frames;
    sr = info.samplerate;
    c = info.channels;
    num_items = (int) round(sr / 100);
    float* incoming_section = new float[num_items*sizeof(float)];
    float* outgoing_section = new float[num_items*sizeof(float)];
    int out_index = 0;
    FloatCircularBuffer buffer (352800, 88200, 0.18, 0.1);
    bool record = false;
    bool previousRecord = false;
    /*
    Load raw data into the circular buffer and write the data to temp.out
    when appropriate.
    */
    out = sf_open("temp.out",SFM_WRITE, &info);
    while ((num = sf_read_float (sf, incoming_section, num_items)) > 0) {
      for (int in_index = 0; in_index < num; in_index++) {
        record = buffer.push(incoming_section[in_index]);
        if (record) {
          //printf("%i\n", out_index);
          buffer.pop(outgoing_section[out_index]);
          out_index = (out_index == (num_items-1)) ? 0 : out_index + 1;
          if (out_index == num_items) {
            printf("writing and recording\n");
            sf_write_float (out, outgoing_section, out_index + 1);
            printf("successful write\n");
          }
        } else if (!record && previousRecord) {
          sf_write_float (out, outgoing_section, out_index + 1);
          sf_close(sf);
          sf_close(out);
          return 0;
        }
        previousRecord = record;
      }
      //buffer.print_average();
    }
    delete [] incoming_section;
    delete [] outgoing_section;
    printf("done\n");
    return 0;
  }

  /* 
  run_prosody_analysis()
  Calls the python prosody analysis script using call_python_method().
  */ 
  int run_prosody_analysis()
  {
    int res;
    std::cout << "Running prosody script." << std::endl;

    // Configure input arguments for call_python_method()
    int nargs = 3;
    char* args[] = {"", "hello_world", "run"};
    res = call_python_method(nargs, args);
    return res;
  }

  int run_localization()
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

      // Initialize the Python Interpreter
    Py_SetProgramName(argv[0]);
    Py_Initialize();
    PySys_SetArgv(argc, argv);

      // Build the name object ... which somehow helps this script locate the module
      // pName = PyString_FromString("~/catkin_ws/src/comprobo15/AudioDog/src/hello_world");

      // Load the module object
      // pModule = PyImport_Import(pName);
    pModule = PyImport_ImportModule(argv[1]);
    std::cout << "    pModule:  " << pModule << std::endl;
    if(pModule == 0){
      std::cout << "Could not find the python module." << std::endl;
      std::cout << "Please run this node from the module's directory." << std::endl;
      return -1;
    }

    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, argv[2]);
    std::cout << "    Calling \"" << argv[2] << "()\":" << std::endl;
    std::cout <<std::endl;

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
        res = PyInt_AsLong(pValue);
        // std::cout << std::endl << "    Result: " << res << std::endl;
        Py_DECREF(pValue);
      }
      else 
      {
        PyErr_Print();
      }

    }

      // Clean up
    Py_DECREF(pModule);
      // Py_DECREF(pName);

      // Finish the Python Interpreter
    Py_Finalize();

    return res;
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