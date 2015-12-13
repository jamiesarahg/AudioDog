// call_function.c - A sample of calling 
// python functions from C code
// 
#include <python2.7/Python.h> 
#include <iostream>

int main(int argc, char *argv[])
{
    PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pArgs;

    if (argc < 3) 
    {
        printf("Usage: exe_name python_source function_name\n");
        return 1;
    }

    // Initialize the Python Interpreter
    Py_SetProgramName(argv[0]);
    Py_Initialize();
    PySys_SetArgv(argc, argv);

    // Build the name object
    pName = PyString_FromString(argv[1]);

//     // Load the module object
//    pModule = PyImport_Import(pName);
    pModule = PyImport_ImportModule(argv[1]);
    std::cout << "name:  " << argv[1] << std::endl;
    std::cout << "pModule:  " << pModule << std::endl;

//     // pDict is a borrowed reference 
    pDict = PyModule_GetDict(pModule);
    std::cout << "pDict:  " << pDict << std::endl;

//     // pFunc is also a borrowed reference 
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
        } else
        {
            pValue = PyObject_CallObject(pFunc, NULL);
        }

        if (pValue != NULL) 
        {
            std::cout << "Return of call: " << PyInt_AsLong(pValue) << std::endl;
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

    return 0;
}