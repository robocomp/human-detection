
#ifndef PYTHONCALL_H
#define PYTHONCALL_H

//avoid python slot keyword redefinition
#pragma push_macro("slots")
#undef slots
#include "Python.h"
#pragma pop_macro("slots")

#include "specificworker.h"


class PythonCall
{
    PyObject *pModule;
	PyObject *pFunc;
	PyObject *pName;

public:
    PythonCall();
    ~PythonCall();
    
    void initialize();
	void finalize();
    void callPythonGNN(SpecificWorker::ModelPerson *p_old);
    float degreesToRadians(const float angle_);
};

#endif