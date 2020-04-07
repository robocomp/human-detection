#include "pythonCall.h"


PythonCall::PythonCall()
{}

PythonCall::~PythonCall()
{}

void PythonCall::initialize()
{
    Py_Initialize();

    pName = PyUnicode_FromString("gnnFromCpp");
    PyErr_Print();

    pModule = PyImport_Import(pName);
    PyErr_Print();

    
    if (pModule != 0)
    {
        pFunc = PyObject_GetAttrString(pModule, "main");
        PyErr_Print();
        qDebug()<<"error retrieving python module";
    }
}

void PythonCall::finalize()
{

	Py_DECREF(PyImport_ImportModule("threading"));
	Py_Finalize();

/*	Py_DECREF(pFunc);
	Py_DECREF(pValue);
	Py_DECREF(x_value);
	Py_DECREF(z_value);
	Py_DECREF(angle_value);*/	

}

void PythonCall::callPythonGNN(SpecificWorker::ModelPerson *p_old)
{

	try
	{
		float x = 0;
		float z = 0;
		float angle = 0;
			

		PyObject *pValue = PyObject_CallObject(pFunc, NULL);

		PyObject *x_value = PyList_GetItem(pValue, 0);
		x = PyFloat_AsDouble(x_value);
		
		PyObject *z_value = PyList_GetItem(pValue, 1);
		z = PyFloat_AsDouble(z_value);
		
		PyObject *angle_value = PyList_GetItem(pValue, 2);
		angle = PyFloat_AsDouble(angle_value);
		
		std::cout<<"Result (x,z,angle): ("<<x<<", "<<z<<", "<<angle<<")"<<std::endl;
		//update model
		if(p_old->human != NULL)
			p_old->human->updateGNN(x, z, degreesToRadians(angle));
	
	}catch(...){
		PyErr_Print();
	}
}

float PythonCall::degreesToRadians(const float angle_)
{	
	if (angle_ == 99999)
		return 99999;
	float angle = angle_ * 2*M_PI / 360;
	if(angle > M_PI)
   		return angle - M_PI*2;
	else if(angle < -M_PI)
   		return angle + M_PI*2;
	else return angle;
}
