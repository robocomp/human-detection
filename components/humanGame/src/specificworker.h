/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *m
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define PI 3.14159

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qmat/qrtmat.h>
#include <fstream>

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

class SpecificWorker : public GenericWorker
{
Q_OBJECT

map<string,QString> mapJointMesh; //Mapa que relaciona el nombre de las partes con los meshs
map<string,RTMat> mapJointRotations; //Mapa que guarda las rotaciones calculadas

using jointPos = std::vector<float> ;

bool upperTrunkFound = false;
bool lowerTrunkFound = false;

vector<string> upperTrunk = {"MidSpine","Head", "Neck", "LeftShoulder", "RightShoulder","LeftElbow","RightElbow" , "LeftHand", "RightHand" };
vector<string> lowerTrunk = {"MidSpine", "BaseSpine" ,"LeftHip","RightHip","LeftKnee","RightKnee","LeftFoot","RightFoot" };


struct Pose3D
{
	float x;
	float y;
	float z;
	float rx;
	float ry;
	float rz;
};

public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	void initialize(int period);

    void relateJointsMeshes();
	void PaintSkeleton (RoboCompHumanTracker::TPerson &person);
	void CalculateJointRotations (RoboCompHumanTracker::TPerson &person);
	RTMat RTMatFromJointPosition (RTMat rS, jointPos p1, jointPos p2, jointPos translation, int axis); //This method calculates the rotation of a Joint given some points
	bool RotateTorso (const QVec &lshoulder, const QVec &rshoulder); //This method allows to rotate the torso from the position and rotation of the shoulders
	void SetPoses (Pose3D &pose, string joint);
    bool checkNecessaryJoints(TPerson &person);
    void paintJointsFromFile();
    vector<string>split(const string& str, const string& delim);
    void saveJointsFromAstra();
    void printJointsFromAstra();

private:
    std::shared_ptr<InnerModel> innerModel;
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif

};

#endif
