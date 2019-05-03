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
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

#ifdef USE_QTGUI
	innerModelViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, false);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
    osgView->setCameraManipulator(tb);

#endif

relateJointsMeshes();

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
        innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }


#ifdef USE_QTGUI
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), false);
#endif

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{

    printJointsFromAstra();
    //saveJointsFromAstra();
//    paintJointsFromFile();


#ifdef USE_QTGUI
	if (innerModelViewer) innerModelViewer->update();
	osgView->frame();
	osgView->autoResize();

#endif
}


void SpecificWorker::printJointsFromAstra()
{

    	try {
		PersonList users;
		humantracker_proxy->getUsersList(users);

		for (auto person : users)
		{
            if(!checkNecessaryJoints(person.second)) {return;}

            PaintSkeleton(person.second);
		}
	}

	catch(...) {}
}

void SpecificWorker::relateJointsMeshes()
{
    mapJointMesh["Head"] = "XN_SKEL_NECK";
    mapJointMesh["ShoulderSpine"] = "XN_SKEL_TORSO";

    mapJointMesh["LeftShoulder"] = "XN_SKEL_LEFT_SHOULDER";
    mapJointMesh["RightShoulder"] = "XN_SKEL_RIGHT_SHOULDER";

    mapJointMesh["LeftElbow"] = "XN_SKEL_LEFT_ELBOW";
    mapJointMesh["RightElbow"] = "XN_SKEL_RIGHT_ELBOW";

    mapJointMesh["LeftHip"] = "XN_SKEL_LEFT_HIP";
    mapJointMesh["RightHip"] = "XN_SKEL_RIGHT_HIP";

    mapJointMesh["LeftHand"] = "XN_SKEL_LEFT_HAND";
    mapJointMesh["RightHand"] = "XN_SKEL_RIGHT_HAND";

    mapJointMesh["LeftKnee"] = "XN_SKEL_LEFT_KNEE";
    mapJointMesh["RightKnee"] = "XN_SKEL_RIGHT_KNEE";

    mapJointMesh["LeftFoot"] = "XN_SKEL_LEFT_FOOT";
    mapJointMesh["RightFoot"] = "XN_SKEL_RIGHT_FOOT";


}

bool SpecificWorker::checkNecessaryJoints(TPerson &person)
{
    jointListType joints = person.joints;

//    for ( const auto &jointfound : mapJointMesh ) {
//        if(!joints.count(jointfound.first)){return false;}
//    }
////
//    return true;

    for(auto upT : upperTrunk)
    {
        if(!joints.count(upT))
        {
            upperTrunkFound = false;
            break;
        }
        else
        {upperTrunkFound = true;}

    }

    for(auto lwT : lowerTrunk)
    {
        if(!joints.count(lwT))
        {
            lowerTrunkFound = false;
            break;
        }
        else
        {lowerTrunkFound = true;}

    }

    if(upperTrunkFound or lowerTrunkFound)
        return true;
    else
        return false;

}

void SpecificWorker::paintJointsFromFile(){

    ifstream file;
    file.open("/home/robocomp/robocomp/components/human-detection/components/humanGame/joints.txt");

    if (!file) {
        cout << "Unable to open file";
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        TPerson person;
        jointListType all_joints;

        vector<string> parts = split(line,"#");

        for (auto p: parts)
        {
            vector<string> joints = split(p," ");

            if(joints.size()== 4)
            {
                joint poses;
                poses.push_back( QString::fromStdString(joints[1]).toFloat());
                poses.push_back(QString::fromStdString(joints[2]).toFloat());
                poses.push_back(QString::fromStdString(joints[3]).toFloat());

                all_joints[joints[0]] = poses;
            }

        }

        person.joints = all_joints;
        if(!checkNecessaryJoints(person))
        {
            qDebug()<<"faltan joints";
            continue;
        }

        PaintSkeleton(person);


    }

}


vector<string> SpecificWorker::split(const string& str, const string& delim)
{
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}


void SpecificWorker::saveJointsFromAstra()
{

    fstream jointfile;
    jointfile.open ( "joints.txt" , ios::app);

    try
    {
        PersonList users;
        humantracker_proxy-> getUsersList(users);

        if(users.size()== 0)
            return;

        for (auto u : users)
        {
            auto id = u.first;
            auto joints = u.second.joints;

            jointfile << id <<"#";

            for (auto j: joints)
            {
                jointfile <<j.first <<" "<<j.second[0] << " " << j.second[1] << " " <<j.second[2];
                jointfile << "#";
            }
        }

        jointfile <<endl;
    }

    catch(...) {}

    jointfile.close();


}


void SpecificWorker::PaintSkeleton (TPerson &person) {

    qDebug()<<__FUNCTION__;

    Pose3D pose;

    CalculateJointRotations(person);

    for (auto dictionaryNamesIt : mapJointMesh) {

        try {
            string idJoint = dictionaryNamesIt.first;
            QString TypeJoint = dictionaryNamesIt.second;

            SetPoses (pose, idJoint);

            innerModel->updateTransformValues(TypeJoint,pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz);

        }
        catch (...) {
            qDebug()<<"Error in PaintSkeleton";
        }

    }

    innerModel->update();

    innerModelViewer->update();
    osgView->frame();
    osgView->autoResize();

}





void SpecificWorker::CalculateJointRotations (TPerson &person) {

    RTMat kinect;

    auto jointList = person.joints; //map that relates the name of the joint with a sequence of its points

    // apunta el torso (inclinación alante/atrás y lateral del torso)

    if (upperTrunkFound)
    {
        qDebug()<<"%%%%%%%%%%%%%%%%%%%%%%%%%% UPPER TRUNK %%%%%%%%%%%%%%%%%%%%%%%%%%";
        qDebug()<<"----------SPINE------------";

        mapJointRotations["Spine"] = RTMat();
        mapJointRotations["Spine"].setTr(jointList["ShoulderSpine"][0],jointList["ShoulderSpine"][1],jointList["ShoulderSpine"][2]);

        qDebug()<<"----------CABEZA----------";
        mapJointRotations["Head"] = RTMatFromJointPosition (mapJointRotations["Spine"], jointList["ShoulderSpine"], jointList["Head"], jointList["ShoulderSpine"], 2);

//        /// alineación de hombros (rotación en Z del torso), previa al cálculo de la transformacion final de los hombros.
//        RTMat LEFT_SHOULDER_PRE_Z = RTMatFromJointPosition (mapJointRotations["Spine"],  jointList["ShoulderSpine"],jointList["LeftShoulder"],jointList["LeftShoulder"], 2);
//        RTMat RIGHT_SHOULDER_PRE_Z = RTMatFromJointPosition (mapJointRotations["Spine"],  jointList["ShoulderSpine"],jointList["RightShoulder"], jointList["RightShoulder"], 2);
//
//        RotateTorso (RIGHT_SHOULDER_PRE_Z.getTr(), LEFT_SHOULDER_PRE_Z.getTr());

        //brazo izquierdo
        qDebug()<<"---------- HOMBRO IZQUIERDO ----------";
        mapJointRotations["ShoulderLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"], jointList["ShoulderSpine"], jointList["LeftShoulder"], jointList["LeftShoulder"], 2);
        qDebug()<<"---------- HOMBRO DERECHO ----------";
        mapJointRotations["ShoulderRight"] = RTMatFromJointPosition (mapJointRotations["Spine"],jointList["ShoulderSpine"], jointList["RightShoulder"], jointList["RightShoulder"], 2);

        qDebug()<<"---------- CODO IZQUIERDO ----------";
        mapJointRotations["ElbowLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["ShoulderLeft"],  jointList["LeftShoulder"],jointList["LeftElbow"], jointList["LeftElbow"], 2);
        qDebug()<<"---------- CODO DERECHO ----------";
        mapJointRotations["ElbowRight"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["ShoulderRight"],  jointList["RightShoulder"], jointList["RightElbow"], jointList["RightElbow"], 2);

        //Manos derecha e izquierda
        qDebug()<<"---------- MANO IZQUIERDA ----------";
        mapJointRotations["HandLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["ShoulderLeft"]*mapJointRotations["ElbowLeft"],jointList["LeftElbow"],jointList["LeftHand"], jointList["LeftHand"], 2);
        qDebug()<<"---------- MANO DERECHA ----------";
        mapJointRotations["HandRight"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["ShoulderRight"]*mapJointRotations["ElbowRight"], jointList["RightElbow"], jointList["RightHand"], jointList["RightHand"], 2);
    }

    if (lowerTrunkFound)
    {
        qDebug()<<"%%%%%%%%%%%%%%%%%%%%%%%%%% LOWER TRUNK %%%%%%%%%%%%%%%%%%%%%%%%%%";

        qDebug()<<"---------- CADERA IZQUIERDA ----------";
        mapJointRotations["HipLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"], jointList["ShoulderSpine"],jointList["LeftHip"], jointList["LeftHip"], 2);
        qDebug()<<"---------- CADERA DERECHA ----------";
        mapJointRotations["HipRight"] = RTMatFromJointPosition (mapJointRotations["Spine"], jointList["ShoulderSpine"], jointList["RightHip"], jointList["RightHip"], 2);


        qDebug()<<"---------- RODILLA IZQUIERDA ----------";
        mapJointRotations["KneeLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["HipLeft"], jointList["LeftHip"],jointList["LeftKnee"],  jointList["LeftKnee"], 2);
        qDebug()<<"---------- RODILLA DERECHA ----------";
        mapJointRotations["KneeRight"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["HipRight"], jointList["RightHip"],jointList["RightKnee"], jointList["RightKnee"], 2);

        qDebug()<<"---------- PIE IZQUIERDO ----------";
        mapJointRotations["FootLeft"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["HipLeft"]*mapJointRotations["KneeLeft"], jointList["LeftKnee"],jointList["LeftFoot"],  jointList["LeftFoot"], 2);
        qDebug()<<"---------- PIE DERECHO ----------";
        mapJointRotations["FootRight"] = RTMatFromJointPosition (mapJointRotations["Spine"]*mapJointRotations["HipRight"]*mapJointRotations["KneeRight"], jointList["RightKnee"],jointList["RightFoot"], jointList["RightFoot"], 2);


    }

    upperTrunkFound = false;
    lowerTrunkFound = false;



qDebug()<<"  ";
qDebug()<<"  ";
qDebug()<<"  ";
qDebug()<<"  ";
qDebug()<<"  ";
qDebug()<<"  ";
}


RTMat SpecificWorker::RTMatFromJointPosition (RTMat rS,jointPos p1, jointPos p2, jointPos translation, int axis) {

    qDebug()<<__FUNCTION__;

    qDebug()<<"p1 " << p1[0] <<" "<< p1[1]<<" "<< p1[2];
    qDebug()<<"p2 " << p2[0] <<" "<< p2[1]<<" "<< p2[2];



    bool XClockWise=false, YClockWise=false, ZClockWise=true;
    float alpha, beta, gamma;

    RTMat rt(XClockWise,YClockWise, ZClockWise);
    QVec p1h = QVec::vec4(p1[0], p1[1], p1[2], 1);
    QVec p2h = QVec::vec4(p2[0], p2[1], p2[2],1);
    QVec translationH = QVec::vec4(translation[0], translation[1], translation[2],1);

    QMat aux = rS;
    aux = aux.invert();
    QVec translationT = aux * translationH;
    QVec p1t = aux * p1h;
    QVec p2t = aux * p2h;
    QVec vT = p2t - p1t;
    QVec v= vT.normalize();

    ///por filas
    switch(axis){
        case 0:
            alpha = 0;

            if(YClockWise) beta = atan2(-v.z(),v.x());
            else beta = atan2(v.z(),v.x());

            if(ZClockWise) gamma = asin(-v.y());
            else gamma = asin(v.y());

            break;
        case 1:
            if(XClockWise) alpha = atan2(v.z(),v.y());
            else alpha = atan2(-v.z(),v.y());

            beta = 0;

            if(ZClockWise) gamma = asin(v.x());
            else gamma = asin(-v.x());

            break;
        case 2:
            if(XClockWise) alpha =  atan2(-v.z(),v.y());
            else alpha =  atan2(v.z(),v.y());

            if(YClockWise) beta = atan2(-v.z(),v.x());
            else beta = atan2(v.z(),v.x());

            gamma = 0;

            break;
    }
    qDebug()<<alpha<<beta<<gamma;
    rt.setRT(alpha,beta,gamma,vT);

    rt.print("Matriz calculada");


    return rt;
}


bool SpecificWorker::RotateTorso (const QVec &lshoulder, const QVec &rshoulder) {

    qDebug()<<__FUNCTION__;

    QVec eje = lshoulder - rshoulder;	//Calculamos el eje que va de un hombro a otro

    eje.normalize();

    if(eje.x()==0) {
        return false;
    }

    float angulo = atan2(eje.y(),eje.x());	//Calculamos el giro necesario para alinear los hombros con el eje (arcotangente de y/x)

    mapJointRotations["Spine"].setRZ(angulo); // Aplicamos dicho giro al eje Z del torso

    return true;
}

void SpecificWorker::SetPoses (Pose3D &pose, string joint) {

    int height=0;
    int head=0;

    if (joint=="ShoulderSpine") {
        height=1500;
    }

    if (joint=="Head") {
        head=140;
    }

    pose.x = 1000*mapJointRotations[joint].getTr().x();
    pose.y = 1000*mapJointRotations[joint].getTr().y()+height;
    pose.z = 1000*mapJointRotations[joint].getTr().z()-(2*height)+head;

    pose.rx = mapJointRotations[joint].getRxValue();
    pose.ry = mapJointRotations[joint].getRyValue();
    pose.rz = mapJointRotations[joint].getRzValue();



}
