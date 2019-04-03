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
    connect(publish_pb, SIGNAL(pressed()), this, SLOT(publish_person()));
    connect(save_pb, SIGNAL(pressed()), this, SLOT(save_file()));
    connect(load_pb, SIGNAL(pressed()), this, SLOT(load_file()));
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
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }

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
	//computeCODE
//	QMutexLocker locker(mutex); 
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

void SpecificWorker::publish_person()
{
    std::cout << "Publish person" << std::endl;
    RoboCompHumanPose::humansDetected human_list;
    RoboCompHumanPose::PersonType person;
    
    //camera
    person.IDcamera = cameraID_sb->value();
    
    QStringList lines = person_te->toPlainText().split('\n', QString::SkipEmptyParts);
    for(auto line: lines)
    {
        person.id = line.split(',')[0].toInt();
        QStringList aux = line.mid(line.indexOf('(')+1,line.indexOf(')')-line.indexOf('(')-1).split(',');

        person.pos.x = aux[0].toFloat();
        person.pos.z = aux[1].toFloat();
        person.pos.ry = aux[2].toFloat();
        person.pos.pos_good = aux[3].contains("true");
        person.pos.rot_good = aux[4].contains("true");
        person.pos.confidence = aux[5].toInt();
        human_list.push_back(person);
qDebug()<<"PersonData"<<person.id<<person.pos.x<<person.pos.z<<person.pos.ry<<person.pos.pos_good<<person.pos.rot_good<<person.pos.confidence;
        
    }
    try
    {
        humanpose_proxy->obtainHumanPose(human_list);
    }
    catch(...)
    {
        std::cout <<"ERROR publishing person, check IceStorm is running"<<std::endl;
    }
    
}

void SpecificWorker::load_file()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open file"), "", tr("Text Files (*.txt )"));
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(0, "error", file.errorString());
        return;
    }
    QTextStream in(&file);
    QString text = in.readAll();
    person_te->clear();
    person_te->setText(text);
}

void SpecificWorker::save_file()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Save file"), "", tr("Text Files (*.txt )"));
    QFile file(filename);
    if(!file.open(QIODevice::WriteOnly)) {
        QMessageBox::information(0, "error", file.errorString());
        return;
    }
    QString content = person_te->toPlainText();
    file.write(content.toUtf8());
    file.close();
}

