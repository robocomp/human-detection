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
	publish_timer = new QTimer();
	current_frame_index = 0;
    connect(publish_pb, SIGNAL(pressed()), this, SLOT(publish_clicked()));
    connect(save_pb, SIGNAL(pressed()), this, SLOT(save_file()));
    connect(load_pb, SIGNAL(pressed()), this, SLOT(load_file()));
    connect(add_pb, SIGNAL(pressed()), this, SLOT(add_frame()));
    connect(publish_timer, SIGNAL(timeout()), this, SLOT(publish_next()));
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



void SpecificWorker::publish_humans(RoboCompHumanPose::humansDetected humans_detected)
{

	try {
		humanpose_pubproxy->obtainHumanPose(humans_detected);
	}
	catch (...) {
		std::cout << "ERROR publishing person, check IceStorm is running" << std::endl;
	}

}

void SpecificWorker::publish_clicked()
{
	auto selected_items = frames_list->selectedItems();
	qDebug()<<"Items selected:"<<frames_list->selectedItems().size();
	if(selected_items.isEmpty()) {
		RoboCompHumanPose::humansDetected humans_detected = this->ui_to_human_struct();
		this->publish_humans(humans_detected);
	}
	else
	{
		current_frame_index = 0;
		publish_pb->setEnabled(false);
		qDebug()<<"\tStarting publish timer with"<<timer_sb->value();
		publish_next();
		publish_timer->start(float(timer_sb->value())*1000);
	}
}

void SpecificWorker::publish_next()
{

	if(current_frame_index < frames_list->selectedItems().size())
	{
		qDebug()<<"To publish next"<<current_frame_index<<frames_list->selectedItems().size();
		auto item = frames_list->item(current_frame_index);
		auto vari = item->data(Qt::UserRole);
		RoboCompHumanPose::humansDetected humans_detected = vari.value<RoboCompHumanPose::humansDetected>();
		qDebug()<<"Humans detected:"<<humans_detected.humanList.size();
		this->publish_humans(humans_detected);
		current_frame_index++;
	}
	else
	{
		qDebug()<<"Stoping publish task";
		publish_pb->setEnabled(true);
		current_frame_index=0;
		publish_timer->stop();
	}
}

RoboCompHumanPose::humansDetected SpecificWorker::ui_to_human_struct() {
    RoboCompHumanPose::humansDetected humans_detected;
    RoboCompHumanPose::PersonType person;

    //camera
    humans_detected.idCamera = cameraID_sb->value();
	qDebug()<<"Converting UI data to Humans Detected Struct...";
	QStringList lines = person_te->toPlainText().split('\n', QString::SkipEmptyParts);
    for (auto line: lines) {
        person.id = line.split(',')[0].toInt();
        QStringList aux = line.mid(line.indexOf('(') + 1, line.indexOf(')') - line.indexOf('(') - 1).split(',');

        person.pos.x = aux[0].toFloat();
        person.pos.z = aux[1].toFloat();
        person.pos.ry = aux[2].toFloat();
        person.pos.posGood = aux[3].contains("true");
        person.pos.rotGood = aux[4].contains("true");
        person.pos.confidence = aux[5].toInt();
        humans_detected.humanList.push_back(person);
        qDebug() << "\tPersonData" << person.id << person.pos.x << person.pos.z << person.pos.ry << person.pos.posGood
                 << person.pos.rotGood << person.pos.confidence;

    }
    return humans_detected;
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


void SpecificWorker::add_frame() {
    QString name = name_te->text();
    if(!name.isEmpty())
    {
        RoboCompHumanPose::humansDetected humans_detected = this->ui_to_human_struct();
        QListWidgetItem *item = new QListWidgetItem();
        item->setText(name);
        QVariant a = QVariant::fromValue<RoboCompHumanPose::humansDetected>( humans_detected);
        item->setData(Qt::UserRole, a);
        frames_list->addItem(item);
    }
    else
    {
        QMessageBox::information(
                this,
                tr("Name of frame"),
                tr("You have to set a name for the frame.") );
        name_te->setFocus();
    }


}

