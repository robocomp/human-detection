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
	connect(clear_pb, SIGNAL(pressed()), this, SLOT(clear_list()));
    connect(publish_timer, SIGNAL(timeout()), this, SLOT(publish_next()));
	connect(frames_list, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(frame_clicked(QListWidgetItem *)));
    controlKeyPressed = false;
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
	if(!controlKeyPressed) {
		FakePoses pose = this->ui_to_human_struct();
		this->publish_humans(pose.data);
	}
	else
	{
		if(frames_list->count() > 0)
		{
			current_frame_index = 0;
			publish_pb->setEnabled(false);
			qDebug() << "\tStarting publish timer with" << timer_sb->value();
			publish_next();
			publish_timer->start(float(timer_sb->value()) * 1000);
		} else {
			QMessageBox::warning(this, "No data to publish in list", "No data to publish in list.\nPlease add some frames to be published.");
		}
	}
}

void SpecificWorker::publish_next()
{

	if(current_frame_index < frames_list->count())
	{
		qDebug()<<"To publish next"<<current_frame_index<<frames_list->count();
		auto item = frames_list->item(current_frame_index);
		auto vari = item->data(Qt::UserRole);
		FakePoses poses = vari.value<FakePoses>();
		qDebug()<<"Humans detected:"<<poses.data.humanList.size();
		this->publish_humans(poses.data);
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

FakePoses SpecificWorker::ui_to_human_struct() {
	FakePoses pose;
	pose.text = person_te->toPlainText();
    RoboCompHumanPose::PersonType person;

    //camera
    pose.data.idCamera = cameraID_sb->value();
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
        pose.data.humanList.push_back(person);
        qDebug() << "\tPersonData" << person.id << person.pos.x << person.pos.z << person.pos.ry << person.pos.posGood
                 << person.pos.rotGood << person.pos.confidence;

    }
    return pose;
}

void SpecificWorker::keyPressEvent(QKeyEvent *event)
{
	if(event->key() == Qt::Key_Control){
		save_pb->setText("Save all");
		load_pb->setText("Load all");
		publish_pb->setText("Publish all");
		controlKeyPressed = true;
	}
}

void SpecificWorker::keyReleaseEvent(QKeyEvent *event)
{

	if(event->key() == Qt::Key_Control){
		save_pb->setText("Save");
		load_pb->setText("Load");
		publish_pb->setText("Publish");
		controlKeyPressed = false;
	}
}

void SpecificWorker::load_file()
{
	if(!controlKeyPressed)
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
	else
	{
		qDebug() << "Loading multiple poses from files:...";
		//Select directory
		QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
														"",
														QFileDialog::ShowDirsOnly
														| QFileDialog::DontResolveSymlinks);
		if(!dir.isEmpty())
		{
			QDirIterator it(dir, QStringList() << "*.txt", QDir::Files);
			while (it.hasNext()) {
				auto the_file = it.next();
				qDebug() << "\tLoading from file"<<the_file;
				QFile file(the_file);
				if(!file.open(QIODevice::ReadOnly)) {
					QMessageBox::information(0, "error", file.errorString());
					return;
				}
				QTextStream in(&file);
				QString text = in.readAll();
				person_te->clear();
				person_te->setText(text);
				name_le->setText(QFileInfo(file).baseName());
				this->add_frame();
			}
		}
	}
	frames_list->sortItems();
}

void SpecificWorker::save_file(QString filename, QString text)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly)) {
		QMessageBox::information(0, "error", file.errorString());
		return;
	}
	file.write(text.toUtf8());
	file.close();
}

void SpecificWorker::save_file()
{
	if(!controlKeyPressed) {
		QString filename = QFileDialog::getSaveFileName(this, tr("Save file"), "", tr("Text Files (*.txt )"));
		this->save_file(filename, person_te->toPlainText());
	}
	else
	{
		//Select directory
		QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
														"",
														QFileDialog::ShowDirsOnly
														| QFileDialog::DontResolveSymlinks);
		if(!dir.isEmpty())
		{


			//loop over items in list
			for(int i = 0; i < frames_list->count(); ++i)
			{
				auto item = frames_list->item(i);
				auto name = item->text();
				auto vari = item->data(Qt::UserRole);
				FakePoses poses = vari.value<FakePoses>();
				qDebug()<<"Humans detected:"<<poses.data.humanList.size();
				//for item, save struct to file
				auto file_path = QDir::cleanPath(dir + QDir::separator() + name+".txt");
				qDebug()<<"Writting to: "<<file_path;

				this->save_file(file_path, poses.text);
			}
		}

	}
}


void SpecificWorker::clear_list() {
	frames_list->clear();
}

void SpecificWorker::add_frame() {
    QString name = name_le->text();
    if(!name.isEmpty())
    {
        FakePoses poses = this->ui_to_human_struct();
        QListWidgetItem *item = new QListWidgetItem();
        item->setText(name);
        QVariant a = QVariant::fromValue<FakePoses>( poses);
        item->setData(Qt::UserRole, a);
        frames_list->addItem(item);
    }
    else
    {
        QMessageBox::information(
                this,
                tr("Name of frame"),
                tr("You have to set a name for the frame.") );
        name_le->setFocus();
    }
}

void SpecificWorker::frame_clicked(QListWidgetItem *item) {
	auto vari = item->data(Qt::UserRole);
	FakePoses poses = vari.value<FakePoses>();
	name_le->setText(item->text());
	cameraID_sb->setValue(poses.data.idCamera);
	person_te->setText(poses.text);
}

