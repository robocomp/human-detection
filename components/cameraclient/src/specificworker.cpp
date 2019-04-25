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
#include <chrono>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
	skeleton = SKELETON_CONNECTIONS({
		{"left_ankle","left_knee"},
		{"left_knee","left_hip"},
		{"right_ankle", "right_knee"},
		{"right_knee", "right_hip"},
		{"left_hip", "right_hip"},
		{"left_shoulder", "left_hip"},
		{"right_shoulder", "right_hip"},
		{"left_shoulder", "right_shoulder"},
		{"left_shoulder", "left_elbow"},
		{"right_shoulder", "right_elbow"},
		{"left_elbow", "left_wrist"},
		{"right_elbow", "right_wrist"},		
		{"left_eye", "right_eye"},		
		{"nose", "left_eye"},
		{"nose", "right_eye"},
		{"left_eye", "left_ear"},
		{"right_eye", "right_ear"},
		{"left_ear", "left_shoulder"},
		{"right_ear", "right_shoulder"}});

		joint_heights = HUMAN_JOINT_HEIGHTS({
		{"left_ankle", 100},
		{"right_ankle", 100},
		{"right_knee", 480},
		{"left_hip", 480},
		{"left_shoulder", 900},
		{"right_shoulder", 900}});
	
		const float fx=830, fy=830, sx=1, sy=1, Ox=320, Oy=240;
		K = QMat::zeros(3,3);
		K(0,0) = -fx/sx; K(0,1) = 0.f; 		K(0,2) = Ox;
		K(1,0) = 0; 	 K(1,1) = -fy/sy; 	K(1,2) = Oy;
		K(2,0) = 0;		 K(2,1) = 0;		K(2,2) = 1;
		Ki = K.invert();
		Ki.print("Ki");
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	camcv.release();
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	//resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.setParent(this);
	QGridLayout* layout = new QGridLayout();
    layout->addWidget(&view);
	this->setLayout(layout);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

	//add person ellipse
	personPose = scene.addEllipse(QRectF(-50,-50, 200, 200), QPen(QColor("LightGreen")), QBrush(QColor("LightGreen")));
	personPose->setFlag(QGraphicsItem::ItemIsMovable);
	personPose->setPos(0, 0);

	innermodel = std::make_shared<InnerModel>("world.xml");
	// RTMat rt( 0.89458078146, -0.0678672716022, 0.0201254915446, -80.604724586, 77.9478624463, 2689.19467926);
//	 RTMat rt( 0.882202804089, 0.00223149708472, 0.0275580454618,-55.7542724609, 187.768630981, 3600.34423828);
//	 RTMat rti = rt.invert();
//	 rti.print("rti");
//	 QVec angles = rti.extractAnglesR_min2();
//	 angles.print("angles");
//	 exit(-1);

	pMOG2 = cv::createBackgroundSubtractorMOG2();
	size_t erosion_size = 2;
	kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, 
										cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
	
	cv::namedWindow("camera", 1);
	cv::setMouseCallback("camera", mouse_callback, this);
	//cam.run(URL);
	//camcv.open("/home/pbustos/Descargas/T1.mp4");
	camcv.open(0);
	
	this->Period = 50;
	//timer.setSingleShot(true);
	timer.start(Period);
	
}

void SpecificWorker::compute()
{
	static auto begin = std::chrono::steady_clock::now();
	static int last_people_detected = 0;
	//auto [ret, frame] = cam.read();	//access without copy
	cv::Mat frame; bool ret = true;
	camcv >> frame;  

	if(ret == false) return;
	pMOG2->apply(frame, fgMaskMOG2);
	cv::erode(fgMaskMOG2, erode, kernel);
	cv::dilate(erode, dilate, kernel);
	auto count = cv::countNonZero(fgMaskMOG2);
	//if( count > 2000 or last_people_detected > 0)
	{
		RoboCompPeopleServer::TImage img;
		img.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
		img.width = frame.cols;
		img.height = frame.rows;
		img.depth = 3;
if(newPoint){
		try
		{
			scale = 0.7;
			//auto people = peopleserver_proxy->processImage(img, 0.7);
//TESTING 
RoboCompPeopleServer::People people;
RoboCompPeopleServer::Person person;
keypoint.score = 1;
person.joints["left_ankle"] = keypoint;
people.push_back(person);
			
			last_people_detected = people.size();
		//	drawBody(frame, people);
			for(auto &person : people)
			{
				QVec coor = getFloorCoordinates(person);
				if(coor.isEmpty()) qDebug() << "no bone found";
				else qDebug() << "Flor coor: " << coor.x() << coor.y() << coor.z();
				personPose->setPos(coor.x(), coor.y());
			}

			//	go from feet upwards
			//		compute floor position
			// publish results
		}
		catch(const Ice::Exception& e)
		{
			std::cerr << e.what() << '\n';
		}
newPoint = false;		
}	
		cv::imshow("camera", frame);
		
		//qDebug() << "compute" << frame.rows << frame.cols;
		cvWaitKey(1);
		auto end = std::chrono::steady_clock::now();
//		std::cout << "Count = " << count << " Elapsed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << std::endl;
		begin = end;
	}
	
}

QVec SpecificWorker::getFloorCoordinates(const RoboCompPeopleServer::Person &p)
{
	qDebug() << __FUNCTION__;
	{
		auto &&[good, coor] = inverseRay(p, "left_ankle");	
	 	if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "right_ankle");	
		if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "left_knee");	
		if(good) return coor;
	}
	{	
		auto &&[good, coor] = inverseRay(p, "left_knee");	
		if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "right_knee");	
		if(good) return coor;
	}
	return QVec();
}

std::tuple<bool, QVec>  SpecificWorker::inverseRay(const RoboCompPeopleServer::Person &p, const std::string &joint)
{
	auto j = &p.joints.at(joint);	
	if( j->score != 0 )
	{
		//qDebug() << __FUNCTION__ << "entro";
		QVec p1 = QVec::vec3(j->x, j->y, 1.0);
		//qDebug() << __FUNCTION__ << "hola";
		QVec p2 = Ki * p1;
		//qDebug() << __FUNCTION__ << "despues ki";
		p2.print("P2");
		QVec p3 = innermodel->transform("world", p2, "camera");
		p3.print("P3");
		QVec cam = innermodel->transform("world", "camera");
		cam.print("cam");
		QVec p4 = cam - p3;
		p4.print("vector restado");
		//double k = (-joint_heights.at(joint) - cam.y()) / p3.y();
		double k = (- cam.y()) / p4.y();
		return std::make_tuple(true, cam + (p4 * (T)k));
	}
	else
		return std::make_tuple(false, QVec());
}

void SpecificWorker::drawBody(cv::Mat frame, const RoboCompPeopleServer::People &people)
{
	for(auto &p : people)
	{
		qDebug() << __FUNCTION__ <<"person id = "<<p.id;
		for (auto &[first, second] : skeleton)
		{
			auto j1 = &p.joints.at(first);
			auto j2 = &p.joints.at(second);
			if (j1->score > 0)
				cv::circle(frame,cv::Point(j1->x/scale, j1->y/scale),10,cv::Scalar(0,0,255));
			if (j2->score > 0)
				cv::circle(frame,cv::Point(j2->x/scale, j2->y/scale),10,cv::Scalar(0,0,255));
			if (j1->score > 0 and j2->score > 0)
				cv::line(frame, cv::Point(j1->x/scale, j1->y/scale), cv::Point(j2->x/scale, j2->y/scale), cv::Scalar(0,255,0), 2);
		}
	}
	cv::imshow("Camara IP", frame);
	cvWaitKey(1);
}

void SpecificWorker::CameraSimple_getImage(RoboCompCameraSimple::TImage &im)
{
//implementCODE

}


void SpecificWorker::mouseClick(int  event, int  x, int  y)
{
	std::cout<<"*********************\n*************************";
	std::cout<<"MOUSE"<< x<< y;
	std::cout<<"*********************\n*************************";
	newPoint = true;
	keypoint.x = x;
	keypoint.y = y;
}