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
		{"left_knee", 480},
		{"left_hip", 900},
		{"right_hip", 900},
		{"left_shoulder", 1500},
		{"right_shoulder", 1500}});
	

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
	config_params = params;
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	scene.setSceneRect(LEFT, BOTTOM, WIDTH, HEIGHT);
	view.scale( 1, -1 );
	view.setScene(&scene);
	view.setParent(this);
	QGridLayout* layout = new QGridLayout();
    layout->addWidget(&view);
	this->setLayout(layout);
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	resize(view.size());
	
	//add person ellipse
	personPose = scene.addEllipse(QRectF(-50,-50, 200, 200), QPen(QColor("LightGreen")), QBrush(QColor("LightGreen")));
	personPose->setFlag(QGraphicsItem::ItemIsMovable);
	personPose->setPos(0, 0);

	innermodel = std::make_shared<InnerModel>(config_params["InnerModelPath"].value);
	// RTMat rt( 0.89458078146, -0.0678672716022, 0.0201254915446, -80.604724586, 77.9478624463, 2689.19467926);
//	 RTMat rt( 0.882202804089, 0.00223149708472, 0.0275580454618,-55.7542724609, 187.768630981, 3600.34423828);
//	 RTMat rt( 0.887527406216, -0.040029399097, -0.0620594508946 , 445.338287354, -422.839385986, 2894.57250977);

/*	 RTMat rti = rt.invert();
	 rti.print("rti");
	 QVec angles = rti.extractAnglesR_min();
	 angles.print("angles");
	 exit(-1);
*/
	pMOG2 = cv::createBackgroundSubtractorMOG2();
	size_t erosion_size = 2;
	kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, 
										cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
	
	cv::namedWindow("camera", 1);
	cv::setMouseCallback("camera", mouse_callback, this);
	cam.run(URL);
	//camcv.open("videoB.webm");
	frame_counter = 0;
	//camcv.open(0);
	
	this->Period = 50;
	//timer.setSingleShot(true);
	timer.start(Period);

	videoWriter = cv::VideoWriter("outcpp.avi",0, 0, cv::Size(640, 480));
	//createRemap(480, 640, -0.122435, -0.0633192, 0.362244);
	createRemap(480, 640, 0.000001,0.000000000001,0);
}
void SpecificWorker::createRemap(int width, int height, float K1, float K2, float K3)
{
	map1.create(width, height, CV_32FC1);
	map2.create(width, height, CV_32FC1);
	for (int x=0;x<width;x++)
	{
		for (int y=0;y<height;y++)
		{
			float r = sqrt( pow(width/2 - x, 2) + pow(height/2 - y, 2) );
			float factor = ( 1 + K1*pow(r,2) + K2*pow(r,4) + K3*pow(r,6) );

			map1.at<float>(x,y) = y * factor;
			map2.at<float>(x,y) = x * factor;
		}
	}

	//other remap 
	mtx = cv::Mat::zeros(3, 3, CV_32FC1);
	mtx.at<float>(0,0) = 612;
	mtx.at<float>(1,1) = 863;
	mtx.at<float>(2,2) = 1;
	mtx.at<float>(0,2) = 341;
	mtx.at<float>(1,2) = 269;

   	dist = cv::Mat::zeros(1,5, CV_32FC1); 
	dist.at<float>(0,0) = -0.122435;
	dist.at<float>(0,1) = -0.0633192;
	dist.at<float>(0,2) = 0.0039073;
	dist.at<float>(0,3) = 0.010313;
	dist.at<float>(0,4) = 0.362244;
	std::cout<<"Mat"<<dist<<std::endl;

	newcameramtx = cv::getOptimalNewCameraMatrix(mtx,dist,cv::Size(width,height),1,cv::Size(width,height));

	cv::initUndistortRectifyMap(mtx,dist,cv::Mat(),newcameramtx, cv::Size(height, width), CV_32FC1, map1b,map2b);


}
void SpecificWorker::compute()
{
	static auto begin = std::chrono::steady_clock::now();
	static int last_people_detected = 0;
	//URL
	auto [ret, frame] = cam.read(); //access without copy
	if(ret == false) return;
	
	//remap
	cv::Mat undistor;
	cv::undistort(frame, undistor, mtx, dist);
	cv::imshow("undistor", undistor);
	//other remap
	cv::Mat dst;
	cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR);
	cv::imshow("remap_original", dst);
	//third remap
	cv::Mat dst2;
	cv::remap(frame, dst2, map1b, map2b, cv::INTER_LINEAR);
	cv::imshow("remap_auto", dst2);
	//live
//	cv::Mat frame; bool ret = true;
//	camcv >> frame;

	//video
	/*camcv.read(frame);
	frame_counter += 1;
	if (frame_counter >= camcv.get(CV_CAP_PROP_FRAME_COUNT))
	{
        frame_counter = 0;
        qDebug()<< "RELOOP VIDEO" << camcv.set(CV_CAP_PROP_POS_MSEC, 0);
		return;
	}*/
	
	videoWriter.write(frame);

//	pMOG2->apply(frame, fgMaskMOG2);
//	cv::erode(fgMaskMOG2, erode, kernel);
//	cv::dilate(erode, dilate, kernel);
//	auto count = cv::countNonZero(fgMaskMOG2);
	//if( count > 2000 or last_people_detected > 0)
	{
		RoboCompPeopleServer::TImage img;
		img.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
		img.width = frame.cols;
		img.height = frame.rows;
		img.depth = 3;

//if(newPoint)
{
		try
		{
			scale = 0.7;
//			auto people = peopleserver_proxy->processImage(img, 0.7);
//TESTING 
RoboCompPeopleServer::People people;
/*RoboCompPeopleServer::Person person;
keypoint.score = 1;
person.joints["left_knee"] = keypoint;
people.push_back(person);
*/
			last_people_detected = people.size();
			drawBody(frame, people);
			for(auto &person : people)
			{
				QVec coor = getFloorCoordinates(person);
				if(coor.isEmpty()) 
					qDebug() << "no bone found";
				else
				{ 
					std::cout << "Flor coor: " << coor.x() <<" "<< coor.y() <<" "<< coor.z() <<std::endl;
					personPose->setPos(coor.x(), coor.z());
				}
			}
		}
		catch(const Ice::Exception& e)
		{
			std::cerr << e.what() << '\n';
		}
newPoint = false;		
}	
try{
		cv::imshow("camera", frame);
}catch(...)
{
	std::cout<<"frame lost"<<std::endl;
}	
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
		auto &&[good, coor] = inverseRay(p, "right_knee");	
		if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "left_hip");	
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
std::cout<< "joint "<< joint <<" "<<j->x <<" "<< j->y<<std::endl;
		QVec p2 = Ki * p1;
		//qDebug() << __FUNCTION__ << "despues ki";
//		p2.print("P2");
		QMat r = innermodel->getRotationMatrixTo(QString::fromStdString(config_params["cameraName"].value), "world");
		QVec p3i = r * p2;
		QVec p3 = innermodel->transform("world", p2, QString::fromStdString(config_params["cameraName"].value));
//		p3i.print("P3");
		QVec cam = innermodel->transform("world", QString::fromStdString(config_params["cameraName"].value));
//		cam.print("cam");
		QVec p4 = p3 - cam;
//		p4.print("vector restado");
		double k = (-joint_heights.at(joint) - cam.y()) / p4.y();
		//double k = (- cam.y()) / p4.y();
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
				cv::circle(frame,cv::Point(j1->x, j1->y),10,cv::Scalar(0,0,255));
			if (j2->score > 0)
				cv::circle(frame,cv::Point(j2->x, j2->y),10,cv::Scalar(0,0,255));
			if (j1->score > 0 and j2->score > 0)
				cv::line(frame, cv::Point(j1->x, j1->y), cv::Point(j2->x, j2->y), cv::Scalar(0,255,0), 2);
		}
	}
	cv::imshow("camera", frame);
	cvWaitKey(1);
}

void SpecificWorker::CameraSimple_getImage(RoboCompCameraSimple::TImage &im)
{
//implementCODE

}


void SpecificWorker::mouseClick(int  event, int  x, int  y)
{
	std::cout<<"*************************"<<std::endl;
	std::cout<<"MOUSE "<< x <<" "<< y<<std::endl;
	std::cout<<"*************************"<<std::endl;
	newPoint = true;
	keypoint.x = x;
	keypoint.y = y;
}