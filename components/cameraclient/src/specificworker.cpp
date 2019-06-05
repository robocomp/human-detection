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
	joints_id = JOINTS_ID({
		"nose", 
		"left_eye", 
		"right_eye", 
		"left_ear", 
		"right_ear", 
		"left_shoulder", 
		"right_shoulder", 
		"left_elbow", 
		"right_elbow", 
		"left_wrist", 
		"right_wrist", 
		"left_hip", 
		"right_hip", 
		"left_knee", 
		"right_knee", 
		"left_ankle", 
		"right_ankle" });

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
	camcv1.release();
	camcv2.release();
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
/*	//RTMat rt( 0.825561463833, -0.0538341365755, 0.25232693553, 262.576263428, 83.0633926392, 3213.46557617); //tripode
	RTMat rt( 0.880809009075, 0.0176280960441, -0.391859769821, -386.366424561, -195.634735107, 3453.25854492); //perchero
	 RTMat rti = rt.invert();
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
	
//	cv::setMouseCallback("camera", mouse_callback, this);
	//cam.run(URL);
	
	

	this->Period = 300;
	timer.start(Period);

	//initVideoLive();
	initVideoReader();
	//createRemap(480, 640, -0.122435, -0.0633192, 0.362244);
//	createRemap(480, 640, 0.000001,0.000000000001,0);

	//descriptor
	
    

}

void SpecificWorker::initVideoLive()
{
	camcv1.open(0);
	camcv1.set(CV_CAP_PROP_FRAME_WIDTH,IWIDTH);
	camcv1.set(CV_CAP_PROP_FRAME_HEIGHT,IHEIGHT);
	camcv2.open(1);
	camcv2.set(CV_CAP_PROP_FRAME_WIDTH,IWIDTH);
	camcv2.set(CV_CAP_PROP_FRAME_HEIGHT,IHEIGHT);
	videoWriter1 = cv::VideoWriter("cameraT.avi",CV_FOURCC('M','J','P','G'), 4, cv::Size(IWIDTH, IHEIGHT));
	videoWriter2 = cv::VideoWriter("cameraP.avi",CV_FOURCC('M','J','P','G'), 4, cv::Size(IWIDTH, IHEIGHT));
}
void SpecificWorker::initVideoReader()
{
	camcv1.open("v3_cameraT.avi");
	camcv2.open("v3_cameraP.avi");
	frame_counter = 0;
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

void SpecificWorker::checkPersonImage(cv::Mat frame, std::string camera)
{
	//convert image
	RoboCompPeopleServer::TImage img;
	img.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
	img.width = frame.cols;
	img.height = frame.rows;
	img.depth = 3;
	try
	{
		scale = 0.7;
//RoboCompPeopleServer::People people;		
		auto people = peopleserver_proxy->processImage(img, 0.7);
		drawBody(frame, people, camera);
		RoboCompHumanPose::personList pList;
		int id =0;
		for(auto &person : people)
		{
			RoboCompHumanPose::JointsDescriptor jDes;
computeORBDescriptor(frame, person.joints, jDes);			
			QVec coor = getFloorCoordinates(person, camera);
			if(coor.isEmpty()) 
				qDebug() << "no bone found";
			else
			{ 
				std::cout << "Flor coor: " << coor.x() <<" "<< coor.y() <<" "<< coor.z() <<std::endl;
				personPose->setPos(coor.x(), coor.z());
				//HumanPose
				RoboCompHumanPose::PersonType pType;
				pType.id= id;
				id++;
				pType.pos.x = coor.x();
				pType.pos.z = coor.z();
				pType.jointsDescriptor = jDes;
				pList.push_back(pType);
			}
		}
		//publish_results
		try{
			if(pList.size() > 0)
			{
				RoboCompHumanPose::humansDetected humanD;
				humanD.humanList = pList;
				if(camera == "cameraT")
				{
					humanD.idCamera = 0;
					humanD.timeStamp = timeStamp1;
				}
				else{
					humanD.idCamera = 1;
					humanD.timeStamp = timeStamp2;
				}
				humanpose_pubproxy->obtainHumanPose(humanD);
			}
		}
		catch(const Ice::Exception& e)
		{
			std::cerr << e.what() << '\n';
		}
	}
	catch(const Ice::Exception& e)
	{
		std::cerr << e.what() << '\n';
	}

}

void SpecificWorker::readFrameLive(int camera, cv::Mat &frame)
{
	auto t0 = std::chrono::high_resolution_clock::now();
	if (camera == 1)
	{
		camcv1 >> frame;
		timeStamp1 =  t0.time_since_epoch() / std::chrono::milliseconds(1);
		videoWriter1.write(frame);
	}
	else
	{
		camcv2 >> frame;
		timeStamp2 =  t0.time_since_epoch() / std::chrono::milliseconds(1);
		videoWriter2.write(frame);
	}
}

void SpecificWorker::readFrameVideo(int camera, cv::Mat &frame)
{
	auto t0 = std::chrono::high_resolution_clock::now();
	try
	{
		if (camera == 1)
		{
			camcv1.read(frame);
			timeStamp1 =  t0.time_since_epoch() / std::chrono::milliseconds(1);
		}
		else
		{
			camcv2.read(frame);
			timeStamp2 =  t0.time_since_epoch() / std::chrono::milliseconds(1);
		}
	}catch(...)
	{
		qDebug()<<"Error reading frame from video";
	}
}

void SpecificWorker::compute()
{
	static auto begin = std::chrono::steady_clock::now();
	//URL
	/*auto [ret, frame] = cam.read(); //access without copy
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
	cv::imshow("remap_auto", dst2);*/
	cv::Mat frame1, frame2; 
	//live

//	readFrameLive(1, frame1);
//	readFrameLive(2, frame2);
	//video
	readFrameVideo(1,frame1);
	readFrameVideo(2,frame2);
	frame_counter += 1;
	if (frame_counter >= camcv1.get(CV_CAP_PROP_FRAME_COUNT))
	{
        frame_counter = 0;
        qDebug()<< "RELOOP VIDEO" << camcv1.set(CV_CAP_PROP_POS_FRAMES, 0),camcv2.set(CV_CAP_PROP_POS_FRAMES, 0);
		return;
	}

	//use frame
	checkPersonImage(frame1, "cameraT");
//TODO => just one camera
//	checkPersonImage(frame2, "cameraP");
	
//	pMOG2->apply(frame, fgMaskMOG2);
//	cv::erode(fgMaskMOG2, erode, kernel);
//	cv::dilate(erode, dilate, kernel);
//	auto count = cv::countNonZero(fgMaskMOG2);

	
	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << std::endl;
	begin = end;
}

QVec SpecificWorker::getFloorCoordinates(const RoboCompPeopleServer::Person &p, const std::string &camera)
{
	qDebug() << __FUNCTION__;
	{
		auto &&[good, coor] = inverseRay(p, "left_ankle", camera);	
	 	if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "right_ankle", camera);	
		if(good) return coor;
	}
/*	{
		auto &&[good, coor] = inverseRay(p, "left_knee", camera);	
		if(good) return coor;
	}
	{	
		auto &&[good, coor] = inverseRay(p, "right_knee", camera);	
		if(good) return coor;
	}
	{
		auto &&[good, coor] = inverseRay(p, "left_hip", camera);	
		if(good) return coor;
		
	}*/
	return QVec();
}

std::tuple<bool, QVec>  SpecificWorker::inverseRay(const RoboCompPeopleServer::Person &p, const std::string &joint, const std::string &camera)
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
		QMat r = innermodel->getRotationMatrixTo(QString::fromStdString(camera), "world");
		QVec p3i = r * p2;
		QVec p3 = innermodel->transform("world", p2, QString::fromStdString(camera));
//		p3i.print("P3");
		QVec cam = innermodel->transform("world", QString::fromStdString(camera));
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

void SpecificWorker::drawBody(cv::Mat frame, const RoboCompPeopleServer::People &people, std::string camera)
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
	cv::imshow(camera, frame);
	cvWaitKey(1);
}

void SpecificWorker::mouseClick(int  event, int  x, int  y)
{
	//set (x,y) reference from image center
//	x = x - IWIDTH/2;
//	y = y - IHEIGHT/2;
	std::cout<<"*************************"<<std::endl;
	std::cout<<"MOUSE "<< x <<" "<< y<<std::endl;
	std::cout<<"*************************"<<std::endl;
/*	newPoint = true;
	keypoint.x = x;
	keypoint.y = y;
*/
	

	float cx = 651;
	float cy = 512;
	float f = 1404;
	float z = 853;
	float xw = z/f * (x-cx);
	float yw = z/f * (y-cy);
	std::cout<<"Image point: "<<x<<" "<<y<<" corrected center(x-cx) "<<(x-cx)<<" (y-cy) "<<y-cy<<" 3D "<<xw<<" "<<yw<<endl;
}

void SpecificWorker::computeORBDescriptor(cv::Mat frame, RoboCompPeopleServer::TJoints joints, RoboCompHumanPose::JointsDescriptor &jDes)
{
	//convert frame to gray
	cv::Mat frameGray;
	cv::cvtColor(frame, frameGray, CV_BGR2GRAY);
std::vector<cv::KeyPoint> drawKeypoints;
	//get descriptos
	cv::Mat descriptors;
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
	jDes = std::vector<std::vector<int>>(joints_id.size(), std::vector<int>());
	unsigned int pos=0;
	for(auto &joint: joints_id)
	{
		if (joints[joint].score > 0)
		{
			//compute keypoint angle


			cv::KeyPoint p = cv::KeyPoint(joints[joint].x, joints[joint].y, DESCRIPTOR_SIZE,-1);
			std::vector<cv::KeyPoint> keypoints;
drawKeypoints.push_back(p);			
			keypoints.push_back(p);
			extractor->compute(frameGray, keypoints, descriptors);
			//copy descriptor to ice structure
			if (descriptors.rows > 0)
				jDes[pos] = descriptors.row(0);	
		}
		pos++;
	}

cv::drawKeypoints(frameGray, drawKeypoints, frameGray, cv::Scalar(0,255,0));
std::cout<<"jDes"<<jDes.size()<<std::endl;	
cv::imshow("orb", frameGray);
}