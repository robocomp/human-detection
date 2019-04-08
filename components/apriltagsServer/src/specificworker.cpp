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

    timer.start(Period);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h7);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h9);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h9);
    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);

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
    QMutexLocker locker(mutex);
    //computeCODE
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


tagsList SpecificWorker::AprilTagsServer_getAprilTags(const Image &frame, const double &tagsize, const double &mfx, const double &mfy)
{
//    cout << "AprilTagsServer_getAprilTags: " <<tagsize<<", "<<mfx<<", "<<mfy<<endl;
    image_gray.create(frame.frmt.height,frame.frmt.width,CV_8UC1);
    image_color.create(frame.frmt.height,frame.frmt.width,CV_8UC3);
    RoboCompAprilTagsServer::tagsList tagsList1;
    try
    {
        image_color.data = (uchar *)(&frame.data[0]);
        cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
        vector< ::AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
//        std::cout << detections.size() << " tags detected:" << std::endl;
        if (detections.size() > 0)
            for (auto &d : detections)
                tagsList1.push_back(send_detection(d, tagsize, mfx, mfy, frame.frmt.width/2, frame.frmt.height/2));
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
    return(tagsList1);

}

void SpecificWorker::rotationFromMatrix(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz) {
    QMat v(3,3);
    for (uint32_t f=0; f<3; f++)
    {
        for (uint32_t c=0; c<3; c++)
        {
            v(f,c) = R(f,c);
        }
    }
    QVec ret = v.extractAnglesR_min();
    rx = ret(0)+M_PIl;
    while (rx >= M_PIl) rx -= 2.*M_PIl;
    while (rx < -M_PIl) rx += 2.*M_PIl;
    ry = -ret(1);
    rz = ret(2);

}

RoboCompAprilTagsServer::tag SpecificWorker::send_detection(::AprilTags::TagDetection detection, double tagsize, double mfx, double mfy, double mpx, double mpy)
{
    cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ") ";
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tagsize, mfx, mfy, mpx, mpy, translation, rotation);
    QVec T(3);
    T(0) = -translation(1);//*0.65;
    T(1) =  translation(2);//*0.65;
    T(2) =  translation(0);//*0.65;
    Eigen::Matrix3d F;
    F << 1, 0,  0,	0,  -1,  0,	0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;

    double rx, ry, rz;
    rotationFromMatrix(fixed_rot, rx, ry, rz);
    cout << mfx << "  " << mfy << endl;
    cout << "  distance=" << T.norm2() << ", x=" << T(0) << ", y=" << T(1) << ", z=" << T(2) << ", rx=" << rx << ", ry=" << ry << ", rz=" << rz << endl;

    RoboCompAprilTagsServer::tag t;
    t.id=detection.id;
    t.tx=T(0);
    t.ty=T(1);
    t.tz=T(2);
    t.rx=rx;
    t.ry=ry;
    t.rz=rz;

    return t;

}