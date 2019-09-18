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

//    timer.start(Period);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h7);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes25h9);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h9);
//    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes16h5);
    m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);
//    image_gray.create(240,320,CV_8UC1);
//    image_color.create(240,320,CV_8UC3);
    image_gray.create(1080,1920,CV_8UC1);
    image_color.create(1080,1920,CV_8UC3);
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = 5;
    timer.start(Period);
}

void SpecificWorker::compute()
{
    QMutexLocker locker(mutex);
    imshow("TagDetections", image_color);
    cv::waitKey(1);
}

tagsList SpecificWorker::AprilTagsServer_getAprilTags(const Image &frame, const double &tagsize, const double &mfx, const double &mfy)
{
 //   cout << "AprilTagsServer_getAprilTags: " <<tagsize<<", "<<mfx<<", "<<mfy<<" resolution: ("<<frame.frmt.width<<","<<frame.frmt.height<<")"<<endl;
    RoboCompAprilTagsServer::tagsList tagsList1;
    try
    {
        memcpy(image_color.data, &frame.data[0], frame.frmt.width*frame.frmt.height*sizeof(uchar)*3);
        cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
        vector< ::AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
        std::cout << detections.size() << " tags detected:" << std::endl;
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
void SpecificWorker::rotationFromMatrix2(const Eigen::Matrix3d &wRo, double &yaw, double &pitch, double &roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
/**
 * Normalize angle to be within the interval [-pi,pi].
 */
const double PI = 3.14159265358979323846;
const double TWOPI = 2.0*PI;
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}
RoboCompAprilTagsServer::tag SpecificWorker::send_detection(::AprilTags::TagDetection detection, double tagsize, double mfx, double mfy, double mpx, double mpy)
{
//    cout << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ") "<<endl;
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tagsize, mfx, mfy, mpx, mpy, translation, rotation);
cout <<rotation <<endl;
    QVec T(3);
    T(0) = -translation(1);//*0.65;
    T(1) =  translation(2);//*0.65;
    T(2) =  translation(0);//*0.65;
    Eigen::Matrix3d F;
    F << 1, 0,  0,	0,  -1,  0,	0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;

    double rx, ry, rz;
    rotationFromMatrix(fixed_rot, rx, ry, rz);
//    cout << "ORIGINAL ROTATION"<<endl;
    cout << mfx << "  " << mfy << endl;
    cout << "ORI  distance=" <<", rx=" << rx << ", ry=" << ry << ", rz=" << rz << endl;
    
//    cout << "NEW ROTATION"<<endl;
    rotationFromMatrix2(fixed_rot, rz, ry, rx);
    cout << mfx << "  " << mfy << endl;
    cout << "NEW  distance=" <<", rx=" << rx << ", ry=" << ry << ", rz=" << rz << endl;

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

