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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

    RoboCompAprilTagsServer::tagsList AprilTagsServer_getAprilTags(RoboCompAprilTagsServer::Image frame, double tagsize, double mfx, double mfy);

public slots:
    void compute();
    void initialize(int period);

private:
    AprilTags::TagDetector* m_tagDetector;
    cv::Mat image_gray, image_color;
    RoboCompAprilTagsServer::tag send_detection(::AprilTags::TagDetection detection, double tagsize, double mfx, double mfy, double mpx, double mpy);
    void rotationFromMatrix(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz);
    void rotationFromMatrix2(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz);


};
inline double standardRad(double t);
#endif
