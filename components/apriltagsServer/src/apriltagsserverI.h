/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#ifndef APRILTAGSSERVER_H
#define APRILTAGSSERVER_H

// Ice includes
#include <Ice/Ice.h>
#include <AprilTagsServer.h>

#include <config.h>
#include "genericworker.h"


class AprilTagsServerI : public virtual RoboCompAprilTagsServer::AprilTagsServer
{
public:
	AprilTagsServerI(GenericWorker *_worker);
	~AprilTagsServerI();

	RoboCompAprilTagsServer::tagsList getAprilTags(RoboCompAprilTagsServer::Image frame, double tagsize, double mfx, double mfy, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
