from classes.person import Person, Position2D


class CameraFrame:

    def __init__(self):
        self.__id_camera = -1
        self.__person_list = []
        self.__timestamp = None


    @property
    def person_list(self):
        return self.__person_list

    @person_list.setter
    def person_list(self, value):
        return self.__person_list

    @property
    def timestamp(self):
        return self.__timestamp

    @timestamp.setter
    def timestamp(self, value):
        self.__timestamp = value

    @property
    def person_list(self):
        return self.__person_list

    @person_list.setter
    def person_list(self, value):
        assert isinstance(value, list), "Person list of CameraFrame must be a list. %s given" % str(type(value))
        self.__person_list = value

    @staticmethod
    def from_ice_struct(humansFromCam):
        # 	struct humansDetected
        # 	{
        # 		 int idCamera;
        #          long timeStamp;
        # 		 personList humanList;
        # 	};
        new_camera_frame = CameraFrame()
        new_camera_frame.id_camera = humansFromCam.idCamera
        new_camera_frame.timestamp = humansFromCam.timeStamp
        new_camera_frame.person_list = []
        for cam_person in humansFromCam.humanList:
            detected_person = Person()
            # TODO: is the camera id the one we assume for the person? How we update this?
            detected_person.person_id = cam_person.id
            detected_person.confidence = cam_person.pos.confidence
            detected_person.cameras.append(humansFromCam.idCamera)
            detected_person.initialice_tracker(Position2D(cam_person.pos.x, cam_person.pos.z))
            new_camera_frame.person_list.append(detected_person)
        return new_camera_frame