import datetime
import random

from classes.kalman import KalmanTracker


PERSON_ALIVE_TIME = 20000

def random_hexrgb():
	r = lambda: random.randint(0, 255)
	rand_color = '#%02X%02X%02X' % (r(), r(), r())
	return rand_color

class Position2D:
	def __init__(self, x=-1, y=-1):
		self.x = x
		self.y = y


	@property
	def x(self):
		return  self._x

	@property
	def y(self):
		return self._y


	@x.setter
	def x(self, value):
		if isinstance(value,[float, int]):
			self._x = value
		else:
			raise TypeError

	@y.setter
	def y(self, value):
		if isinstance(value, [float, int]):
			self._y = value
		else:
			raise TypeError

	def __repr__(self):
		return '(%f, %f)'%(self.x, self.y)


class Person:
	def __init__(self):
		self._person_id = -1
		self._pos = Position2D()
		# self._position_history = Queue(5)
		# self._rot = 0
		self.tracker = KalmanTracker()
		self.__color = random_hexrgb()
		self._cameras = []
		self._velocity = []
		self._last_time_detected = -1
		self._last_time_predicted = -1
		self._confidence = 0


	@property
	def person_id(self):
		return self._person_id

	@property
	def confidence(self):
	    return self._confidence

	@confidence.setter
	def confidence(self, value):
	    self._confidence = value

	@property
	def pos(self):
		return self._pos

	# @property
	# def rot(self):
	# 	return self._rot

	@property
	def color(self):
		return self.__color


	@color.setter
	def color(self, value):
		assert isinstance(value, basestring) and value.startswith('#') and len(value) == 7, "Color must be given in a hex value string with the format #FFFFFFF but %s given"+str(value)
		self.__color = value

	@property
	def cameras(self):
		return self._cameras

	@person_id.setter
	def person_id(self, p_id):
		self._person_id  = p_id

	# @pos.setter
	# def pos(self, value):
	# 	self._position_history.put(self._pos)
	# 	if isinstance(value, list) and not isinstance(value, basestring):
	# 		if len(value) > 0:
	# 			self._pos.x = value[0]
	# 		if len(value) > 1:
	# 			self._pos.y = value[1]
	# 		if len(value) > 2:
	# 			self._pos.z = value[2]
	# 		if len(value) > 3:
	# 			raise IndexError
	# 	elif isinstance(value, Position):
	# 		self._pos = value
	# 	else:
	# 		raise TypeError

	def is_active(self):
		current_time = datetime.datetime.now()
		time_diff = self._last_time_detected - current_time
		if time_diff > PERSON_ALIVE_TIME:
			return False
		else:
			return True

	def predict(self):
		if self._last_time_predicted != -1:
			current_time = datetime.datetime.now()
			seconds_dt = (self._last_time_detected-current_time).total_seconds()
			predicted, cov =self.tracker.predict_with_time_diff(seconds_dt)
			self._pos = Position2D(float(predicted[0]*1000), float(predicted[2]*1000))
			self._last_time_predicted = current_time

	def update_position(self, position):
		current_time = datetime.datetime.now()
		self._last_time_detected = current_time
		self._last_time_predicted = current_time
		predicted, cov =self.tracker.update([position.x/1000, position.y/1000])
		self._pos = Position2D(float(predicted[0]*1000), float(predicted[2]*1000))


	# Reset the tracker to this position
	def initialice_tracker(self, position):
		current_time = datetime.datetime.now()
		self._last_time_detected = current_time
		self._last_time_predicted = current_time
		self.tracker.init_to_position(position.x/1000, position.y/1000)
		self._pos = position


	def detection_delta_time(self):
		current_time = datetime.datetime.now()
		delta_time = (current_time-self._last_time_detected).total_seconds()
		return delta_time

	@staticmethod
	def merge(person1, person2):
		if person1.confidence >= person2.confidence:
			return person1
		else:
			return person2

	def __repr__(self):
		return 'Person (%d) at %s'%(self.person_id, str(self.pos))