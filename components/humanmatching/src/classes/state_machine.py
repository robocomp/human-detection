import sys

from PySide2.QtCore import QTimer, QState, QFinalState, QStateMachine, SIGNAL, Signal


class HumanMatchingStateMachine(QStateMachine):
	initial_to_first_person = Signal()
	first_person_to_tracking = Signal()
	prediction_to_prediction = Signal()
	prediction_to_data_association = Signal()
	data_association_to_data_update = Signal()
	data_update_to_prediction = Signal()



	def __init__(self, parent):
		super(HumanMatchingStateMachine, self).__init__()
		self._parent = parent
		self._test_timer = QTimer()
		self._prediction_timer = QTimer()

		self._initial_state = QState()


		self._first_person_state = QState()


		self._tracking_state = QState()

		self._prediction_state = QState(self._tracking_state)
		self._data_association_state = QState(self._tracking_state)
		self._data_update_state = QState(self._tracking_state)

		self._final_state = QFinalState()

		self._tracking_state.setInitialState(self._prediction_state)


		self.addState(self._initial_state)
		self.addState(self._first_person_state)
		self.addState(self._tracking_state)
		# self.addState(self._prediction_state)
		# self.addState(self._data_association_state)
		# self.addState(self._data_update_state)
		self.setInitialState(self._initial_state)

		self._initial_state.addTransition(self.initial_to_first_person, self._first_person_state)
		self._first_person_state.addTransition(self.first_person_to_tracking, self._tracking_state)
		self._prediction_state.addTransition(self._test_timer.timeout, self._prediction_state)
		self._prediction_state.addTransition(self.prediction_to_data_association, self._data_association_state)
		self._data_association_state.addTransition(self.data_association_to_data_update, self._data_update_state)
		self._data_update_state.addTransition(self.data_update_to_prediction, self._prediction_state)


		self._initial_state.entered.connect(self._parent.initial_state_entered)
		self._first_person_state.entered.connect(self._parent.first_person_state_entered)
		self._prediction_state.entered.connect(self._parent.prediction_state_entered)
		self._data_association_state.entered.connect(self._parent.data_association_state_entered)
		self._data_update_state.entered.connect(self._parent.data_update_state_entered)

		# self._initial_state.addTransition(self, SIGNAL('new_humans_signal()'), self._first_person_state)
		# self._first_person_state.addTransition(self, SIGNAL('first_humans_updated_signal()'), self._tracking_state)
		# self._prediction_state.addTransition(self._prediction_timer, SIGNAL('timeout()'), self._prediction_state)
		# self._prediction_state.addTransition(self, SIGNAL('new_humans_signal()'), self._data_association_state)
		# self._data_association_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._data_update_state)

		# self._data_association_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._data_update_state)
		# self._data_update_state.addTransition(self._test_timer, SIGNAL('timeout()'), self._prediction_state)

		# self._initial_state.entered.connect(self._initial_state_method)
		# self._first_person_state.entered.connect(self._first_person_state_method)
		# self._prediction_state.entered.connect(self._prediction_state_method)
		# self._data_association_state.entered.connect(self._data_association_state_method)
		# self._data_update_state.entered.connect(self._data_update_state_method)

		self._test_timer.start(1000/30)

	# def initial_state_entered(self):
	# 	print("Error. Lack of initial_state_entered method implementation")
	# 	sys.exit(-1)
	#
	# def first_person_state_entered(self):
	# 	print("Error. Lack of first_person_state_entered method implementation")
	# 	sys.exit(-1)
	#
	# def prediction_state_entered(self):
	# 	print("Error. Lack of prediction_state_entered method implementation")
	# 	sys.exit(-1)
	#
	# def prediction_state_entered(self):
	# 	print("Error. Lack of prediction_state_entered method implementation")
	# 	sys.exit(-1)
	#
	# def data_association_state_entered(self):
	# 	print("Error. Lack of data_association_state_entered method implementation")
	# 	sys.exit(-1)
	#
	# def data_update_state_entered(self):
	# 	print("Error. Lack of data_update_state_entered method implementation")
	# 	sys.exit(-1)
