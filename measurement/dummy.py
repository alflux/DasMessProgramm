from .measurement import register, AbstractMeasurement, Contacts, SignalInterface
from .measurement import FloatValue, IntegerValue, DatetimeValue

from random import random
from datetime import datetime
from time import sleep


@register('Dummy Measurement')
class DummyMeasurement(AbstractMeasurement):
    def __init__(self, signal_interface: SignalInterface):
        super().__init__(signal_interface)
        self._number_of_contacts = Contacts.TWO

        self._n = 10
        self._path = ''
        self._contacts = ()

    @property
    def inputs(self):
        return {'n': IntegerValue('Number of Points', default=10)}

    @property
    def outputs(self):
        return {'random': FloatValue('Random Value'),
                'DateTime': DatetimeValue('Timestamp')}

    @property
    def recommended_plots(self):
        return [('DateTime', 'random')]

    def initialize(self, path, contacts, n=10):
        """
        :param path:
        :param contacts:
        :param n:
        :return:
        """
        self._n = n
        self._path = path
        self._contacts = contacts

    def run(self):
        self._signal_interface.emit_started()

        for i in range(self._n):
            self._signal_interface.emit_data({'DateTime': datetime.now(), 'random': random()})
            sleep(0.5)

        self._signal_interface.emit_finished({})
