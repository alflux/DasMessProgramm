from .vsm_measurement import register, AbstractMeasurement, SignalInterface, PlotRecommendation
from .vsm_measurement import AbstractValue, FloatValue, IntegerValue, DatetimeValue

from random import random
from datetime import datetime
from time import sleep
from typing import Tuple, Dict, List
from typing.io import TextIO
from serial import Serial

from scientificdevices.scientificMagnetics.smc import SMC
from scientificdevices.stanfordResearchSystems.sr830m import SR830m
from scientificdevices.lakeshore.ls340 import LS340
from scientificdevices.vsm.vsmEncoder.vsmEncoder import VsmEncoder
from scientificdevices.vsm.vsmMotorControl.vsmMotor import VsmMotor

@register('VSM Sample Position')
class VsmPositionMeasurement(AbstractMeasurement):
    """This is a Dummy Measurement"""
    def __init__(self,
                 signal_interface: SignalInterface,
                 path: str,
                 bField: float,
                 start: int,
                 stop: int,
                 step: int,
                 amplitude: float,
                 gpib_magnet: str = 'GPIB0::4::INSTR',
                 gpib_lia: str = 'GPIB0::7::INSTR',
                 gpib_temp: str = 'GPIB0::12::INSTR',
                 frequency: float = 44,
                 comment: str = '') -> None:
        super().__init__(signal_interface, path)

        # Geräte intialisieren
        self.magnet = SMC(GPIBPort = gpib_magnet)
        self.lia = SR830m(GPIBPort = gpib_lia)
        self.liaReset()
        self.temperature = LS340(GPIBPort = gpib_temp)
        for i in [0, 1]:
            string = '/dev/ttyACM{}'.format(i)
            self.initArduino(port=string)

        # Parameter initialisieren
        self.start = start
        self.stop = stop
        self.step = step
        self.amplitude = amplitude
        self.bField = bField
        self.frequency = frequency


    def liaReset(self):
        self.lia.ilin = 2  # 2-Linefilter ein
        self.lia.fmod = 1  # Interne Referenz
        self.lia.slvl = 0.004  # Schalte Ausgang ab
        self.lia.harm = 1
        # TODO: Noise-Level?
        # TODO: sensitivity unempfindlich setzen?
        # TODO: Zeitkonstante setzen

    def initArduino(self, port: str):
        _unknownDevice = Serial(port, 115200)
        time.sleep(2)
        _unknownDevice.write('V'.encode('ascii'))
        text = _unknownDevice.readline().decode().strip('\r\n')
        _unknownDevice.close()
        time.sleep(1)

        if text == 'ENCODER':
            self.encoder = VsmEncoder(SerialPort=port)
        elif text == 'MOTOR':
            self.motor = VsmMotor(SerialPort=port)



    @staticmethod
    def inputs() -> Dict[str, AbstractValue]:
        return {'start': IntegerValue('Start', default = 800),
                'stop': IntegerValue('Stop', default = 2200),
                'step': IntegerValue('Step', default = 50),
                'amplitude': FloatValue('Amplitude / V', default = 0.150),
                'bField': FloatValue('B-Feld / T', default = 0.5),
                'frequency': FloatValue('Frequency / Hz', default = 44)}

    @staticmethod
    def outputs() -> Dict[str, AbstractValue]:
        return {'r': FloatValue('LIA R'),
                'theta': FloatValue('LIA Theta'),
                'position' : FloatValue('Position'),
                'datetime': DatetimeValue('Timestamp')}


    @property
    def recommended_plots(self) -> List[PlotRecommendation]:
        return [PlotRecommendation('Positionsabhängigkeit',
                                   x_label='position', y_label='r')
                ]

    def _measure(self, file_handle) -> None:
        self.__print_header(file_handle)

        for i in range(self._n):
            print('{} {} {}'.format(datetime.now().isoformat(), random(), random()), file=file_handle)
            file_handle.flush()
            self._signal_interface.emit_data({'datetime': datetime.now(),
                                              'random1': random(),
                                              'random2': random()})


            sleep(1)
            if self._should_stop.is_set():
                self._signal_interface.emit_aborted()
                break

        self._signal_interface.emit_finished(self._recommended_plot_file_paths)

    def __print_header(self, fil: TextIO) -> None:
        print('#WAZAUP?', file=fil)
        print('datetime random1 random3', file=fil)
