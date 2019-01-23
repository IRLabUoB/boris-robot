
#!/usr/bin/env python

from boris_tools.udp_server import UDPServer
import numpy as np

class NetFTSensor(object):

    def __init__(self, **kwargs):

        self._udp_server = UDPServer(**kwargs)

        self._udp_server.connect()

    def read_wrench(self):


        data, _ = self._udp_server.receive()
        values = map(np.float64,data.split(','))

        force = values[:3]
        torque = values[3:6]
        stamp = values[6]

        return np.array(force), np.array(torque), stamp