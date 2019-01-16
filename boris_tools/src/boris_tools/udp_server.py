from __future__ import print_function

import time
import socket
import numpy as np 

from boris_tools.network_utils import get_this_host_ip

class UDPServer(object):

    def __init__(self, **kwargs):

        self._ip = kwargs.get('ip', get_this_host_ip())
        self._port = kwargs.get('port', 50000)
        self._buffer_size = kwargs.get('buffer_size', 1024)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._is_closed = True

    def __del__(self):
        self._socket.close()


    def connect(self, **kwargs):

        self.close()

        self._ip = kwargs.get('ip', self._ip)
        self._port = kwargs.get('port', self._port)

        print("Opening socket at: ",(self._ip, self._port))
        self._socket.bind((self._ip, self._port))
        self._is_closed = False


    def close(self):

        if not self._is_closed:
            self._socket.close()
            self._is_closed = True

    def receive(self):

        data, addr_rec = self._socket.recvfrom(self._buffer_size)

        return data.rstrip('\x00').rstrip('\n') , addr_rec