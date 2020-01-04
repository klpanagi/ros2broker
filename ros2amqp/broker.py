# Copyright (C) 2020  Panayiotou, Konstantinos <klpanagi@gmail.com>
# Author: Panayiotou, Konstantinos <klpanagi@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import (
    absolute_import,
    division,
    print_function,
    unicode_literals
)

import uuid


class BrokerDefinition(object):

    SUPPORTED_PROTOCOLS = ['AMQP']

    def __init__(
        self,
        name=None,
        host='localhost',
        port='5672',
        vhost='/',
        global_ns='',
        protocol='AMQP'
    ):
        """__init__

        :param name: Name of the broker / Optional
        :param host: The domain of the server hosting the broker.
        :param port: The port of the broker. Differs between protocols
        :param vhost: The vhost to connect to.
        :param global_ns: A global namespace for the communication
        :param protocol: Protocol to use for messaging communication
        """
        # TODO: Make Enumeration class
        if protocol not in self.SUPPORTED_PROTOCOLS:
            raise ValueError('')
        self._name = name
        self._host = host
        self._port = port
        self._vhost = vhost
        self._global_ns = global_ns
        self._protocol = protocol

        if self._name is None:
            self._name = '{}-{}'.format(self.__class__.__name__,
                                        str(uuid.uuid4().fields[-1])[:7]
                                        )

    @property
    def host(self):
        return self._host

    @property
    def port(self):
        return self._port

    @property
    def vhost(self):
        return self._vhost

    @property
    def global_ns(self):
        return self._global_ns

    @property
    def protocol(self):
        return self._protocol
