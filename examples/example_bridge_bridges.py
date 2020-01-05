#!/usr/bin/env python3

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

import sys

from ros2amqp import (
    PubConnector, ROSPubEndpoint, BrokerPubEndpoint,
    SubConnector, ROSSubEndpoint, BrokerSubEndpoint,
    RPCConnector, ROSServiceEndpoint, BrokerRPCEndpoint,
    BrokerAuthPlain, BrokerDefinition,
    ConnectorThreadExecutor
)


def main():
    executor = ConnectorThreadExecutor()

    broker = BrokerDefinition(
        name='home_broker',
        host='155.207.33.189',
        port='5672',
        vhost='/',
        global_ns=''
    )

    ros_ep_b = ROSSubEndpoint(
        msg_type='std_msgs/String',
        uri='/example_sub',
        name='b'
    )

    broker_ep_b = BrokerSubEndpoint(
        uri='example_sub',
        name='b',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(SubConnector(ros_ep_b, broker_ep_b))

    ros_ep_a = ROSPubEndpoint(
        msg_type='std_msgs/String',
        uri='/example_pub',
        name='a'
    )

    broker_ep_a = BrokerPubEndpoint(
        uri='example_pub',
        name='a',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(PubConnector(ros_ep_a, broker_ep_a))

    ros_ep_c = ROSServiceEndpoint(
        srv_type='std_srvs/SetBool',
        uri='/example_rpc',
        name='c'
    )

    broker_ep_c = BrokerRPCEndpoint(
        uri='example_rpc',
        name='c',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(RPCConnector(ros_ep_c, broker_ep_c))

    executor.run_forever()

if __name__ == "__main__":
    main()