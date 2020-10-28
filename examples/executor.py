#!/usr/bin/env python

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

from ros2broker import (
    PubConnector, ROSPubEndpoint, BrokerPubEndpoint,
    SubConnector, ROSSubEndpoint, BrokerSubEndpoint,
    RPCConnector, ROSServiceEndpoint, BrokerRPCEndpoint,
    BrokerAuthPlain, BrokerDefinition,
    ConnectorThreadExecutor
)

import sys


def main():
    broker = BrokerDefinition(
        name=None,
        host='155.207.33.189',
        port='5672',
        vhost='/',
        global_ns=''
    )

    ros_ep_1 = ROSPubEndpoint(
        msg_type='std_msgs/String',
        uri='/test_pub_con',
        name='test_pub_con'
    )

    broker_ep_1 = BrokerPubEndpoint(
        uri='test_pub_con',
        name='test_pub_con',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    ros_ep_2 = ROSSubEndpoint(
        msg_type='std_msgs/String',
        uri='/test_sub_con',
        name='test_sub_con'
    )

    broker_ep_2 = BrokerSubEndpoint(
        uri='test_sub_con',
        name='test_sub_con',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    ros_ep_3 = ROSServiceEndpoint(
        srv_type='std_srvs/Empty',
        uri='/test_srv_con',
        name='test_srv_con'
    )

    broker_ep_3 = BrokerRPCEndpoint(
        uri='test_srv_con',
        name='test_srv_con',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    pc = PubConnector(ros_ep_1, broker_ep_1)
    sc = SubConnector(ros_ep_2, broker_ep_2)
    rpc_con = RPCConnector(ros_ep_3, broker_ep_3)

    executor = ConnectorThreadExecutor()
    executor.run_connector(pc)
    executor.run_connector(sc)
    executor.run_connector(rpc_con)

    executor.run_forever()


if __name__ == "__main__":
    main()
