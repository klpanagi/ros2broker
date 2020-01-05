#!/usr/bin/env python{{ python_version }}

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
        name='{{ broker.name }}',
        host='{{ broker.host }}',
        port='{{ broker.port }}',
        vhost='{{ broker.vhost }}',
        global_ns='{{ broker.global_ns }}'
    )
{% for c in sub_connectors %}
    ros_ep_{{ c.ros_endpoint.name }} = ROSSubEndpoint(
        msg_type='{{ c.ros_endpoint.msg_type }}',
        uri='{{ c.ros_endpoint.uri }}',
        name='{{ c.ros_endpoint.name }}'
    )

    broker_ep_{{ c.broker_endpoint.name }} = BrokerSubEndpoint(
        uri='{{ c.broker_endpoint.uri }}',
        name='{{ c.broker_endpoint.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(SubConnector(ros_ep_{{ c.ros_endpoint.name }}, broker_ep_{{ c.broker_endpoint.name }}))
{% endfor %}{% for c in pub_connectors %}
    ros_ep_{{ c.ros_endpoint.name }} = ROSPubEndpoint(
        msg_type='{{ c.ros_endpoint.msg_type }}',
        uri='{{ c.ros_endpoint.uri }}',
        name='{{ c.ros_endpoint.name }}'
    )

    broker_ep_{{ c.broker_endpoint.name }} = BrokerPubEndpoint(
        uri='{{ c.broker_endpoint.uri }}',
        name='{{ c.broker_endpoint.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(PubConnector(ros_ep_{{ c.ros_endpoint.name }}, broker_ep_{{ c.broker_endpoint.name }}))
{% endfor %}{% for c in rpc_connectors %}
    ros_ep_{{ c.ros_endpoint.name }} = ROSServiceEndpoint(
        srv_type='{{ c.ros_endpoint.srv_type }}',
        uri='{{ c.ros_endpoint.uri }}',
        name='{{ c.ros_endpoint.name }}'
    )

    broker_ep_{{ c.broker_endpoint.name }} = BrokerRPCEndpoint(
        uri='{{ c.broker_endpoint.uri }}',
        name='{{ c.broker_endpoint.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='bot', password='b0t')
    )

    executor.run_connector(RPCConnector(ros_ep_{{ c.ros_endpoint.name }}, broker_ep_{{ c.broker_endpoint.name }}))
{% endfor %}
    executor.run_forever()

if __name__ == "__main__":
    main()
