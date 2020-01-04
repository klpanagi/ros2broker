#!/usr/bin/python2

from ros2amqp import (
    PubConnector, ROSPubEndpoint, BrokerPubEndpoint,
    SubConnector, ROSSubEndpoint, BrokerSubEndpoint,
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

    pc = PubConnector(ros_ep_1, broker_ep_1)
    pc2 = SubConnector(ros_ep_2, broker_ep_2)

    executor = ConnectorThreadExecutor()
    executor.run_connector(pc)
    executor.run_connector(pc2)

    executor.run_forever()


if __name__ == "__main__":
    main()
