from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import uuid

import rospy
import amqp_common
from rosconversions import ros_msg_to_dict, get_message_class

from .endpoint import *


class Connector(object):
    def __init__(self, name=None, debug=False):
        self._name = name
        self._debug = debug
        self._ros_endpoint = None
        self._broker_endpoint = None

        if self._name is None:
            self._name = '{}-{}'.format(self.__class__.__name__,
                                        str(uuid.uuid4().fields[-1])[:7]
                                        )

    @property
    def name(self):
        return self._name

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, val):
        self._debug = val

    @property
    def ros_endpoint(self):
        return self._ros_endpoint

    @property
    def broker_endpoint(self):
        return self._broker_endpoint

    def run(self): raise NotImplementedError()


class RPCConnector(Connector):
    def __init__(
            self,
            ros_endpoint,
            broker_endpoint,
            *args,
            **kwargs
    ):
        super(RPCConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint


class PubConnector(Connector):
    def __init__(
        self,
        ros_endpoint,
        broker_endpoint,
        *args,
        **kwargs
    ):
        super(PubConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint

    def _init_ros_subscriber(self):
        if self.debug:
            log_level = rospy.DEBUG
        else:
            log_level = rospy.INFO
        _uri = self.ros_endpoint.uri
        _node_name = self.ros_endpoint.name
        _msg_type = self.ros_endpoint.msg_type

        _msg_pkg = _msg_type.split('/')[0]
        _msg_cls  = _msg_type.split('/')[1]

        msg_cls = get_message_class(_msg_type)

        # Initialize ROS Subscriber
        self._ros_sub = rospy.Subscriber(_uri, msg_cls, self._ros_sub_callback)
        rospy.loginfo('ROS Subscriber <{}> ready!'.format(_uri))

    def _ros_sub_callback(self, msg):
        data = {}
        rospy.logdebug('ROS received message: %s' % (msg))
        try:
            data = ros_msg_to_dict(msg)
        except Exception as exc:
            rospy.logerr('Serialization exception thrown: {}'.format(str(exc)))
            return
        self._broker_send_msg(data)


    def _init_broker_publisher(self):
        _uri = self._broker_endpoint.uri
        _host = self._broker_endpoint.broker_ref.host
        _port = self._broker_endpoint.broker_ref.port
        _vhost = self._broker_endpoint.broker_ref.vhost

        _username = self._broker_endpoint.auth.username
        _password = self._broker_endpoint.auth.password

        _broker_conn_params = amqp_common.ConnectionParameters(
            host=_host,
            port=_port,
            vhost=_vhost
        )

        _broker_conn_params.credentials = amqp_common.Credentials(
            _username,
            _password
        )

        self._broker_pub = amqp_common.PublisherSync(
            _uri,
            connection_params=_broker_conn_params
        )

        rospy.loginfo('AMQP Publisher <{}> ready!'.format(_uri))

    def _broker_send_msg(self, data):
        if not isinstance(data, dict):
            raise TypeError('Data should be of type dict')
        try:
            self._broker_pub.publish(data)
            # rospy.logdebug('Published message: {}'.format(data))
            print('[*] - Published message: {}'.format(data))
        except Exception as e:
            rospy.logerr('Exception thrown while trying ' +
                         'to publish data to AMQP broker: {}'.format(str(e)))

    def run(self):
        """Start the Bridge"""
        self._init_ros_subscriber()
        self._init_broker_publisher()

        rospy.loginfo('Connector [ROS:{} -> AMQP:{}] ready'.format(
            self._ros_endpoint.uri, self._broker_endpoint.uri))

        while not rospy.is_shutdown():
            rospy.sleep(0.001)

class SubConnector(Connector):
    def __init__(
            self,
            ros_endpoint,
            broker_endpoint,
            *args,
            **kwargs
    ):
        super(SubConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint

    @property
    def ros_endpoint(self):
        return self._ros_endpoint

    @property
    def broker_endpoint(self):
        return self._broker_endpoint