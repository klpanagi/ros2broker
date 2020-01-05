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
import traceback
import rospy
import amqp_common
import json

from rosconversions import (
    ros_msg_to_dict,
    get_message_class,
    get_service_class,
    dict_to_ros_msg,
    dict_to_ros_srv_request,
    ros_srv_resp_to_dict
)

from .endpoint import *


class Connector(object):
    def __init__(self, name=None, debug=False):
        """__init__

        :param name: The name of the connector / Optional
        :param debug: Either or not to enable debug console output
        """
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
        """__init__

        :param ros_endpoint: ROSEndpoint instance
        :param broker_endpoint: BrokerEndpoint instance
        :param *args:
        :param **kwargs:
        """
        super(RPCConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint

    def _init_rpc_server(self):
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

        self._broker_rpc_server = amqp_common.RpcServer(
            _uri,
            on_request=self._broker_rpc_callback,
            connection_params=_broker_conn_params,
            debug=True
        )

    def _init_ros_service_client(self):
        _uri = self.ros_endpoint.uri
        _node_name = self.ros_endpoint.name
        _srv_type = self.ros_endpoint.srv_type

        _srv_pkg = _srv_type.split('/')[0]
        _srv_cls  = _srv_type.split('/')[1]

        srv_cls = get_service_class(_srv_type)

        if srv_cls is None:
            raise ValueError('Given Srv type ({}) does not exist'.format(
                _srv_type))

        rospy.loginfo('Waiting for ROS Service {} ...'.format(_uri))
        # rospy.wait_for_service(_uri)

        self.ros_srv = rospy.ServiceProxy(
            _uri,
            srv_cls
        )
        rospy.loginfo('ROS Service Client [{}] ready'.format(_uri))

    def _broker_rpc_callback(self, msg, meta):
        """."""
        resp = {}
        try:
            srv_cls = get_service_class(self.ros_endpoint.srv_type)
            srv_req = dict_to_ros_srv_request(self.ros_endpoint.srv_type, msg)
            rospy.loginfo('Calling ROS Service <%s>' % (self.ros_endpoint.uri))
            resp = self.ros_srv(srv_req)
        except rospy.ServiceException as exc:
            rospy.logerr('ROS Service call failed: {}'.format(exc))
            resp = None
        except Exception as exc:
            print(traceback.format_exc())
            resp = None
        if resp is None:
            resp = srv_cls._response_class()
        data = ros_srv_resp_to_dict(resp)
        return data

    def run(self):
        """Start the bridge."""
        self._init_rpc_server()
        self._broker_rpc_server.run_threaded()
        self._init_ros_service_client()

        rospy.loginfo('Connector [AMQP:{} -> ROS:{}] ready'.format(
            self._broker_endpoint.uri, self._ros_endpoint.uri))

        while not rospy.is_shutdown():
            rospy.sleep(0.001)


class PubConnector(Connector):
    def __init__(
        self,
        ros_endpoint,
        broker_endpoint,
        *args,
        **kwargs
    ):
        """__init__

        :param ros_endpoint: ROSEndpoint instance
        :param broker_endpoint: BrokerEndpoint instance
        :param *args:
        :param **kwargs:
        """
        super(PubConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint

    def _init_ros_subscriber(self):
        """_init_ros_subscriber"""
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
        """_ros_sub_callback

        :param msg: Message received from ROS Topic
        """
        data = {}
        rospy.logdebug('ROS received message: %s' % (msg))
        try:
            data = ros_msg_to_dict(msg)
        except Exception as exc:
            rospy.logerr('Serialization exception thrown: {}'.format(str(exc)))
            return
        self._broker_send_msg(data)

    def _init_broker_publisher(self):
        """_init_broker_publisher"""
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
        """_broker_send_msg

        :param data: Data to send to the broker / Dictionary
        """
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
        """__init__

        :param ros_endpoint: ROSEndpoint instance
        :param broker_endpoint: BrokerEndpoint instance
        :param *args:
        :param **kwargs:
        """
        super(SubConnector, self).__init__(*args, **kwargs)
        self._ros_endpoint = ros_endpoint
        self._broker_endpoint = broker_endpoint

    @property
    def ros_endpoint(self):
        return self._ros_endpoint

    @property
    def broker_endpoint(self):
        return self._broker_endpoint

    def _init_broker_subscriber(self):
        """_init_broker_subscriber"""
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

        self._broker_sub = amqp_common.SubscriberSync(
            _uri,
            on_message=self._broker_sub_callback,
            connection_params=_broker_conn_params,
            queue_size=10
        )
        rospy.loginfo('AMQP Subscriber <{}> ready!'.format(_uri))
        self._broker_sub.run_threaded()

    def _broker_sub_callback(self, msg, meta):
        """_broker_sub_callback

        :param msg: The input message / Dictionary
        :param meta: Metainformation passed to client callback for advanced
            usage.
        """
        try:
            ros_msg = dict_to_ros_msg(self.ros_endpoint.msg_type, msg)
            rospy.loginfo('Sending message to ROS: {}'.format(ros_msg))
            self._ros_pub.publish(ros_msg)
        except Exception as exc:
            rospy.loginfo('Could not convert input message [{}]' + \
                          ' to {{ rossub.topic.msgType }}'.format(msg))

    def _init_ros_publisher(self):
        """_init_ros_publisher"""
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

        self._ros_pub = rospy.Publisher(
            _uri,
            msg_cls,
            queue_size=10
        )
        rospy.loginfo('ROS Publisher <{}> ready!'.format(_uri))

    def run(self):
        """Start the Bridge"""
        self._init_broker_subscriber()
        self._init_ros_publisher()

        rospy.loginfo('Connector [AMQP:{} -> ROS:{}] ready'.format(
            self._broker_endpoint.uri, self._ros_endpoint.uri))

        while not rospy.is_shutdown():
            rospy.sleep(0.001)
