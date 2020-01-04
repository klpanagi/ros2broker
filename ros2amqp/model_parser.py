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

import yaml
from pprint import pprint


from .connector import (
    SubConnector,
    PubConnector,
    RPCConnector
)

from .endpoint import (
    ROSPubEndpoint, ROSSubEndpoint, ROSServiceEndpoint,
    BrokerPubEndpoint, BrokerSubEndpoint, BrokerRPCEndpoint,
)

from .broker import BrokerDefinition

from .auth import BrokerAuthPlain


class ModelParser(object):
    @staticmethod
    def load_from_file(fpath):
        raise NotImplementedError()


class YAMLParser(ModelParser):
    @staticmethod
    def load(fpath):
        c_list = []
        with open(fpath, 'r') as f:
            model = yaml.load(f, Loader=yaml.FullLoader)
            connectors = model['connector']
            broker = model['broker']
            for c in connectors:
                c_name = c['name']
                c_type = c['type']
                ros_ep = c['ros_endpoint']
                broker_ep = c['broker_endpoint']

                print('[*] - Found Connector Definition {} of type {}'.format(
                    c_name, c_type))

                if c_type == 'pub':
                    conn = YAMLParser._create_pub_connector(ros_ep,
                                                            broker_ep,
                                                            broker[0]
                                                            )
                    c_list.append(conn)
                elif c_type == 'sub':
                    conn = YAMLParser._create_sub_connector(ros_ep,
                                                            broker_ep,
                                                            broker[0]
                                                            )
                    c_list.append(conn)
                elif c_type == 'service' or c_type == 'rpc':
                    conn = YAMLParser._create_rpc_connector(ros_ep,
                                                            broker_ep,
                                                            broker[0]
                                                            )
                    c_list.append(conn)
                else:
                    raise ValueError(
                        '\'type\' property should be one' + \
                        'of [pub, sub, rpc/service]'
                    )
            return c_list

    @staticmethod
    def _create_pub_connector(ros_ep, broker_ep, broker):
        broker = BrokerDefinition(
            name=None,
            host=broker['host'],
            port=broker['port'],
            vhost=broker['vhost'],
            global_ns=broker['global_ns']
        )

        ros_ep = ROSPubEndpoint(
            msg_type=ros_ep['msg_type'],
            uri=ros_ep['uri'],
            name=ros_ep['name']
        )

        broker_ep = BrokerPubEndpoint(
            uri=broker_ep['uri'],
            name=broker_ep['name'],
            broker_ref=broker,
            auth=BrokerAuthPlain(username='bot', password='b0t')
        )

        pc = PubConnector(ros_ep, broker_ep)
        return pc

    @staticmethod
    def _create_sub_connector(ros_ep, broker_ep, broker):
        broker = BrokerDefinition(
            name=None,
            host=broker['host'],
            port=broker['port'],
            vhost=broker['vhost'],
            global_ns=broker['global_ns']
        )

        ros_ep = ROSSubEndpoint(
            msg_type=ros_ep['msg_type'],
            uri=ros_ep['uri'],
            name=ros_ep['name']
        )

        broker_ep = BrokerSubEndpoint(
            uri=broker_ep['uri'],
            name=broker_ep['name'],
            broker_ref=broker,
            auth=BrokerAuthPlain(username='bot', password='b0t')
        )

        sc = SubConnector(ros_ep, broker_ep)
        return sc

    @staticmethod
    def _create_rpc_connector(ros_ep, broker_ep, broker):
        broker = BrokerDefinition(
            name=None,
            host=broker['host'],
            port=broker['port'],
            vhost=broker['vhost'],
            global_ns=broker['global_ns']
        )

        ros_ep = ROSServiceEndpoint(
            srv_type=ros_ep['srv_type'],
            uri=ros_ep['uri'],
            name=ros_ep['name']
        )

        broker_ep = BrokerRPCEndpoint(
            uri=broker_ep['uri'],
            name=broker_ep['name'],
            broker_ref=broker,
            auth=BrokerAuthPlain(username='bot', password='b0t')
        )

        rc = RPCConnector(ros_ep, broker_ep)
        return rc
