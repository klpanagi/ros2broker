from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

from .broker import BrokerDefinition


class Endpoint(object):

    def __init__(self, uri, name):
        self._uri = uri
        self._name = name

    @property
    def uri(self):
        return self._uri

    @property
    def name(self):
        return self._name


class ROSEndpoint(Endpoint):
    def __init__(self, *args, **kwargs):
        super(ROSEndpoint, self).__init__(*args, **kwargs)


class BrokerEndpoint(Endpoint):
    def __init__(self, exchange, broker_ref, auth, *args, **kwargs):
        self._exchange = exchange
        self._auth = auth
        self._broker_ref = broker_ref
        super(BrokerEndpoint, self).__init__(*args, **kwargs)

    @property
    def exchange(self):
        return self._exchange

    @property
    def exchange(self):
        return self._exchange

    @property
    def auth(self):
        return self._auth

    @property
    def broker_ref(self):
        return self._broker_ref


class ROSServiceEndpoint(ROSEndpoint):
    def __init__(self, srv_type, *args, **kwargs):
        self._srv_type = srv_type
        super(ROSServiceEndpoint, self).__init__(*args, **kwargs)

    @property
    def srv_type(self):
        return self._srv_type


class ROSPubEndpoint(ROSEndpoint):
    def __init__(self, msg_type, queue_size, latch, *args, **kwargs):
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._latch = latch
        super(ROSPubEndpoint, self).__init__(*args, **kwargs)

    @property
    def msg_type(self):
        return self._msg_type

    @property
    def queue_size(self):
        return self._queue_size

    @property
    def latch(self):
        return self._latch


class ROSSubEndpoint(ROSEndpoint):
    def __init__(self, msg_type, *args, **kwargs):
        self._msg_type = msg_type
        super(ROSSubEndpoint, self).__init__(*args, **kwargs)

    @property
    def msg_type(self):
        return self._msg_type


class BrokerRPCEndpoint(BrokerEndpoint):
    def __init__(self, exchange='', *args, **kwargs):
        kwargs['exchange'] = exchange
        super(BrokerRPCEndpoint, self).__init__(*args, **kwargs)


class BrokerPubEndpoint(BrokerEndpoint):
    def __init__(self, exchange='amq.topic', *args, **kwargs):
        kwargs['exchange'] = exchange
        super(BrokerPubEndpoint, self).__init__(*args, **kwargs)


class BrokerSubEndpoint(BrokerEndpoint):
    def __init__(self, exchange='amq.topic', *args, **kwargs):
        kwargs['exchange'] = exchange
        super(BrokerSubEndpoint, self).__init__(*args, **kwargs)
