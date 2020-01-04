from __future__ import (absolute_import, division,
                        print_function, unicode_literals)


class BrokerEndpointAuth(object):
    def __init__(self):
        pass


class BrokerAuthPlain(BrokerEndpointAuth):
    def __init__(self, username, password):
        """__init__

        :param username: The auth username
        :param password: The auth password
        """
        self._username = username
        self._password = password

    @property
    def username(self):
        return self._username

    @property
    def password(self):
        return self._password
