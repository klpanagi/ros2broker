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

from concurrent.futures import ThreadPoolExecutor, as_completed

# from Queue import Queue
# from threading import Thread, Event

import rospy


class ConnectorThreadExecutor(object):
    def __init__(self, max_workers=50):
        """__init__

        :param max_workers: Maximum number of connector workers
        """
        self._max_workers = max_workers
        self._executor = ThreadPoolExecutor(max_workers=self._max_workers)
        # self._executor = ThreadPool(self._max_workers)
        self._tasks = {}
        rospy.init_node('BrokerConnectorExecutor')

    def run_connector(self, connector):
        """run_connector

        :param connector:
        """
        task = self._executor.submit(self._submit_task, connector)
        # task = self._executor.add_task(self._submit_task, connector)
        self._store_task(task, connector.name)
        return task

    def run_forever(self):
        """run_forever"""
        self._executor.shutdown(wait=True)
        # self._executor.wait_completion()

    def _submit_task(self, connector):
        """_submit_task

        :param connector:
        """
        connector.run()

    def _store_task(self, task, task_id):
        """_store_task

        :param task:
        :param task_id:
        """
        self._tasks['task_id'] = task
