from __future__ import (
    absolute_import,
    division,
    print_function,
    unicode_literals
)

from concurrent.futures import ThreadPoolExecutor, as_completed

import rospy


class ConnectorThreadExecutor(object):
    def __init__(self, max_workers=50):
        """__init__

        :param max_workers: Maximum number of connector workers
        """
        self._max_workers = max_workers
        self._executor = ThreadPoolExecutor(max_workers=self._max_workers)
        self._tasks = {}
        rospy.init_node('BrokerConnectorExecutor')

    def run_connector(self, connector):
        """run_connector

        :param connector:
        """
        task_future = self._executor.submit(self._submit_task, connector)
        self._store_task(task_future, connector.name)
        return task_future

    def run_forever(self):
        """run_forever"""
        self._executor.shutdown(wait=True)

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

