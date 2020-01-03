from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

from concurrent.futures import ThreadPoolExecutor, as_completed

import rospy


class ConnectorThreadExecutor(object):
    def __init__(self, max_workers=50):
        self._max_workers = max_workers
        self._executor = ThreadPoolExecutor(max_workers=self._max_workers)
        self._tasks = {}
        rospy.init_node('BrokerConnectorExecutor')

    def run_connector(self, connector):
        task_future = self._executor.submit(self._submit_task, connector)
        self._store_task(task_future, connector.name)
        return task_future

    def run_forever(self):
        self._executor.shutdown(wait=True)

    def _submit_task(self, connector):
        connector.run()

    def _store_task(self, task, task_id):
        self._tasks['task_id'] = task

