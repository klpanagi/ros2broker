#!/usr/bin/env python

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

from ros2broker import (
    ConnectorThreadExecutor,
    YAMLParser
)


def main():
    model_path = ''
    if len(sys.argv) < 2:
        model_path = 'example_model.yaml'
    else:
        model_path = sys.argv[1]
    c_list = YAMLParser.load(model_path)
    executor = ConnectorThreadExecutor()
    for c in c_list:
        executor.run_connector(c)

    try:
        executor.run_forever()
    except Exception as exc:
        sys.exit(1)


if __name__ == "__main__":
    main()
