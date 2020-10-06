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

from os.path import dirname, join, exists, isdir
from os import getcwd, chmod

import yaml

from jinja2 import Environment, PackageLoader, FileSystemLoader


class BridgeGenerator(object):
    def __init__(self):
        self._env = Environment(
            loader=FileSystemLoader(join(dirname(__file__), 'gen_tpls'),
                                    followlinks=True
                                    )
        )

        self._tpl = self._env.get_template('bridge.py')

    def gen_from_yaml(
        self,
        model_path,
        dest_path=None,
        license=None,  # Does not yet suppot adding license
        python2=False,
        python3=True
    ):
        if dest_path is None:
            dest_path = getcwd()
        with open(model_path, 'r') as f:
            _model = yaml.load(f, Loader=yaml.Loader)
            _info = _model['info']
            _connectors = _model['connector']
            _broker = _model['broker'][0]

            _fdest = join(dest_path, '{}_bridges.py'.format(_info['name']))

            _python = '2' if python2 else '3'

            _subC = [c for c in _connectors if c['type'] == 'sub']
            _pubC = [c for c in _connectors if c['type'] == 'pub']
            _rpcC = [c for c in _connectors if c['type'] == 'rpc']

            _source = self._tpl.render(
                python_version=_python,
                broker=_broker,
                sub_connectors=_subC,
                pub_connectors=_pubC,
                rpc_connectors=_rpcC,
            )
            with open(_fdest, 'w') as f:
                f.write(_source)
            # Give execution permissions to the generated python file
            chmod(_fdest, 509)

    def gen_from_dict(self):
        raise NotImplementedError()
