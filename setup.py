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

import os
try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages


# Utility function to read the README file.
# Used for the long_description.  It's nice, because now
#   1) we have a top level README file
#   2) it's easier to type in the README file than to put a raw string in below
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    install_requires=read('requirements.txt').splitlines(),
    name='ros2broker',
    version='0.1.0',
    description= 'ROS/ROS2 to AMQP (RabbitMQ broker) protocol connectors',
    url='https://github.com/klpanagi/ros2broker',
    download_url='',
    author='Konstantinos Panayiotou',
    author_email='klpanagi@gmail.com',
    maintainer='Konstantinos Panayiotou',
    maintainer_email='klpanagi@gmail.com',
    license='GNU GPLv3',
    test_suite='tests',
    include_package_data=False,
    # A list naming all the packages you want to include
    packages=find_packages(),
    # Package data to be added to packages
    package_data={},
    # Specify additional files needed by the module distribution:
    #  configuration files, message catalogs, data files
    data_files=[],
    # A script(s) to be installed into standard locations like /usr/bin
    scripts=['bin/ros2broker'],
    console_scripts={
        'console_scripts': [
            'ros2broker = ros2broker.cli:main'
        ]
    },
    zip_safe=True,
    long_description=read('README.md') if os.path.exists('README.md') else ""
)
