# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


MPC_TARGETS = {
    'mpc': ('mpc', 'mpc'),
    'mpc_curv_ls_v1': ('mpc_curv_ls_v1', 'mpc_curv_ls_v1'),
    'mpc_curv_ls_v2': ('mpc_curv_ls_v2', 'mpc_curv_ls_v2'),
}


def launch_setup(context, *args, **kwargs):
    variant = LaunchConfiguration('MPC').perform(context)

    if variant not in MPC_TARGETS:
        available = ', '.join(sorted(MPC_TARGETS.keys()))
        raise RuntimeError(f"Unsupported MPC variant '{variant}'. Available options: {available}")

    package, executable = MPC_TARGETS[variant]

    node = Node(
        package=package,
        executable=executable,
        name='mpc',
        parameters=[LaunchConfiguration('mpc_config')]
    )

    return [node]


def generate_launch_description():
    mpc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mpc.yaml'
    )

    mpc_config_arg = DeclareLaunchArgument(
        'mpc_config',
        default_value=mpc_config,
        description='Descrição para configuração do MPC'
    )

    variant_arg = DeclareLaunchArgument(
        'MPC',
        default_value='mpc_curv_ls_v1',
        description='Selecione qual pacote/executável MPC lançar'
    )

    return LaunchDescription([
        mpc_config_arg,
        variant_arg,
        OpaqueFunction(function=launch_setup),
    ])
