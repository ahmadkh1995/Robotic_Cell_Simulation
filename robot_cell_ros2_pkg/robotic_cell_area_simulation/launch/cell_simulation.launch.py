from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotic_cell_area_simulation',
            executable='robotic_cell_simulator.py',
            name='robotic_cell_simulator',
            output='screen',
            parameters=[
                {'wms_server_ip': 'localhost'},
                {'wms_server_port': 5000},
                {'robot_cell_port': 8080},
                {'gui_port': 8081},
                {'success_rate': 0.9}
            ],
            remappings=[]
        ),
        Node(
            package='robotic_cell_area_simulation',
            executable='barcode_scanner.py',
            name='barcode_scanner',
            output='screen',
            parameters=[]
        ),
        Node(
            package='robotic_cell_area_simulation',
            executable='door_handler.py',
            name='door_handler',
            output='screen',
            parameters=[]
        ),
        Node(
            package='robotic_cell_area_simulation',
            executable='emergency_button_handler.py',
            name='emergency_button_handler',
            output='screen',
            parameters=[]
        ),
        Node(
            package='robotic_cell_area_simulation',
            executable='stack_light_handler.py',
            name='stack_light_handler',
            output='screen',
            parameters=[]
        )
    ])
