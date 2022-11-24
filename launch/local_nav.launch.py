import launch
import launch_ros
import os


def generate_launch_description(): 
   launch_speed_limit = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='set_speed_limit.py',
      name='set_speed_limit')
       
   enforce_speed_limit = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='enforce_speed_limit.py',
      name='enforce_speed_limit')

   #cpp_executable = launch_ros.actions.Node(
   #   package='custom_nav_stack_pkg',
   #   exectuable='cpp_executable2',
   #   name='cpp_executable2'
   #)

   #client_python = launch_ros.actions.Node(
   #   package='nav2_tutorial',
   #   executable='client_python.py',
   #   name='client_python'
   #)

   approach_speed_controller = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='approach_speed_controller.py',
      name='approach_speed_controller'
   )

   get_penguin_positions = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='get_penguin_positions.py',
      name='get_obstacle_positions'
   )

   goal_pose_from_penguins = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='penguin_to_goal_pose.py',
      name='penguin_to_goal_pose'
   )
                             
   return launch.LaunchDescription([
        #launch_speed_limit,
        #enforce_speed_limit,  
        #cpp_executable,
        #client_python,  
        approach_speed_controller,
        get_penguin_positions,
        goal_pose_from_penguins
    ])

