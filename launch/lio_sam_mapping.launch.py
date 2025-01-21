import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import tempfile
import atexit
import sys


def livox_params(common_params, lio_sam_params):
    common_params.update({
        'pointCloudTopic': "/livox/lidar_custom",
        'lidarFrame': "livox_frame",
        'lidarType': "livox",
        'lidarMinRange': 1.0,
        'lidarMaxRange': 40.0
    })

    lio_sam_params.update({
        'N_SCAN': 4,
        'Horizon_SCAN': 6000,
        'timeField': "time",
        'downsampleRate': 1,
        'groundScanInd': 1
    })

def robosense_params(common_params, lio_sam_params):
    common_params.update({
        'pointCloudTopic': "/rslidar_points",
        'lidarFrame': "rslidar",
        'lidarType': "robosense",
        'lidarMinRange': 0.5,
        'lidarMaxRange': 150.0
    })

    lio_sam_params.update({
        'N_SCAN': 32,
        'Horizon_SCAN': 1800,
        'timeField': "time",
        'downsampleRate': 1,
        'groundScanInd': 24
    })

def hesai_params(common_params, lio_sam_params):
    common_params.update({
        'pointCloudTopic': "/points_raw",
        'lidarFrame': "hesai_lidar",
        'lidarType': "hesai",
        'lidarMinRange': 0.5,
        'lidarMaxRange': 150.0
    })

    lio_sam_params.update({
        'N_SCAN': 16,
        'Horizon_SCAN': 1800,
        'timeField': "time",
        'downsampleRate': 1,
        'groundScanInd': 10
    })

def velodyne_params(common_params, lio_sam_params):
    common_params.update({
        'pointCloudTopic': "/velodyne_points",
        'lidarFrame': "velodyne",
        'lidarType': "velodyne",
        'lidarMinRange': 0.5,
        'lidarMaxRange': 150.0,
        'groundScanInd': 10
    })

    lio_sam_params.update({
        'N_SCAN': 16,
        'Horizon_SCAN': 1800,
        'timeField': "time",
        'downsampleRate': 1,
        'groundScanInd': 10
    })

def update_map(common_params, map_name, map_location):
    common_params.update({
        'mapName': map_name,
        'mapLocation': map_location
    })



def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration("map_name").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    lidar_type = LaunchConfiguration("lidar_type").perform(context)
    map_dir = LaunchConfiguration("map_location").perform(context)
    map_location = map_dir + "/" + map_name + "/"
    print(lidar_type)
    mapping_param = LaunchConfiguration("mapping_param").perform(context)
    mapping_param += "/" + robot_type +"_params.yaml"
    # print(lidar_type)
    with open(mapping_param, 'r') as file:
        params = yaml.safe_load(file)
    # Extract only 'common' and 'ig_lio_config'
    modified_params = {
        '/**': {
            'ros__parameters': {
                'common': params['/**']['ros__parameters']['common'],
                'lio_sam_config': params['/**']['ros__parameters']['lio_sam_config']
            }
        }
    }
    # print(modified_params)
    common_params = modified_params['/**']['ros__parameters'].get('common', {})
    lio_sam_params = modified_params['/**']['ros__parameters'].get('lio_sam_config', {})
    update_map(common_params, map_name, map_location)
    if lidar_type == "livox":
        livox_params(common_params, lio_sam_params)
    elif lidar_type == "hesai":
        hesai_params(common_params, lio_sam_params)
    elif lidar_type == "robosense":
        robosense_params(common_params, lio_sam_params)
    elif lidar_type == "velodyne":
        velodyne_params(common_params, lio_sam_params)
    else:
        print("in valid lidar type")
        sys.exit(1)
    map_dir = os.path.join(map_location, map_name)
    print(modified_params)

    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
        yaml.dump(modified_params, temp_file)
        temp_file_path = temp_file.name
    # Cleanup function to remove the temporary file
    def cleanup_temp_file():
        if os.path.exists(temp_file_path):
            os.remove(temp_file_path)
            print(f"Temporary parameter file {temp_file_path} deleted.")

    # Register cleanup function to be called on exit
    atexit.register(cleanup_temp_file)

    # robotodom = Node(
    #     package='lio_sam',
    #     executable='lio_sam_robotOdomForMapping',
    #     parameters=[temp_file_path,{'use_sim_time':LaunchConfiguration("use_sim_time")}],
    #     output='screen'
    #     )
    robotodom = Node(
        package='lio_sam',
        executable='lio_sam_robotOdomForReloc',
        parameters=[temp_file_path,{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
    )
    imu_pre = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        parameters=[temp_file_path,{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
        )

    imagep = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        parameters=[temp_file_path,{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
        )

    featurex = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        parameters=[temp_file_path,{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
        )

    map_opt = Node(
        package='lio_sam',
        executable='lio_sam_mapOptmization',
        parameters=[temp_file_path,{'map_location': map_location},{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
        )

    logrecord= Node(
        package='lio_sam',
        executable='lio_sam_logRecord',
        parameters=[temp_file_path,{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output='screen'
        )
        
  
    return [
        robotodom,
        imu_pre,
        imagep,
        featurex,
        map_opt,
        logrecord
    ]

def generate_launch_description():
 

    mapping_param_arg = DeclareLaunchArgument(
            'mapping_param',
            default_value="/home/wataru/test_ws",
            description='Full path to mapping parameter file to load'
    )

    map_location_dir_arg = DeclareLaunchArgument(
            'map_location',
            default_value="/home/wataru/test_ws",
            description='Full path to map location'
    )
    map_name_arg = DeclareLaunchArgument(
            'map_name',
            default_value="test",
            description='Map name'
    )
    robot_type_arg = DeclareLaunchArgument(
            'robot_type',
            default_value="b2",
            description='robot type'
    )
    lidar_type_arg = DeclareLaunchArgument(
            'lidar_type',
            default_value="robosense",
            description='lidar type'
    )
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value="false",
            description='whether to use use_sim_time'
    )
    
    
    return launch.LaunchDescription([
        mapping_param_arg,
        map_location_dir_arg,
        map_name_arg,
        use_sim_time_arg,
        robot_type_arg,
        lidar_type_arg,
        OpaqueFunction(function=launch_setup)
    ])