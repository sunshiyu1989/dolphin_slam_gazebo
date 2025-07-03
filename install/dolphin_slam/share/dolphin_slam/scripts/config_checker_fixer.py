#!/usr/bin/env python3
"""
Dolphin SLAM 配置检查和修复工具
快速诊断并修复配置问题
"""

import os
import yaml
import sys

def main():
    print("🔍 Dolphin SLAM 配置诊断工具")
    print("=" * 40)
    
    # 路径定义
    workspace_dir = os.path.expanduser("~/dolphin_slam_ws")
    config_file = os.path.join(workspace_dir, "src/dolphin_slam/config/dolphin_slam_params.yaml")
    
    print(f"📂 检查配置文件: {config_file}")
    
    if not os.path.exists(config_file):
        print(f"❌ 配置文件不存在: {config_file}")
        return False
    
    # 读取和分析配置文件
    try:
        with open(config_file, 'r') as f:
            content = f.read()
            
        print("📄 当前配置文件内容:")
        print("-" * 30)
        print(content[:500] + "..." if len(content) > 500 else content)
        print("-" * 30)
        
        # 尝试解析YAML
        config = yaml.safe_load(content)
        
        # 检查是否是正确的ROS2格式
        print("\n🔍 格式检查:")
        
        expected_nodes = [
            'robot_state_node',
            'image_processing_node', 
            'place_cell_node',
            'experience_map_node',
            'local_view_node'
        ]
        
        format_errors = []
        
        for node in expected_nodes:
            if node in config:
                if 'ros__parameters' in config[node]:
                    print(f"✅ {node}: 格式正确")
                else:
                    print(f"❌ {node}: 缺少 ros__parameters")
                    format_errors.append(node)
            else:
                print(f"❌ {node}: 节点配置缺失")
                format_errors.append(node)
        
        # 检查关键路径
        print("\n📁 路径检查:")
        nav_file = "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/navigation/navigation.csv"
        
        if os.path.exists(nav_file):
            print(f"✅ 导航文件存在: {nav_file}")
            with open(nav_file, 'r') as f:
                lines = f.readlines()
            print(f"   记录数: {len(lines)} 行")
        else:
            print(f"❌ 导航文件不存在: {nav_file}")
            
        # 如果有格式错误，提供修复选项
        if format_errors:
            print(f"\n🛠️ 发现 {len(format_errors)} 个格式错误")
            
            response = input("是否要自动修复配置文件? (y/N): ")
            
            if response.lower() == 'y':
                fix_config_file(config_file)
                return True
            else:
                print("请手动修复配置文件格式")
                return False
        else:
            print("\n✅ 配置文件格式正确")
            
            # 检查参数值
            if 'robot_state_node' in config and 'ros__parameters' in config['robot_state_node']:
                nav_csv = config['robot_state_node']['ros__parameters'].get('navigation_csv', '')
                print(f"\n📋 配置的导航文件路径: {nav_csv}")
                
                if nav_csv and nav_csv != nav_file:
                    print("⚠️  配置的路径与预期不符")
                    response = input("是否更新为正确路径? (y/N): ")
                    if response.lower() == 'y':
                        update_navigation_path(config_file, nav_file)
                        return True
                elif not nav_csv:
                    print("❌ 导航路径未配置")
                    response = input("是否设置正确路径? (y/N): ")
                    if response.lower() == 'y':
                        update_navigation_path(config_file, nav_file)
                        return True
            
        return True
        
    except Exception as e:
        print(f"❌ 读取配置文件失败: {e}")
        return False

def fix_config_file(config_file):
    """修复配置文件为正确的ROS2格式"""
    print("🔧 开始修复配置文件...")
    
    # 备份原文件
    backup_file = config_file + ".backup"
    os.rename(config_file, backup_file)
    print(f"💾 原文件已备份为: {backup_file}")
    
    # 创建正确格式的配置文件
    correct_config = """# Dolphin SLAM ROS2 配置文件 (自动修复版)

robot_state_node:
  ros__parameters:
    navigation_csv: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/navigation/navigation.csv"
    dvl_topic: "/dvl/data"
    imu_topic: "/imu/data"
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    dvl_position:
      x: 0.75
      y: 0.0
      z: -0.4
    dvl_orientation:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
    use_ekf: true
    process_noise_std: 0.1
    measurement_noise_std: 0.05
    publish_tf: true
    publish_rate: 20.0

image_processing_node:
  ros__parameters:
    camera_path: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/camera"
    sonar_path: "/media/psf/Samsung T7/SLAM Data/Sunboat_03-09-2023/2023-09-03-07-58-37/sonar"
    feature_type: "SIFT"
    max_features: 500
    surf_hessian_threshold: 400
    surf_upright: false
    process_every_n_frames: 2
    enable_visualization: true
    camera_topic: "/camera/image_raw"
    sonar_topic: "/sonar/image_raw"
    descriptors_topic: "/features/descriptors"
    keypoints_topic: "/features/keypoints"

local_view_node:
  ros__parameters:
    matching_algorithm: "fabmap"
    similarity_threshold: 0.65
    vocabulary_size: 500
    clustering_algorithm: "kmeans"
    max_templates: 2000
    template_decay_rate: 0.995
    descriptors_topic: "/features/descriptors"
    matches_topic: "/local_view/matches"

place_cell_node:
  ros__parameters:
    neurons_per_dimension: 16
    neurons_step: 0.2
    recurrent_connection_std: 3.0
    weight_function: "mexican_hat"
    input_learning_rate: 0.1
    min_input_age: 10
    global_inhibition: 0.001
    activation_threshold: 0.1
    odometry_topic: "/robot/odometry"
    visual_match_topic: "/local_view/matches"
    activity_topic: "/place_cells/activity"

experience_map_node:
  ros__parameters:
    match_threshold: 0.75
    lv_factor: 0.5
    pc_factor: 0.5
    min_experience_age: 5
    loop_closure_threshold: 0.8
    max_loop_distance: 5.0
    map_pruning_enabled: true
    max_experiences: 5000
    place_cell_topic: "/place_cells/activity"
    local_view_topic: "/local_view/matches"
    experience_topic: "/experience_map/experiences"
    loop_closure_topic: "/experience_map/loop_closures"
    odometry_topic: "/robot/odometry"
"""
    
    with open(config_file, 'w') as f:
        f.write(correct_config)
    
    print("✅ 配置文件已修复为正确的ROS2格式")

def update_navigation_path(config_file, nav_file):
    """更新导航文件路径"""
    print("📝 更新导航文件路径...")
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    if 'robot_state_node' not in config:
        config['robot_state_node'] = {}
    if 'ros__parameters' not in config['robot_state_node']:
        config['robot_state_node']['ros__parameters'] = {}
    
    config['robot_state_node']['ros__parameters']['navigation_csv'] = nav_file
    
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
    
    print(f"✅ 导航路径已更新为: {nav_file}")

if __name__ == "__main__":
    success = main()
    
    if success:
        print("\n🎉 诊断完成！")
        print("\n下一步:")
        print("1. cd ~/dolphin_slam_ws")
        print("2. colcon build --packages-select dolphin_slam --symlink-install")
        print("3. source install/setup.bash")
        print("4. export ROS_DOMAIN_ID=42")
        print("5. ros2 launch dolphin_slam dolphin_slam_launch.py")
    else:
        print("\n❌ 诊断失败，请检查错误信息")
        sys.exit(1)
