#!/usr/bin/env python3

"""
Nav2 Integration Test Script
Tests the nav2 configuration and integration with Hector SLAM
"""

import yaml
import os
import sys
from pathlib import Path

def test_nav2_params():
    """Test nav2 parameters file validity"""
    print("Testing nav2 parameters configuration...")

    params_file = Path("config/nav2_params.yaml")
    if not params_file.exists():
        print(f"❌ ERROR: {params_file} not found!")
        return False

    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)

        # Check required nav2 nodes
        required_nodes = [
            'map_server', 'amcl', 'bt_navigator', 'controller_server',
            'local_costmap', 'global_costmap', 'planner_server'
        ]

        missing_nodes = []
        for node in required_nodes:
            if node not in params:
                missing_nodes.append(node)

        if missing_nodes:
            print(f"❌ ERROR: Missing nav2 node configurations: {missing_nodes}")
            return False

        print("✅ Nav2 parameters file is valid")

        # Check critical parameters
        print("\nChecking critical nav2 parameters:")

        # AMCL parameters
        if 'global_frame_id' in params['amcl']['ros__parameters']:
            frame_id = params['amcl']['ros__parameters']['global_frame_id']
            print(f"  - AMCL global frame: {frame_id}")

        # Costmap parameters
        if 'global_frame' in params['global_costmap']['global_costmap']['ros__parameters']:
            global_frame = params['global_costmap']['global_costmap']['ros__parameters']['global_frame']
            print(f"  - Global costmap frame: {global_frame}")

        # Check if using map topic (important for SLAM integration)
        use_map_topic = params['global_costmap']['global_costmap']['ros__parameters'].get('use_map_topic', False)
        print(f"  - Using map topic: {use_map_topic}")

        if not use_map_topic:
            print("  ⚠️  WARNING: Global costmap should use map topic for SLAM integration")

        return True

    except yaml.YAMLError as e:
        print(f"❌ ERROR: Invalid YAML in nav2_params.yaml: {e}")
        return False
    except Exception as e:
        print(f"❌ ERROR: Failed to read nav2_params.yaml: {e}")
        return False

def test_launch_file():
    """Test launch file syntax"""
    print("\nTesting launch file configuration...")

    launch_file = Path("launch/turtlebot3_hector_slam_launch.py")
    if not launch_file.exists():
        print(f"❌ ERROR: {launch_file} not found!")
        return False

    try:
        # Check if file can be compiled
        with open(launch_file, 'r') as f:
            content = f.read()

        compile(content, str(launch_file), 'exec')
        print("✅ Launch file syntax is valid")

        # Check for nav2 integration
        if 'nav2_bringup' in content:
            print("✅ Nav2 bringup integration found")
        else:
            print("❌ ERROR: Nav2 bringup not found in launch file")
            return False

        # Check for timer delay
        if 'TimerAction' in content:
            print("✅ Timer delay for SLAM initialization found")
        else:
            print("⚠️  WARNING: No timer delay found for SLAM initialization")

        return True

    except SyntaxError as e:
        print(f"❌ ERROR: Syntax error in launch file: {e}")
        return False
    except Exception as e:
        print(f"❌ ERROR: Failed to validate launch file: {e}")
        return False

def test_dependencies():
    """Test if required ROS packages are available"""
    print("\nTesting ROS package dependencies...")

    required_packages = [
        'nav2_bringup',
        'nav2_controller',
        'nav2_planner',
        'nav2_bt_navigator',
        'turtlebot3_bringup',
        'rviz2'
    ]

    import subprocess

    missing_packages = []
    for package in required_packages:
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True, text=True, check=True
            )
            if package not in result.stdout:
                missing_packages.append(package)
            else:
                print(f"  ✅ {package}")
        except subprocess.CalledProcessError:
            print("❌ ERROR: Failed to check ROS packages")
            return False

    if missing_packages:
        print(f"❌ ERROR: Missing required packages: {missing_packages}")
        return False

    print("✅ All required ROS packages are available")
    return True

def main():
    """Run all tests"""
    print("🤖 Nav2 Integration Test Suite")
    print("=" * 40)

    # Change to workspace directory
    os.chdir('/home/owner/hector_ws')

    tests = [
        ("Nav2 Parameters", test_nav2_params),
        ("Launch File", test_launch_file),
        ("ROS Dependencies", test_dependencies)
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n🔍 Running {test_name} test...")
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ ERROR: {test_name} test failed with exception: {e}")
            results.append((test_name, False))

    print("\n" + "=" * 40)
    print("📊 Test Results:")
    print("=" * 40)

    passed = 0
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{len(results)} tests passed")

    if passed == len(results):
        print("🎉 All tests passed! Nav2 integration is ready for testing.")
        return 0
    else:
        print("⚠️  Some tests failed. Please address the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())