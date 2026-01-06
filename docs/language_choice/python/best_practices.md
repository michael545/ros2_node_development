# Python Best Practices for ROS2

## Introduction

Python is widely used in ROS2 for its rapid development capabilities, extensive libraries, and ease of use. However, to build professional, production-ready ROS2 applications in Python, you need to follow specific best practices that address Python's unique characteristics in the ROS2 ecosystem.

## Table of Contents

1. [Performance Optimization](#performance-optimization)
2. [Memory Management](#memory-management)
3. [Concurrency Patterns](#concurrency-patterns)
4. [Error Handling](#error-handling)
5. [Type Safety](#type-safety)
6. [Testing Strategies](#testing-strategies)
7. [Code Organization](#code-organization)
8. [Real-World Examples](#real-world-examples)

## Performance Optimization

### 1. Use NumPy for Numerical Computations

**Problem:** Pure Python loops are slow for numerical computations.

**Solution:** Use NumPy for array operations and mathematical computations.

```python
import numpy as np
import rclpy
from rclpy.node import Node

class FastSensorProcessor(Node):
    def __init__(self):
        super().__init__('fast_sensor_processor')
        self.subscription = self.create_subscription(
            PointCloud2, 'points', self.process_point_cloud, 10)
        
        # Pre-allocate NumPy arrays for performance
        self.buffer = np.zeros((10000, 3), dtype=np.float32)
    
    def process_point_cloud(self, msg):
        # Convert ROS message to NumPy array (efficient conversion)
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        
        # Vectorized operations (much faster than Python loops)
        filtered = points[np.linalg.norm(points, axis=1) < 5.0]  # Range filtering
        transformed = filtered @ self.rotation_matrix  # Matrix multiplication
        
        # Further processing...
```

### 2. Avoid Global Interpreter Lock (GIL) Bottlenecks

**Problem:** Python's GIL can limit multi-core utilization.

**Solution:** Use multiprocessing for CPU-bound tasks.

```python
import multiprocessing
import rclpy
from rclpy.node import Node

class ParallelProcessor(Node):
    def __init__(self):
        super().__init__('parallel_processor')
        self.pool = multiprocessing.Pool(processes=4)
        
    def process_data_parallel(self, data_chunks):
        # Use multiprocessing to bypass GIL
        results = self.pool.map(self._process_chunk, data_chunks)
        return results
    
    def _process_chunk(self, chunk):
        # CPU-intensive processing
        return process_chunk_algorithm(chunk)
    
    def cleanup(self):
        self.pool.close()
        self.pool.join()
```

### 3. Use Cython for Performance-Critical Code

**Problem:** Some algorithms are too slow in pure Python.

**Solution:** Use Cython to compile Python to C.

```python
# sensor_processor.pyx (Cython file)
import numpy as np
cimport numpy as np
cimport cython

@cython.boundscheck(False)
@cython.wraparound(False)
def fast_point_cloud_filter(np.ndarray[np.float32_t, ndim=2] points, float threshold):
    cdef int i
    cdef int count = 0
    cdef np.ndarray[np.float32_t, ndim=2] result = np.empty((points.shape[0], 3), dtype=np.float32)
    
    for i in range(points.shape[0]):
        if points[i, 0]**2 + points[i, 1]**2 + points[i, 2]**2 < threshold**2:
            result[count] = points[i]
            count += 1
    
    return result[:count]
```

## Memory Management

### 1. Avoid Memory Leaks in Callbacks

**Problem:** Long-running nodes can accumulate memory if not careful.

**Solution:** Use weak references and proper cleanup.

```python
import weakref
import rclpy
from rclpy.node import Node

class MemorySafeNode(Node):
    def __init__(self):
        super().__init__('memory_safe_node')
        self.data_cache = weakref.WeakValueDictionary()
        
        # Use timer for periodic cleanup
        self.cleanup_timer = self.create_timer(60.0, self.cleanup_cache)
    
    def process_data(self, msg):
        # Store data with weak reference
        cache_key = msg.header.stamp.nanoseconds
        processed = self._expensive_processing(msg)
        self.data_cache[cache_key] = processed
        
        return processed
    
    def cleanup_cache(self):
        # WeakValueDictionary automatically removes unreferenced items
        current_keys = list(self.data_cache.keys())
        if len(current_keys) > 1000:  # Keep cache size reasonable
            for key in current_keys[:-500]:  # Keep only recent 500
                if key in self.data_cache:
                    del self.data_cache[key]
```

### 2. Use Generators for Large Data Streams

**Problem:** Processing large datasets can consume excessive memory.

**Solution:** Use generators for streaming processing.

```python
def process_large_dataset(dataset):
    """Generator that processes data in chunks"""
    chunk_size = 1000
    for i in range(0, len(dataset), chunk_size):
        chunk = dataset[i:i + chunk_size]
        yield process_chunk(chunk)

class StreamingProcessor(Node):
    def __init__(self):
        super().__init__('streaming_processor')
        self.subscription = self.create_subscription(
            LargeDataset, 'large_data', self.handle_large_data, 10)
    
    def handle_large_data(self, msg):
        dataset = convert_message_to_dataset(msg)
        
        # Process in streaming fashion
        for result in process_large_dataset(dataset):
            self.publish_result(result)
            
        # Memory is automatically freed as we go
```

## Concurrency Patterns

### 1. Use Async/Await for I/O-bound Operations

**Problem:** Blocking I/O operations can freeze the ROS2 event loop.

**Solution:** Use async/await for non-blocking I/O.

```python
import asyncio
import aiohttp
import rclpy
from rclpy.node import Node

class AsyncDataFetcher(Node):
    def __init__(self):
        super().__init__('async_data_fetcher')
        self.session = aiohttp.ClientSession()
        
        # Create background task
        self.background_task = asyncio.create_task(self.fetch_data_periodically())
    
    async def fetch_data_periodically(self):
        while rclpy.ok():
            try:
                data = await self.fetch_from_api()
                self.process_fetched_data(data)
            except Exception as e:
                self.get_logger().error(f"Fetch failed: {e}")
            
            await asyncio.sleep(60.0)  # Wait before next fetch
    
    async def fetch_from_api(self):
        async with self.session.get('https://api.example.com/data') as response:
            return await response.json()
    
    def cleanup(self):
        self.background_task.cancel()
        asyncio.run(self.session.close())
```

### 2. Multi-threaded Executor for Mixed Workloads

**Problem:** Need to handle both real-time and background tasks.

**Solution:** Use multi-threaded executor with proper task separation.

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class MixedWorkloadNode(Node):
    def __init__(self):
        super().__init__('mixed_workload_node')
        
        # Real-time callback group (mutually exclusive)
        self.realtime_group = MutuallyExclusiveCallbackGroup()
        
        # Background callback group (can run in parallel)
        self.background_group = ReentrantCallbackGroup()
        
        # Real-time subscription
        self.realtime_sub = self.create_subscription(
            SensorData, 'sensor_data', self.realtime_callback, 10,
            callback_group=self.realtime_group)
        
        # Background subscription
        self.background_sub = self.create_subscription(
            BackgroundTask, 'background_task', self.background_callback, 10,
            callback_group=self.background_group)
    
    def realtime_callback(self, msg):
        # This will run with mutual exclusion
        self.process_sensor_data(msg)
    
    def background_callback(self, msg):
        # This can run in parallel with other background tasks
        self.process_background_task(msg)

def main():
    rclpy.init()
    node = MixedWorkloadNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
```

## Error Handling

### 1. Comprehensive Exception Handling

**Problem:** Unhandled exceptions can crash nodes.

**Solution:** Implement robust exception handling at all levels.

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.subscription = self.create_subscription(
            SensorData, 'sensor_data', self.safe_callback, 10)
    
    def safe_callback(self, msg):
        try:
            self.process_data(msg)
        except ValueError as e:
            self.get_logger().warn(f"Data validation error: {e}")
            self.handle_bad_data(msg)
        except RuntimeError as e:
            self.get_logger().error(f"Runtime error: {e}")
            self.request_recovery()
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}", exc_info=True)
            self.enter_safe_state()
    
    def process_data(self, msg):
        # Main processing logic with validation
        if not self.validate_message(msg):
            raise ValueError("Invalid sensor data")
        
        # Process data...
```

### 2. Graceful Degradation

**Problem:** Partial failures should not crash the entire system.

**Solution:** Implement graceful degradation strategies.

```python
class FaultTolerantNode(Node):
    def __init__(self):
        super().__init__('fault_tolerant_node')
        self.primary_sensor = None
        self.backup_sensor = None
        self.degraded_mode = False
    
    def on_primary_sensor_failure(self):
        self.get_logger().warn("Primary sensor failed, switching to backup")
        self.degraded_mode = True
        # Adjust processing parameters for degraded mode
        self.adjust_for_degraded_mode()
    
    def on_recovery(self):
        self.get_logger().info("System recovered, resuming normal operation")
        self.degraded_mode = False
        self.restore_normal_parameters()
    
    def process_with_fallback(self, primary_data, backup_data):
        try:
            return self.process_primary(primary_data)
        except Exception as e:
            self.get_logger().warn(f"Primary processing failed: {e}")
            if backup_data is not None:
                return self.process_backup(backup_data)
            else:
                return self.get_fallback_result()
```

## Type Safety

### 1. Use Type Hints

**Problem:** Python's dynamic typing can lead to runtime errors.

**Solution:** Use type hints for better code clarity and IDE support.

```python
from typing import List, Dict, Optional, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class TypedProcessor(Node):
    def __init__(self):
        super().__init__('typed_processor')
        self.point_clouds: Dict[str, PointCloud2] = {}
        self.config: Dict[str, float] = self.load_config()
    
    def load_config(self) -> Dict[str, float]:
        """Load configuration with type hints"""
        return {
            'max_range': 10.0,
            'min_intensity': 0.1,
            'cluster_size': 5
        }
    
    def process_point_cloud(self, msg: PointCloud2) -> Optional[List[Tuple[float, float, float]]]:
        """Process point cloud with type hints"""
        try:
            points = self._extract_points(msg)
            filtered = self._filter_points(points)
            return filtered
        except Exception as e:
            self.get_logger().error(f"Processing failed: {e}")
            return None
    
    def _extract_points(self, msg: PointCloud2) -> List[Tuple[float, float, float]]:
        """Extract points from PointCloud2 message"""
        # Implementation...
        return []
    
    def _filter_points(self, points: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """Filter points based on configuration"""
        max_range = self.config['max_range']
        return [p for p in points if self._calculate_distance(p) < max_range]
    
    def _calculate_distance(self, point: Tuple[float, float, float]) -> float:
        """Calculate Euclidean distance"""
        return (point[0]**2 + point[1]**2 + point[2]**2)**0.5
```

### 2. Runtime Type Checking

**Problem:** Need to validate types at runtime for critical operations.

**Solution:** Use runtime type checking libraries.

```python
from pydantic import BaseModel, ValidationError, confloat
import rclpy
from rclpy.node import Node

class SensorConfig(BaseModel):
    max_range: confloat(gt=0, le=100) = 10.0
    min_intensity: confloat(ge=0, le=1) = 0.1
    update_rate: confloat(gt=0, le=1000) = 10.0

class ValidatedNode(Node):
    def __init__(self):
        super().__init__('validated_node')
        
        # Declare parameters with validation
        self.declare_parameter('config.max_range', 10.0)
        self.declare_parameter('config.min_intensity', 0.1)
        self.declare_parameter('config.update_rate', 10.0)
        
        # Load and validate configuration
        self.config = self.load_validated_config()
    
    def load_validated_config(self) -> SensorConfig:
        """Load configuration with runtime validation"""
        try:
            config_data = {
                'max_range': self.get_parameter('config.max_range').get_parameter_value().double_value,
                'min_intensity': self.get_parameter('config.min_intensity').get_parameter_value().double_value,
                'update_rate': self.get_parameter('config.update_rate').get_parameter_value().double_value
            }
            return SensorConfig(**config_data)
        except ValidationError as e:
            self.get_logger().error(f"Invalid configuration: {e}")
            raise
```

## Testing Strategies

### 1. Unit Testing with pytest

**Problem:** Need to test individual components in isolation.

**Solution:** Use pytest with ROS2 mocking.

```python
import pytest
from unittest.mock import Mock, patch
from my_robot_package.sensor_processor import SensorProcessor

def test_sensor_processing():
    """Test sensor processing logic"""
    
    # Create mock node
    mock_node = Mock(spec=Node)
    mock_node.get_logger.return_value = Mock()
    
    # Create processor with mock dependencies
    processor = SensorProcessor()
    processor.node = mock_node
    
    # Test data
    test_points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    
    # Test filtering
    filtered = processor.filter_by_range(test_points, max_range=4.0)
    
    assert len(filtered) == 1
    assert filtered[0] == (1.0, 2.0, 3.0)

def test_error_handling():
    """Test error handling"""
    
    processor = SensorProcessor()
    
    # Test with invalid data
    with pytest.raises(ValueError):
        processor.process_invalid_data([(None, None, None)])
```

### 2. Integration Testing

**Problem:** Need to test component interactions.

**Solution:** Use ROS2 launch testing framework.

```python
import launch
import launch_ros
import launch_testing
import pytest

@pytest.mark.launch_test
def generate_test_description():
    """Generate test description with nodes to test"""
    
    sensor_processor_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='sensor_processor',
        name='sensor_processor'
    )
    
    test_node = launch_ros.actions.Node(
        package='my_robot_package',
        executable='test_sensor_integration',
        name='test_node'
    )
    
    return launch.LaunchDescription([
        sensor_processor_node,
        test_node,
        launch_testing.actions.ReadyToTest()
    ])

class TestSensorIntegration:
    """Integration test class"""
    
    def test_sensor_pipeline(self, proc_info, sensor_processor, test_node):
        """Test the complete sensor processing pipeline"""
        
        # Wait for nodes to be ready
        proc_info.assertWaitForShutdown(sensor_processor, timeout=10)
        proc_info.assertWaitForShutdown(test_node, timeout=10)
        
        # Test pipeline functionality
        # ...
```

## Code Organization

### 1. Modular Design Pattern

**Problem:** Monolithic nodes are hard to maintain.

**Solution:** Organize code into modular components.

```python
# my_robot_package/
# ├── __init__.py
# ├── core/
# │   ├── __init__.py
# │   ├── sensor_processor.py  # Core algorithms
# │   ├── path_planner.py      # Planning algorithms
# │   └── utils.py             # Utility functions
# ├── nodes/
# │   ├── __init__.py
# │   ├── sensor_node.py       # ROS node wrappers
# │   ├── planning_node.py     # ROS node wrappers
# │   └── main.py              # Entry points
# └── launch/
#     └── robot.launch.py      # Launch files

# core/sensor_processor.py
class SensorProcessor:
    """Core sensor processing algorithms (ROS-agnostic)"""
    
    def __init__(self, config):
        self.config = config
        # Initialize processing pipelines
    
    def process_point_cloud(self, data):
        """Process point cloud data"""
        # Core algorithm implementation
        return processed_data
    
    def filter_and_segment(self, points):
        """Filter and segment point cloud"""
        # Implementation...
        return segments

# nodes/sensor_node.py
import rclpy
from rclpy.node import Node
from my_robot_package.core.sensor_processor import SensorProcessor

class SensorNode(Node):
    """ROS wrapper for sensor processor"""
    
    def __init__(self):
        super().__init__('sensor_node')
        
        # Load configuration
        config = self.load_config()
        
        # Initialize core processor
        self.processor = SensorProcessor(config)
        
        # Set up ROS interfaces
        self.subscription = self.create_subscription(
            PointCloud2, 'points', self.handle_points, 10)
        self.publisher = self.create_publisher(
            ProcessedCloud, 'processed_points', 10)
    
    def handle_points(self, msg):
        """Handle incoming point cloud messages"""
        data = self.convert_message(msg)
        processed = self.processor.process_point_cloud(data)
        self.publish_result(processed)
```

### 2. Configuration Management

**Problem:** Hard-coded configurations are inflexible.

**Solution:** Use YAML configuration files with validation.

```python
import yaml
from pathlib import Path
from pydantic import BaseModel, ValidationError

class RobotConfig(BaseModel):
    sensor_ranges: dict
    processing_parameters: dict
    safety_limits: dict

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Load configuration
        self.config = self.load_config()
        
        # Apply configuration
        self.apply_config()
    
    def load_config(self) -> RobotConfig:
        """Load and validate configuration"""
        config_path = Path(__file__).parent / 'config' / 'robot_config.yaml'
        
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
            
            return RobotConfig(**config_data)
        except ValidationError as e:
            self.get_logger().error(f"Invalid configuration: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            raise
    
    def apply_config(self):
        """Apply configuration to node"""
        # Configure sensors
        for sensor_name, params in self.config.sensor_ranges.items():
            self.configure_sensor(sensor_name, params)
        
        # Configure processing
        self.processor.set_parameters(self.config.processing_parameters)
```

## Real-World Examples

### 1. Advanced Sensor Fusion Node

```python
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class AdvancedSensorFusion(Node):
    """
    Advanced sensor fusion node using Kalman filtering
    """
    
    def __init__(self):
        super().__init__('advanced_sensor_fusion')
        
        # Configure QoS for different sensor requirements
        imu_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        mag_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        wheel_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Create subscriptions
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, imu_qos)
        
        self.mag_sub = self.create_subscription(
            MagneticField, 'imu/mag', self.mag_callback, mag_qos)
        
        self.wheel_sub = self.create_subscription(
            Twist, 'wheel_velocity', self.wheel_callback, wheel_qos)
        
        # Create publisher
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', QoSProfile(depth=1))
        
        # Initialize Kalman filter state
        self.state = np.zeros(12)  # Position (3), velocity (3), orientation (3), angular velocity (3)
        self.covariance = np.eye(12) * 0.1
        
        # Process noise and measurement noise
        self.Q = np.eye(12) * 0.01
        self.R_imu = np.eye(6) * 0.1
        self.R_wheel = np.eye(2) * 0.5
        
        # Timing
        self.last_update_time = self.get_clock().now()
        
        # Configuration
        self.declare_parameter('use_mag', True)
        self.declare_parameter('use_wheel', True)
        self.declare_parameter('imu_weight', 0.8)
        self.declare_parameter('wheel_weight', 0.6)
        
        self.use_mag = self.get_parameter('use_mag').get_parameter_value().bool_value
        self.use_wheel = self.get_parameter('use_wheel').get_parameter_value().bool_value
        self.imu_weight = self.get_parameter('imu_weight').get_parameter_value().double_value
        self.wheel_weight = self.get_parameter('wheel_weight').get_parameter_value().double_value
    
    def imu_callback(self, msg):
        """Handle IMU data with Kalman filter update"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        # Predict step
        self.predict(dt)
        
        # Update step with IMU data
        self.update_with_imu(msg, self.R_imu)
        
        # Publish result
        self.publish_odometry()
        
        self.last_update_time = current_time
    
    def predict(self, dt):
        """Kalman filter prediction step"""
        # State transition model (simplified)
        F = np.eye(12)
        F[0:3, 3:6] = np.eye(3) * dt  # Position update from velocity
        F[3:6, 6:9] = np.eye(3) * dt  # Velocity update from orientation
        
        # Predict state and covariance
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + self.Q
    
    def update_with_imu(self, msg, R):
        """Kalman filter update with IMU data"""
        # Measurement model (simplified)
        H = np.zeros((6, 12))
        H[0:3, 3:6] = np.eye(3)  # Linear acceleration
        H[3:6, 9:12] = np.eye(3)  # Angular velocity
        
        # Measurement vector
        z = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Kalman gain
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + K @ (z - H @ self.state)
        self.covariance = (np.eye(12) - K @ H) @ self.covariance
        
        # Update orientation from IMU
        self.update_orientation_from_imu(msg)
    
    def update_orientation_from_imu(self, msg):
        """Update orientation using IMU quaternion"""
        # Convert quaternion to Euler angles
        rot = Rotation.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        euler = rot.as_euler('xyz')
        self.state[6:9] = euler  # Store orientation in state
    
    def publish_odometry(self):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.state[2]
        
        # Orientation
        rot = Rotation.from_euler('xyz', self.state[6:9])
        quat = rot.as_quat()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.linear.y = self.state[4]
        odom.twist.twist.linear.z = self.state[5]
        odom.twist.twist.angular.x = self.state[9]
        odom.twist.twist.angular.y = self.state[10]
        odom.twist.twist.angular.z = self.state[11]
        
        # Covariance (simplified)
        odom.pose.covariance = [
            self.covariance[0,0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.covariance[1,1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.covariance[2,2], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.covariance[6,6], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.covariance[7,7], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.covariance[8,8]
        ]
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = AdvancedSensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Machine Learning Integration

```python
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MLInferenceNode(Node):
    """
    ROS2 node for machine learning inference
    """
    
    def __init__(self):
        super().__init__('ml_inference_node')
        
        # Load ML model
        self.model = self.load_model()
        self.bridge = CvBridge()
        
        # Create subscription
        self.subscription = self.create_subscription(
            Image, 'camera/image', self.image_callback, 10)
        
        # Create publisher
        self.publisher = self.create_publisher(
            DetectionArray, 'detections', 10)
        
        # Pre-allocate tensors for performance
        self.input_tensor = torch.zeros((1, 3, 224, 224), dtype=torch.float32)
        
        # Warm up model
        self.warm_up_model()
    
    def load_model(self):
        """Load PyTorch model"""
        # Load from file
        model = torch.jit.load('model.pt')
        model.eval()
        
        # Move to GPU if available
        if torch.cuda.is_available():
            model = model.to('cuda')
            self.get_logger().info("Using CUDA for inference")
        else:
            self.get_logger().info("Using CPU for inference")
        
        return model
    
    def warm_up_model(self):
        """Warm up model to avoid first-run latency"""
        with torch.no_grad():
            dummy_input = torch.randn(1, 3, 224, 224)
            if torch.cuda.is_available():
                dummy_input = dummy_input.to('cuda')
            self.model(dummy_input)
    
    def image_callback(self, msg):
        """Process camera images with ML model"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Preprocess image
            processed = self.preprocess_image(cv_image)
            
            # Run inference
            with torch.no_grad():
                if torch.cuda.is_available():
                    processed = processed.to('cuda')
                
                outputs = self.model(processed.unsqueeze(0))
                
                if torch.cuda.is_available():
                    outputs = outputs.cpu()
            
            # Post-process results
            detections = self.postprocess_outputs(outputs)
            
            # Publish results
            self.publish_detections(detections, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
    
    def preprocess_image(self, image):
        """Preprocess image for model input"""
        # Resize
        resized = cv2.resize(image, (224, 224))
        
        # Convert to float and normalize
        normalized = resized.astype(np.float32) / 255.0
        
        # Convert to tensor and permute dimensions
        tensor = torch.from_numpy(normalized).permute(2, 0, 1)
        
        # Normalize using ImageNet stats
        mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        tensor = (tensor - mean) / std
        
        return tensor
    
    def postprocess_outputs(self, outputs):
        """Convert model outputs to detection format"""
        # Implementation depends on model output format
        # This is a simplified example
        
        detections = []
        
        # Apply confidence threshold
        confidences = torch.softmax(outputs, dim=1)
        max_conf, preds = torch.max(confidences, dim=1)
        
        for i, (conf, pred) in enumerate(zip(max_conf, preds)):
            if conf > 0.7:  # Confidence threshold
                detection = {
                    'class_id': int(pred),
                    'confidence': float(conf),
                    'bbox': [0.0, 0.0, 1.0, 1.0]  # Simplified
                }
                detections.append(detection)
        
        return detections
    
    def publish_detections(self, detections, header):
        """Publish detection results"""
        msg = DetectionArray()
        msg.header = header
        
        for det in detections:
            detection_msg = Detection()
            detection_msg.class_id = det['class_id']
            detection_msg.confidence = det['confidence']
            detection_msg.bbox.x = det['bbox'][0]
            detection_msg.bbox.y = det['bbox'][1]
            detection_msg.bbox.width = det['bbox'][2]
            detection_msg.bbox.height = det['bbox'][3]
            
            msg.detections.append(detection_msg)
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MLInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

These Python best practices for ROS2 demonstrate how to build professional, production-ready robotic applications that are:

1. **Performant**: Optimized for speed and efficiency
2. **Reliable**: Robust error handling and graceful degradation
3. **Maintainable**: Well-organized, documented, and tested
4. **Scalable**: Designed for complex robotic systems
5. **Production-Ready**: Suitable for real-world deployment

By following these patterns, you can leverage Python's strengths while mitigating its weaknesses in robotic applications, creating systems that are both developer-friendly and production-robust.