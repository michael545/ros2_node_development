# C++ Advanced Patterns for ROS2

## Memory Pooling for Real-Time Systems

### Problem: Memory Allocation in Real-Time Contexts

In real-time robotics applications, dynamic memory allocation can cause unpredictable delays and jitter. This is particularly problematic in:
- **Motor control loops** (1kHz+ update rates)
- **Sensor processing pipelines** (high-frequency data streams)
- **Safety-critical systems** (emergency stop handling)

### Solution: Object Pool Pattern

```cpp
// include/real_time_utils/memory_pool.hpp
#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <stdexcept>

namespace real_time_utils {

/**
 * @class MemoryPool
 * @brief Fixed-size memory pool for real-time applications
 * 
 * This class provides pre-allocated memory blocks to avoid
 * dynamic allocation during real-time operations.
 */
template<typename T>
class MemoryPool {
public:
    /**
     * @brief Construct a new Memory Pool object
     * @param size Number of pre-allocated objects
     */
    explicit MemoryPool(size_t size) {
        // Pre-allocate all memory at construction time
        for (size_t i = 0; i < size; ++i) {
            free_list_.push_back(std::make_unique<T>());
        }
    }

    /**
     * @brief Acquire an object from the pool
     * @return Pointer to the acquired object
     * @throws std::runtime_error if pool is exhausted
     */
    T* acquire() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (free_list_.empty()) {
            throw std::runtime_error("Memory pool exhausted");
        }
        
        // Get the last element (faster than front() for vector)
        auto obj = free_list_.back().release();
        free_list_.pop_back();
        
        return obj;
    }

    /**
     * @brief Release an object back to the pool
     * @param obj Pointer to the object to release
     */
    void release(T* obj) {
        if (!obj) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        free_list_.push_back(std::unique_ptr<T>(obj));
    }

    /**
     * @brief Get the number of available objects
     * @return Size of the free list
     */
    size_t available() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return free_list_.size();
    }

    /**
     * @brief Get the total pool size
     * @return Total number of objects in the pool
     */
    size_t size() const {
        return free_list_.capacity();
    }

private:
    std::vector<std::unique_ptr<T>> free_list_;
    mutable std::mutex mutex_;
};

} // namespace real_time_utils
```

### Real-World Application: Sensor Data Processing

```cpp
// include/sensor_processing/sensor_data_processor.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "real_time_utils/memory_pool.hpp"

namespace sensor_processing {

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor()
        : Node("point_cloud_processor"),
          point_pool_(100),  // Pre-allocate 100 point cloud objects
          processing_pool_(50) {  // Pre-allocate 50 processing buffers
        
        // Configure real-time QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliability(RCL_BEST_EFFORT);
        qos.durability(RCL_VOLATILE);
        qos.lifetime(rclcpp::Duration(100ms));
        
        // Create subscription with memory-efficient callback
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                process_point_cloud(msg);
            });
        
        // Create publisher
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "processed_points", qos);
        
        RCLCPP_INFO(get_logger(), "PointCloudProcessor initialized with memory pools");
        RCLCPP_INFO(get_logger(), "Point pool: %zu objects, Processing pool: %zu objects",
                   point_pool_.size(), processing_pool_.size());
    }

private:
    void process_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Acquire processing buffer from pool (zero allocation)
        auto* processing_buffer = processing_pool_.acquire();
        if (!processing_buffer) {
            RCLCPP_ERROR(get_logger(), "Processing pool exhausted!");
            return;
        }
        
        // Process data using pre-allocated buffer
        try {
            // Real-time processing with bounded execution time
            auto start_time = std::chrono::high_resolution_clock::now();
            
            process_data_safely(msg->data, processing_buffer);
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            // Ensure we don't exceed real-time budget
            if (duration.count() > 1000) { // 1ms budget
                RCLCPP_WARN(get_logger(), "Processing exceeded real-time budget: %lld μs",
                           duration.count());
            }
            
            // Publish results using pooled message
            auto processed_msg = create_processed_message(processing_buffer);
            publisher_->publish(std::move(processed_msg));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
        
        // Return buffer to pool
        processing_pool_.release(processing_buffer);
    }
    
    void process_data_safely(const std::vector<uint8_t>& input, void* buffer) {
        // Implementation uses pre-allocated buffer
        // No dynamic memory allocation here!
        
        // Example: Fixed-size processing
        auto* float_buffer = static_cast<float*>(buffer);
        const size_t max_points = 10000; // Fixed capacity
        
        // Process input data with bounds checking
        size_t points_processed = 0;
        for (size_t i = 0; i < input.size() && points_processed < max_points; i += 12) {
            // Convert bytes to floats (example: XYZ coordinates)
            if (i + 12 <= input.size()) {
                float x, y, z;
                std::memcpy(&x, &input[i], 4);
                std::memcpy(&y, &input[i + 4], 4);
                std::memcpy(&z, &input[i + 8], 4);
                
                // Apply processing (e.g., filtering, transformation)
                float_buffer[points_processed * 3] = x * scale_factor_;
                float_buffer[points_processed * 3 + 1] = y * scale_factor_;
                float_buffer[points_processed * 3 + 2] = z * scale_factor_;
                
                points_processed++;
            }
        }
    }
    
    sensor_msgs::msg::PointCloud2::SharedPtr create_processed_message(void* buffer) {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        
        // Configure message header
        msg->header.stamp = now();
        msg->header.frame_id = "processed_frame";
        
        // Set up point cloud structure
        msg->height = 1;
        msg->width = 10000; // Fixed size matching our buffer
        msg->is_dense = true;
        
        // Point cloud fields
        sensor_msgs::msg::PointField field;
        field.name = "x"; field.offset = 0; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        msg->fields.push_back(field);
        
        field.name = "y"; field.offset = 4;
        msg->fields.push_back(field);
        
        field.name = "z"; field.offset = 8;
        msg->fields.push_back(field);
        
        msg->point_step = 12; // 3 floats * 4 bytes each
        msg->row_step = msg->point_step * msg->width;
        
        // Copy data from our processing buffer
        msg->data.resize(msg->row_step);
        std::memcpy(msg->data.data(), buffer, msg->row_step);
        
        return msg;
    }

    // Memory pools for real-time operation
    real_time_utils::MemoryPool<sensor_msgs::msg::PointCloud2> point_pool_;
    real_time_utils::MemoryPool<std::array<float, 30000>> processing_pool_; // 10k points * 3 coordinates
    
    // Configuration
    float scale_factor_ = 1.0f;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

} // namespace sensor_processing
```

### Key Benefits of This Pattern

1. **Predictable Performance**: No dynamic memory allocation during real-time operations
2. **Bounded Execution Time**: Processing fits within real-time budget
3. **Memory Safety**: Proper RAII management of resources
4. **Error Resilience**: Graceful handling of pool exhaustion
5. **Resource Monitoring**: Track pool usage for debugging

## Multi-Threaded Executor Patterns

### Problem: Efficient CPU Utilization

Modern robots often have multi-core processors, but naive ROS2 node design can lead to:
- **Underutilized CPU cores**
- **Thread contention**
- **Priority inversion** issues

### Solution: Custom Multi-Threaded Executor

```cpp
// include/advanced_executors/custom_executor.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <atomic>
#include <memory>

namespace advanced_executors {

/**
 * @class PrioritizedMultiThreadedExecutor
 * @brief Custom executor with thread priority management
 */
class PrioritizedMultiThreadedExecutor {
public:
    /**
     * @brief Construct executor with specific thread configuration
     * @param high_priority_threads Number of high-priority threads
     * @param normal_threads Number of normal-priority threads
     * @param low_priority_threads Number of low-priority threads
     */
    PrioritizedMultiThreadedExecutor(
        size_t high_priority_threads = 2,
        size_t normal_threads = 4,
        size_t low_priority_threads = 2)
        : running_(false) {
        
        // Create thread pools with different priorities
        create_thread_pool(high_priority_threads, ThreadPriority::HIGH);
        create_thread_pool(normal_threads, ThreadPriority::NORMAL);
        create_thread_pool(low_priority_threads, ThreadPriority::LOW);
        
        RCLCPP_INFO(rclcpp::get_logger("PrioritizedMultiThreadedExecutor"),
                   "Created executor with %zu high, %zu normal, %zu low priority threads",
                   high_priority_threads, normal_threads, low_priority_threads);
    }
    
    ~PrioritizedMultiThreadedExecutor() {
        stop();
    }
    
    /**
     * @brief Add a node to the executor
     * @param node Shared pointer to the node
     * @param priority Priority level for the node
     */
    void add_node(
        rclcpp::Node::SharedPtr node,
        ThreadPriority priority = ThreadPriority::NORMAL) {
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Create executor for this node based on priority
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        
        // Assign to appropriate thread pool
        switch (priority) {
            case ThreadPriority::HIGH:
                high_priority_executors_.push_back(executor);
                break;
            case ThreadPriority::NORMAL:
                normal_priority_executors_.push_back(executor);
                break;
            case ThreadPriority::LOW:
                low_priority_executors_.push_back(executor);
                break;
        }
    }
    
    /**
     * @brief Start the executor threads
     */
    void start() {
        if (running_) return;
        
        running_ = true;
        
        // Start all thread pools
        for (auto& pool : thread_pools_) {
            for (auto& thread_data : pool.threads) {
                thread_data.thread = std::thread(&PrioritizedMultiThreadedExecutor::thread_function, 
                                               this, pool.priority);
                
                // Set thread priority if possible
                set_thread_priority(thread_data.thread.native_handle(), pool.priority);
            }
        }
    }
    
    /**
     * @brief Stop the executor threads
     */
    void stop() {
        if (!running_) return;
        
        running_ = false;
        
        // Wake up all threads
        cv_.notify_all();
        
        // Join all threads
        for (auto& pool : thread_pools_) {
            for (auto& thread_data : pool.threads) {
                if (thread_data.thread.joinable()) {
                    thread_data.thread.join();
                }
            }
        }
    }
    
    /**
     * @brief Spin the executor (blocking)
     */
    void spin() {
        start();
        
        // Main thread can do monitoring or low-priority work
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            log_status();
        }
    }

private:
    enum class ThreadPriority {
        HIGH, NORMAL, LOW
    };
    
    struct ThreadPool {
        ThreadPriority priority;
        std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> executors;
        std::vector<struct ThreadData> threads;
    };
    
    struct ThreadData {
        std::thread thread;
        std::atomic<bool> should_exit{false};
    };
    
    std::vector<ThreadPool> thread_pools_;
    std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> high_priority_executors_;
    std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> normal_priority_executors_;
    std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>> low_priority_executors_;
    
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
    
    void create_thread_pool(size_t thread_count, ThreadPriority priority) {
        if (thread_count == 0) return;
        
        ThreadPool pool;
        pool.priority = priority;
        pool.threads.resize(thread_count);
        
        thread_pools_.push_back(pool);
    }
    
    void thread_function(ThreadPriority priority) {
        // Get the appropriate executor list for this priority
        std::vector<std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>>* executors;
        
        switch (priority) {
            case ThreadPriority::HIGH:
                executors = &high_priority_executors_;
                break;
            case ThreadPriority::NORMAL:
                executors = &normal_priority_executors_;
                break;
            case ThreadPriority::LOW:
                executors = &low_priority_executors_;
                break;
            default:
                return;
        }
        
        // Thread-specific executor
        auto local_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        
        while (running_) {
            // Distribute work among executors
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (!executors->empty()) {
                // Round-robin scheduling
                static size_t current_index = 0;
                auto& executor = (*executors)[current_index % executors->size()];
                current_index++;
                
                // Execute available work
                executor->spin_some();
            } else {
                // Wait for work or timeout
                cv_.wait_for(lock, std::chrono::milliseconds(10));
            }
        }
    }
    
    void set_thread_priority(std::thread::native_handle_type handle, ThreadPriority priority) {
        // Platform-specific thread priority setting
        // This is a simplified example - real implementation would use
        // platform-specific APIs (pthread_setschedparam on Linux, etc.)
        
        #ifdef __linux__
        // Example for Linux - would need proper error handling
        // pthread_setschedparam(handle, SCHED_FIFO, &params);
        #endif
    }
    
    void log_status() {
        size_t high_work = 0, normal_work = 0, low_work = 0;
        
        // Count pending work in each priority queue
        // This would be more sophisticated in a real implementation
        
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("PrioritizedMultiThreadedExecutor"),
                            *get_clock(), 5000,
                            "Executor Status - High: %zu, Normal: %zu, Low: %zu pending",
                            high_work, normal_work, low_work);
    }
};

} // namespace advanced_executors
```

### Real-World Usage Example

```cpp
// src/multi_threaded_robot_system.cpp
#include "advanced_executors/custom_executor.hpp"
#include "sensor_processing/sensor_data_processor.hpp"
#include "motor_control/motor_controller.hpp"
#include "navigation/navigation_stack.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Create nodes with different priority requirements
    auto sensor_processor = std::make_shared<sensor_processing::PointCloudProcessor>();
    auto motor_controller = std::make_shared<motor_control::MotorController>();
    auto navigation_stack = std::make_shared<navigation::NavigationStack>();
    auto monitoring_node = std::make_shared<monitoring::SystemMonitor>();
    
    // Create prioritized executor
    advanced_executors::PrioritizedMultiThreadedExecutor executor(
        2,  // High priority threads for sensor processing and motor control
        4,  // Normal priority threads for navigation
        1   // Low priority thread for monitoring
    );
    
    // Add nodes with appropriate priorities
    executor.add_node(sensor_processor, advanced_executors::ThreadPriority::HIGH);
    executor.add_node(motor_controller, advanced_executors::ThreadPriority::HIGH);
    executor.add_node(navigation_stack, advanced_executors::ThreadPriority::NORMAL);
    executor.add_node(monitoring_node, advanced_executors::ThreadPriority::LOW);
    
    // Start the executor
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

## Zero-Copy Message Passing

### Problem: Data Copying Overhead

In high-throughput systems, copying message data can consume significant CPU resources and introduce latency.

### Solution: Shared Memory and Move Semantics

```cpp
// include/performance_optimizations/zero_copy_communication.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

namespace performance_optimizations {

/**
 * @class ZeroCopyPublisher
 * @brief Publisher that minimizes data copying
 */
template<typename MessageT>
class ZeroCopyPublisher {
public:
    ZeroCopyPublisher(
        rclcpp::Node* node,
        const std::string& topic_name,
        size_t history_depth = 10)
        : node_(node) {
        
        // Configure publisher with appropriate QoS
        auto qos = rclcpp::QoS(history_depth);
        qos.reliability(RCL_BEST_EFFORT); // Often sufficient for high-throughput
        
        publisher_ = node_->create_publisher<MessageT>(topic_name, qos);
        
        // Pre-allocate message pool
        message_pool_.reserve(history_depth);
        for (size_t i = 0; i < history_depth; ++i) {
            message_pool_.push_back(std::make_unique<MessageT>());
        }
    }
    
    /**
     * @brief Publish message using move semantics
     * @param message Message to publish (will be moved)
     */
    void publish(MessageT&& message) {
        // Use move semantics to avoid copying
        publisher_->publish(std::move(message));
    }
    
    /**
     * @brief Get a message from pool (for in-place construction)
     * @return Pointer to message from pool
     */
    MessageT* acquire_message() {
        if (message_pool_.empty()) {
            // Fallback: create new message if pool exhausted
            return new MessageT();
        }
        
        auto msg = message_pool_.back().release();
        message_pool_.pop_back();
        return msg;
    }
    
    /**
     * @brief Release message back to pool
     * @param message Pointer to message
     */
    void release_message(MessageT* message) {
        if (message) {
            // Clear message data to reset state
            // This depends on message type - for PointCloud2 we might:
            if constexpr (std::is_same_v<MessageT, sensor_msgs::msg::PointCloud2>) {
                message->data.clear();
                message->data.shrink_to_fit();
            }
            
            message_pool_.push_back(std::unique_ptr<MessageT>(message));
        }
    }

private:
    rclcpp::Node* node_;
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
    std::vector<std::unique_ptr<MessageT>> message_pool_;
};

/**
 * @class ZeroCopySubscriber
 * @brief Subscriber optimized for zero-copy processing
 */
template<typename MessageT>
class ZeroCopySubscriber {
public:
    using CallbackT = std::function<void(typename MessageT::SharedPtr)>
    
    ZeroCopySubscriber(
        rclcpp::Node* node,
        const std::string& topic_name,
        CallbackT callback,
        size_t queue_size = 10)
        : node_(node), callback_(std::move(callback)) {
        
        auto qos = rclcpp::QoS(queue_size);
        qos.reliability(RCL_BEST_EFFORT);
        
        subscription_ = node_->create_subscription<MessageT>(
            topic_name, qos,
            [this](typename MessageT::SharedPtr msg) {
                // Directly pass shared pointer - no copying
                this->callback_(std::move(msg));
            });
    }

private:
    rclcpp::Node* node_;
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
    CallbackT callback_;
};

} // namespace performance_optimizations
```

### Usage Example: High-Throughput Camera Processing

```cpp
// include/camera_processing/zero_copy_camera_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "performance_optimizations/zero_copy_communication.hpp"

namespace camera_processing {

class ZeroCopyCameraNode : public rclcpp::Node {
public:
    ZeroCopyCameraNode() : Node("zero_copy_camera_node") {
        // Create zero-copy publisher
        publisher_ = std::make_unique<performance_optimizations::ZeroCopyPublisher<sensor_msgs::msg::Image>>(
            this, "processed_image");
        
        // Create zero-copy subscriber
        subscriber_ = std::make_unique<performance_optimizations::ZeroCopySubscriber<sensor_msgs::msg::Image>>(
            this, "raw_image",
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                process_image(std::move(msg));
            });
        
        RCLCPP_INFO(get_logger(), "Zero-copy camera node initialized");
    }

private:
    void process_image(sensor_msgs::msg::Image::SharedPtr img) {
        // Process image data in-place using the shared pointer
        // The underlying data is not copied!
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // Example: Simple image processing (in-place if possible)
        if (img->encoding == "bgr8") {
            // Process BGR image data directly
            auto* data_ptr = img->data.data();
            size_t data_size = img->data.size();
            
            // Apply processing (example: simple brightness adjustment)
            for (size_t i = 0; i < data_size; i += 3) {
                // BGR format: Blue, Green, Red
                if (i + 2 < data_size) {
                    data_ptr[i] = std::min(255, data_ptr[i] + brightness_adjust_);     // Blue
                    data_ptr[i+1] = std::min(255, data_ptr[i+1] + brightness_adjust_); // Green  
                    data_ptr[i+2] = std::min(255, data_ptr[i+2] + brightness_adjust_); // Red
                }
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        RCLCPP_DEBUG(get_logger(), "Image processing took %lld μs", duration.count());
        
        // Publish processed image using move semantics
        publisher_->publish(std::move(img));
    }
    
    std::unique_ptr<performance_optimizations::ZeroCopyPublisher<sensor_msgs::msg::Image>> publisher_;
    std::unique_ptr<performance_optimizations::ZeroCopySubscriber<sensor_msgs::msg::Image>> subscriber_;
    
    int brightness_adjust_ = 10; // Configuration parameter
};

} // namespace camera_processing
```

## RAII Resource Management

### Problem: Resource Leaks in Long-Running Systems

ROS2 nodes often run for extended periods, making resource management critical.

### Solution: RAII Pattern for ROS2 Resources

```cpp
// include/resource_management/raii_resources.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace resource_management {

/**
 * @class ScopedNode
 * @brief RAII wrapper for ROS2 nodes
 */
class ScopedNode {
public:
    template<typename NodeT, typename... Args>
    static std::shared_ptr<NodeT> create(Args&&... args) {
        auto node = std::make_shared<NodeT>(std::forward<Args>(args)...);
        
        // Register cleanup
        node->add_on_shutdown_callback([node]() {
            RCLCPP_INFO(node->get_logger(), "Node shutting down cleanly");
        });
        
        return node;
    }
};

/**
 * @class ScopedExecutor
 * @brief RAII wrapper for executors
 */
class ScopedExecutor {
public:
    ScopedExecutor() : executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()) {
        // Configure executor
        executor_->set_max_threads(4);
    }
    
    void add_node(rclcpp::Node::SharedPtr node) {
        executor_->add_node(node);
    }
    
    void spin() {
        executor_->spin();
    }
    
    ~ScopedExecutor() {
        // Clean shutdown
        executor_->cancel();
    }

private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

/**
 * @class ResourceGuard
 * @brief Generic RAII guard for resources
 */
template<typename Resource, typename CleanupFunc>
class ResourceGuard {
public:
    ResourceGuard(Resource resource, CleanupFunc cleanup)
        : resource_(resource), cleanup_(cleanup) {}
    
    ~ResourceGuard() {
        if (cleanup_) {
            cleanup_(resource_);
        }
    }
    
    // No copying allowed
    ResourceGuard(const ResourceGuard&) = delete;
    ResourceGuard& operator=(const ResourceGuard&) = delete;
    
    // Allow moving
    ResourceGuard(ResourceGuard&& other) noexcept
        : resource_(other.resource_), cleanup_(other.cleanup_) {
        other.cleanup_ = nullptr;
    }
    
    Resource* get() { return &resource_; }
    const Resource* get() const { return &resource_; }

private:
    Resource resource_;
    CleanupFunc cleanup_;
};

} // namespace resource_management
```

### Usage Example: Safe Resource Handling

```cpp
// src/safe_robot_system.cpp
#include "resource_management/raii_resources.hpp"
#include "sensor_processing/sensor_data_processor.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        // Create nodes with RAII management
        auto sensor_node = resource_management::ScopedNode::create<sensor_processing::PointCloudProcessor>();
        
        // Create executor with RAII management
        resource_management::ScopedExecutor executor;
        executor.add_node(sensor_node);
        
        // Example: File resource management
        {
            auto file_guard = resource_management::ResourceGuard(
                fopen("sensor_data.log", "w"),
                [](FILE* file) {
                    if (file) {
                        fclose(file);
                        RCLCPP_INFO(rclcpp::get_logger("ResourceGuard"), "File closed cleanly");
                    }
                });
            
            if (file_guard.get()) {
                fprintf(*file_guard.get(), "Sensor processing started\n");
            }
        } // File automatically closed here
        
        // Run the system
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
```

## Best Practices Summary

### 1. **Memory Management**
- **Pre-allocate** all memory for real-time systems
- **Use object pools** for frequently allocated objects
- **Avoid dynamic allocation** in critical paths
- **Monitor memory usage** with RAII guards

### 2. **Concurrency**
- **Use appropriate executors** for different workloads
- **Set thread priorities** for real-time requirements
- **Minimize thread contention** with proper synchronization
- **Monitor thread health** and resource usage

### 3. **Performance Optimization**
- **Use move semantics** to avoid copying
- **Leverage zero-copy techniques** where possible
- **Profile before optimizing** - measure don't guess
- **Set appropriate QoS** for each communication channel

### 4. **Error Handling**
- **Use RAII** for resource management
- **Implement graceful degradation** for pool exhaustion
- **Monitor system health** continuously
- **Log meaningful error messages** for debugging

### 5. **Code Organization**
- **Separate real-time and non-real-time code**
- **Use namespaces** to avoid naming conflicts
- **Document thread safety requirements**
- **Write unit tests** for critical components

These advanced C++ patterns form the foundation of professional ROS2 development, enabling you to build robust, high-performance robotic systems that meet the demanding requirements of real-world applications.