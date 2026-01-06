# **ROS 2 Client Library Architecture and Polyglot Development: A Comprehensive Analysis of rclcpp, rclpy, and rclrs**

## **1\. Introduction: The Polyglot Architecture of the Robot Operating System**

The second generation of the Robot Operating System (ROS 2\) represents a paradigm shift in robotics middleware, moving away from the custom TCP/IP handling of ROS 1 to a layered architecture built on top of the Data Distribution Service (DDS) standard. A critical design goal of ROS 2 is to support a polyglot development environment where nodes written in different programming languages can interoperate seamlessly while sharing a common behavioral core. This consistency is achieved not by rewriting the logic for every language, but through a foundational C library known as the ROS Client Library (rcl).

This report provides an exhaustive technical analysis of the ROS 2 client library ecosystem. It scrutinizes the industry-standard C++ library (rclcpp), the rapid-prototyping Python library (rclpy), and the emerging, safety-critical Rust library (rclrs). We examine their internal architectures, memory management strategies, threading models, and performance characteristics. Furthermore, we explore the practical application of these libraries in complex systems like the Navigation2 (Nav2) stack, analyzing design patterns such as Lifecycle Nodes, Component Composition, and Behavior Trees. The analysis extends to advanced integration strategies—including zero-copy intra-process communication and high-frequency web bridging—and concludes with a rigorous examination of the Rust memory model's implications for robotics development.

## **2\. The Architectural Core: The rcl C Layer**

### **2.1 The Foundation of Consistency**

At the heart of the ROS 2 client library architecture lies the rcl (ROS Client Library) package. This C library is responsible for the fundamental implementation of ROS concepts: nodes, publishers, subscriptions, services, clients, and timers. By centralizing this logic in C, the ROS 2 design ensures that the behavior of the system remains consistent regardless of the language used by the developer. This prevents the "behavioral drift" observed in ROS 1, where roscpp and rospy were entirely separate implementations that often handled edge cases—such as time or parameter updates—differently.1

The rcl layer serves as the unified interface to the underlying ROS Middleware (rmw) layer. The rmw layer abstracts the specific DDS implementation (e.g., Fast DDS, Cyclone DDS, RTI Connext), allowing rcl to remain agnostic to the transport mechanism. When a user creates a subscription in Python or C++, the call stack eventually descends into rcl\_subscription\_init, which then invokes the rmw\_create\_subscription function specific to the active middleware.1

### **2.2 Memory Management and Safety in rcl**

As a C library, rcl requires manual memory management. It defines strictly typed structures—such as rcl\_node\_t, rcl\_publisher\_t, and rcl\_wait\_set\_t—which must be explicitly allocated and deallocated by the caller. This design places a significant burden on the authors of client libraries (rclcpp, rclpy, rclrs) to implement robust wrappers that ensure these C structures are initialized and finalized correctly. A failure in the client library to properly manage the lifecycle of an rcl struct can lead to memory leaks or undefined behavior, as the rcl layer assumes the caller owns the memory.1

For example, the rcl\_wait\_set\_t struct is central to the execution model. It aggregates all the handles (subscriptions, timers, service servers) that a node is interested in. The executor mechanism in any high-level language must populate this wait set, call rcl\_wait, and then process the active entities. This polling mechanism is the engine of a ROS 2 node, and understanding it is crucial for analyzing the performance differences between the client libraries.4

## **3\. The Industry Standard: rclcpp (C++)**

rclcpp is the primary client library for ROS 2, designed to leverage modern C++ standards (C++14, C++17, and increasingly C++20). It creates an idiomatic C++ interface that abstracts the raw pointers of rcl behind smart pointers and object-oriented design patterns.

### **3.1 Memory Management: The RAII Paradigm**

In contrast to the manual management of rcl, rclcpp heavily utilizes the Resource Acquisition Is Initialization (RAII) idiom. ROS entities are managed via std::shared\_ptr and std::unique\_ptr. When a Node is created, it is typically instantiated as a std::shared\_ptr\<rclcpp::Node\>. This ensures that the underlying rcl\_node\_t is automatically destroyed when the last reference to the node goes out of scope. This automatic lifecycle management is critical for preventing resource leaks in long-running robotic applications.6

### **3.2 The Execution Model: Executors and Scheduling**

One of the most complex and distinct features of rclcpp compared to ROS 1 is its Executor system. The Executor is responsible for coordinating the execution of callbacks (e.g., when a message arrives or a timer expires).

#### **3.2.1 Single-Threaded vs. Multi-Threaded Executors**

* **SingleThreadedExecutor:** This is the default execution model. It runs all callbacks in the thread that calls spin(). It guarantees that callbacks are executed sequentially, which simplifies thread safety (no mutexes needed for shared data within the node). However, it introduces head-of-line blocking; if one callback takes a long time to compute, all other timers and message processing are stalled.4  
* **MultiThreadedExecutor:** This executor creates a pool of threads (typically equal to the number of CPU cores) to process callbacks concurrently. While this increases throughput, it introduces non-determinism. The developer must ensure that any state shared between callbacks is protected by locking mechanisms (e.g., std::mutex), as the executor may schedule two callbacks from the same node to run simultaneously on different threads.5

#### **3.2.2 Callback Groups and Prioritization**

To manage the complexity of the MultiThreadedExecutor, rclcpp introduces **Callback Groups**. These allow developers to define concurrency rules for specific subsets of callbacks within a node:

1. **Mutually Exclusive Callback Group:** Callbacks assigned to this group are enforced to execute sequentially, never in parallel. This is useful for protecting a specific resource without locking the entire node.  
2. **Reentrant Callback Group:** Callbacks in this group can be executed in parallel, and even multiple instances of the *same* callback can run concurrently (e.g., multiple service requests hitting a server simultaneously). This provides maximum parallelism but requires fully reentrant code.9

Recent versions of rclcpp (Galactic and later) allow adding callback groups to specific Executors. This enables a sophisticated design pattern where high-priority control loops are assigned to a dedicated Executor (pinned to a real-time thread), while low-priority logging and diagnostics are assigned to a background Executor, preventing priority inversion.5

#### **3.2.3 The Events Executor**

A significant evolution in rclcpp is the introduction of the **Events Executor**. Traditional executors rely on polling rcl\_wait, which builds a wait\_set and checks for updates. This can introduce latency and CPU overhead. The Events Executor uses an event-driven design, where the middleware pushes events to the executor, triggering immediate callback execution. This approach aims to reduce the jitter and latency associated with the polling loop, making it more suitable for high-performance scenarios.12

### **3.3 Data Passing Patterns: Intra-Process Communication (IPC)**

One of the most significant advantages of rclcpp is its support for **Intra-Process Communication (IPC)**. When multiple nodes are running within the same process address space, transmitting data via the loopback network interface (localhost) involves unnecessary serialization and deserialization.

#### **3.3.1 The std::unique\_ptr Mechanism**

To achieve true zero-copy transfer between nodes in the same process, rclcpp requires the use of std::unique\_ptr.

* **Mechanism:** When a publisher publishes a message as a std::unique\_ptr\<Msg\>, it transfers ownership of that memory to the middleware. If the middleware detects that the subscriber is in the same process, it can simply pass the pointer to the subscriber.  
* **Requirement:** The subscriber must also request the message as a std::unique\_ptr or const std::shared\_ptr.  
* **Implication:** This prevents the "double update" problem. If data were passed by shared reference, the publisher could modify the data while the subscriber is reading it. By enforcing unique ownership, rclcpp ensures that once data is published, the publisher can no longer modify it, maintaining data integrity without the cost of copying.14

#### **3.3.2 Loaned Messages**

For even higher performance, particularly when interfacing with shared-memory middleware (like Eclipse iceoryx or Fast DDS Shared Memory), rclcpp implements the **Loaned Message** API.

* **Problem:** In standard IPC, the message is allocated in the user space (heap) and then a pointer is passed. In shared memory IPC, the memory must be allocated in a specific shared memory segment managed by the middleware.  
* **Solution:** The publisher calls borrow\_loaned\_message(). The middleware allocates memory in the shared segment and returns a wrapper. The user populates this memory and publishes it.  
* **Constraint:** This effectively eliminates *all* copies, from generation to reception. However, it imposes strict requirements on the message type (typically must be Plain Old Data/POD) and the middleware configuration.16

### **3.4 Component-Based Architecture**

rclcpp strongly promotes a **Component-Based** software design. In ROS 1, "Nodelets" were an advanced feature. In ROS 2, writing nodes as Components is the standard design pattern.

* **Design Pattern:** Developers write nodes as classes inheriting from rclcpp::Node and compile them into shared libraries (.so or .dll).  
* **Dynamic Composition:** These libraries can be loaded at runtime into a generic component\_container process using the ros2 component load command. This decoupling of logic (the Node) from the execution context (the Process) allows system integrators to group nodes into processes to optimize for IPC or isolate faults.19

## **4\. The Prototyping Powerhouse: rclpy (Python)**

rclpy provides Python bindings for the rcl C library. It is widely used for rapid prototyping, system configuration, and integrating strict ROS 2 systems with the vast ecosystem of Python data science and AI libraries (NumPy, SciPy, etc).

### **4.1 The Global Interpreter Lock (GIL) and Concurrency**

The most significant constraint when analyzing rclpy is the Python **Global Interpreter Lock (GIL)**. The GIL is a mutex that protects access to Python objects, preventing multiple threads from executing Python bytecodes at once.

#### **4.1.1 The Illusion of Multithreading**

rclpy provides a MultiThreadedExecutor similar to rclcpp. However, its behavior is fundamentally different due to the GIL.

* **C++:** A MultiThreadedExecutor running on 4 cores on the BCM2712 CPU can execute 4 CPU-intensive callbacks simultaneously (True Parallelism).  
* **Python:** A MultiThreadedExecutor running on 4 cores can only execute one CPU-intensive callback at a time.   
* **Implication:** The MultiThreadedExecutor in Python is primarily useful for **I/O-bound concurrency**. If a callback waits for a service response or a file read, it releases the GIL, allowing another thread to run a callback. For CPU-bound tasks (e.g., image processing, particle filter localization), rclpy multithreading does *not* provide the performance benefit and may even degrade performance due to context-switching overhead, and grabage collector blocks that range anywhere from a few to a few dozen milliseconds.22

### **4.2 Performance Bottlenecks**

Research and benchmarks indicate that rclpy suffers from significant performance penalties compared to rclcpp and rclrs.

* **Latency:** The overhead of the Python interpreter and the C-to-Python wrapping results in message latencies 3x to 100x higher than C++, depending on the workload.  
* **Throughput:** For high-frequency topics (IMU data) (\>500 Hz), rclpy nodes frequently fail to keep up, dropping messages where rclcpp nodes work perfectly fine.  
* **Garbage Collection:** Python's garbage collector introduces non-deterministic (jitter), making rclpy unsuitable for real-time control loops where timing deadlines are strict.2

### **4.3 Integration with asyncio**

To modernize the ROS 2 Python experience, developers often integrate rclpy with Python's asyncio library. This allows the use of async/await syntax for non-blocking I/O, which is standard in modern Python.

* **Strategy:** The integration usually involves running rclpy.spin\_once as a recurring task within the asyncio event loop, or running the asyncio loop in a separate thread. This is the preferred pattern for nodes that must act as web servers or interface with asynchronous database drivers.26

## **5\. The Challenger: rclrs (Rust) and the Memory Safety Revolution**

Rust is emerging as a powerful alternative in the ROS 2 ecosystem. It promises the performance of C++ with the memory safety and ease of development associated with higher-level languages. rclrs is the primary client library for Rust.

### **5.1 The Rust Memory Model in ROS 2**

The integration of Rust into ROS 2 is defined by the **clash** between ROS 2's callback-based architecture and Rust's strict ownership model.

#### **5.1.1 The Shared State Problem**

In rclcpp, sharing data between a subscriber callback and a timer callback is trivial: both are methods of the same class and access member variables via this. In Rust, the compiler forbids multiple mutable references to the same data (XOR mutability).

* **The Conflict:** An executor typically requires ownership of the callbacks. If a callback closes over a variable (captures it), it takes ownership. If two callbacks try to capture the same mutable variable, the borrow checker rejects the code.  
* **The Solution: Arc\<Mutex\<T\>\>:** To share state, ROS 2 Rust nodes universally adopt the Arc\<Mutex\<State\>\> pattern.  
  * **Arc (Atomic Reference Counted):** Allows multiple callbacks to own a handle to the same data. It is thread-safe (Atomic) because callbacks might run on different threads.  
  * **Mutex (Mutual Exclusion):** Provides "interior mutability." Even though the Arc is immutable, the Mutex allows modifying the data inside it, ensuring that only one thread accesses it at a time.  
  * **Implication:** This enforces thread safety at compile time. Data races—in C++ rclcpp nodes—are impossible in safe Rust. However, it introduces the risk of deadlocks if locking logic is not done correctly.27

#### **5.1.2 Send and Sync Traits**

Rust's compiler checks for thread safety using the Send (can be moved to another thread) and Sync (can be shared between threads) traits. Since ROS 2 executors are inherently multithreaded (or should to be), all data captured by callbacks must typically satisfy these traits. This prevents a whole class of concurrency bugs where non-thread-safe types are accidentally used in a multithreaded context.27

### **5.2 Build System Integration: Cargo vs. Colcon**

A major hurdle for rclrs adoption is the build system. ROS 2 relies on colcon (wrapping CMake/Python), while Rust relies on Cargo.

* **The Challenge:** colcon expects build artifacts in an install directory and relies on package.xml for dependency resolution. Cargo builds to a target directory and relies on Cargo.toml.  
* **Solution: colcon-cargo-ros2:** This tool creates a bridge. It is a colcon extension that:  
  1. Reads the ament index to find ROS dependencies.  
  2. Automatically generates Rust bindings for ROS messages (rosidl\_generator\_rs) and places them in a build directory.  
  3. Patches the Cargo.toml or .cargo/config.toml of the package on-the-fly to link against these generated bindings.  
  4. Invokes cargo build.  
     This abstraction allows Rust packages to coexist in a standard colcon build workflow alongside C++ and Python packages.29

### **5.3 Message Generation and Types**

Rust lacks the runtime reflection capabilities of Python or the preprocessor power of C++. Message generation (rosidl\_generator\_rs) translates ROS .msg files into Rust structs.

* **Type Mapping:** A ROS string maps to a Rust std::string::String. A ROS int32 maps to std::vec::Vec\<i32\>.  
* **Build Time:** Because Rust relies on monomorphization (generics) and static linking, and because message bindings are often generated as separate crates, build times can be significant. The message generation pipeline is complex, involving the transpilation of .msg to .idl, then to Rust code, which must then be compiled.31

### **5.4 Learning Curve and Workflow**

* **Learning Curve:** The learning curve for rclrs is pretty steep initially due to the borrow checker. Students must understand lifetimes and the Arc\<Mutex\> pattern immediately to write even a basic node. However, once mastered, the maintenance burden is lower due to fewer runtime errors (no segfaults).  
* **Workflow:** The workflow is heavily centered on cargo. Tools like cargo check provide rapid feedback without full compilation. This contrasts with the slower CMake configuration cycle in rclcpp.34

## **6\. Comparative Analysis: C++, Python, and Rust**

The following table summarizes the key distinctions between the three primary client libraries.

| Feature | rclcpp (C++) | rclpy (Python) | rclrs (Rust) |
| :---- | :---- | :---- | :---- |
| **Performance** | **High:** Native speed, Zero-copy support. | **Low:** Interpreter overhead, GIL bottlenecks. | **High:** Comparable to C++, Zero-copy emerging. |
| **Memory Safety** | **Manual/RAII:** Use-after-free & segfaults possible. | **Runtime Safe:** Exceptions instead of crashes. | **Compile-Time Safe:** Borrow checker prevents memory errors. |
| **Concurrency** | **Complex:** Data races possible; manual locking. | **Limited:** GIL prevents CPU parallelism. | **Safe:** Compiler enforces thread safety (Send/Sync). |
| **Real-Time** | **Yes:** Deterministic memory, no GC. | **No:** Garbage Collector (GC) pauses. | **Yes:** No GC, deterministic execution. |
| **Build System** | **CMake:** Powerful but verbose. Standard for ROS. | **Setuptools:** Standard Python, easy to use. | **Cargo:** Modern, but friction with Colcon. |
| **Ecosystem** | **Mature:** Vast libraries (Nav2, MoveIt). | **Mature:** Data Science/AI (PyTorch, NumPy). | **Emerging:** Limited robotics-specific crates. |
| **Message Passing** | std::shared\_ptr, unique\_ptr (IPC). | Object overhead, serialization costs. | Arc, Box, emerging Zero-copy support. |

### **6.1 Performance Quantification**

Benchmarks verify that rclrs achieves latency and throughput parity with rclcpp. In many tests, they are within 5-10% of each other, with rclrs sometimes winning due to aggressive compiler optimizations in LLVM. In contrast, rclpy consistently demonstrates significantly higher latency and jitter, particularly as message payloads increase in size (e.g., /scan PointClouds) or frequency (IMU data at 1000Hz+).13

## **7\. Case Study: Navigation2 (Nav2) Architecture**

To understand how these libraries function in production, we analyze the Navigation2 (Nav2) stack, a sophisticated system primarily built in rclcpp.

### **7.1 Architecture and Design Patterns**

Nav2 employs a **Lifecycle Node** architecture managed by a centralized lifecycle\_manager. Nodes like the controller\_server and planner\_server are state machines. They do not begin processing data upon startup; they must be transitioned through Unconfigured \-\> Inactive \-\> Active states. This design pattern ensures the system is deterministically initialized—for example, ensuring the Costmap is fully loaded before the Planner begins accepting goals.35

### **7.2 Behavior Trees and Data Passing**

Nav2 replaces the Finite State Machines (FSM) of ROS 1 navigation with **Behavior Trees (BT)** using the BehaviorTree.CPP library.

* **The Blackboard Pattern:** Data is passed between BT nodes (e.g., ComputePathToPose \-\> FollowPath) via a "Blackboard." This is a shared key-value store.  
* **Implementation Details:** The blackboard acts essentially as a map of std::string to std::any (or type-erased pointers).  
* **The Controversy:** While flexible, the Blackboard lacks strict type safety. A node might expect a geometry\_msgs/PoseStamped at key "goal", but if the previous node wrote a Pose, the system may crash or behave unpredictably at runtime.  
* **Best Practice:** The recommended design pattern in Nav2 is to use **Input and Output Ports** for BT nodes. Ports explicitly define the data types a node consumes and produces, allowing the XML loader to validate connections at startup. Direct manipulation of the Blackboard is discouraged in favor of port remapping.38

### **7.3 Why C++ for Nav2?**

Nav2 involves computationally intensive algorithms: A\* pathfinding, DWB local planning (trajectory rollouts), and costmap ray-casting. Implementing these in rclpy would introduce unacceptable latency. rclcpp allows for highly optimized, template-based implementations of these algorithms. Furthermore, the Component composition of rclcpp allows the Planner and Controller to run in the same process as the Costmap, enabling zero-copy transfer of the massive Costmap data structures.41

## **8\. Integration Strategies**

### **8.1 Passing Data Between rclcpp Nodes (Optimization)**

To minimize overhead when passing data between two rclcpp nodes:

1. **Component Composition:** Load both nodes into the same rclcpp\_components::ComponentManager (container).  
2. **Configuration:** Enable use\_intra\_process\_comms in the rclcpp::NodeOptions.  
3. **Data Type:** Publish messages using std::unique\_ptr\<T\>.  
4. **Result:** The RMW layer is bypassed. The pointer is moved from the publisher to the subscriber. This is the "Zero-Copy" strategy essential for Lidar and Camera pipelines.15

### **8.2 Web Bridge Integration**

Integrating ROS 2 with web-based interfaces (e.g., for fleet management dashboards) requires bridging DDS to WebSockets.

* **Strategy 1: rosbridge\_suite (Legacy):** Converts ROS messages to JSON.  
  * **Limitation:** JSON serialization is CPU-intensive. Converting a 1080p image to a Base64 string in JSON creates a massive bottleneck, often capping framerates at \<10 FPS and consuming 100% of a CPU core.43  
* **Strategy 2: foxglove\_bridge (Recommended):** Uses a custom binary protocol (based on CBOR/binary schemas) over WebSockets.  
  * **Advantage:** It supports ROS 2 .msg and .idl schemas natively. It avoids the text serialization overhead. Benchmarks show it can handle high-bandwidth streams (like PointClouds) with significantly lower latency and CPU usage than rosbridge.  
  * **QoS:** For web streams, it is crucial to set the QoS to Volatile (Best Effort). Reliable QoS over a WebSocket bridge can cause head-of-line blocking if packets are dropped on Wi-Fi.43

## **9\. Testing Strategies across Libraries**

Reliability in robotics is non-negotiable. Each library implies a specific testing methodology.

### **9.1 rclcpp Testing (GTest)**

Testing in rclcpp utilizes the ament\_cmake\_gtest framework.

* **Unit Tests:** Standard C++ gtest logic, independent of ROS.  
* **Integration Tests:** The "Node Fixture" pattern is common. A test class inherits from ::testing::Test. In the SetUp method, it initializes rclcpp and an Executor. It spawns a std::thread to run executor.spin(). The test body then publishes messages and uses std::future or condition variables to wait for callbacks to fire. This asynchronous testing is complex but necessary to verify node behavior.12

### **9.2 rclpy Testing (pytest)**

Python testing uses pytest integrated via ament\_cmake\_pytest or ament\_python.

* **Launch Testing:** The launch\_testing framework is heavily used. It launches a node (system under test) and a test node (driver). The test node asserts that the system under test publishes the expected outputs. This is a black-box testing approach widely used in the ROS 2 core packages.48

### **9.3 rclrs Testing (cargo test)**

Rust simplifies the testing workflow.

* **Integration Tests:** Rust automatically treats files in the tests/ directory as integration tests. Each file is compiled as a separate crate.  
* **Methodology:** A test function creates an rclrs::Context, creates a Node, and creates a publisher/subscription pair. It then spins the node (often in a separate thread or using async runtimes) to verify message passing.  
* **Safety:** The Rust compiler acts as a static analysis tool during tests, ensuring that concurrent test logic does not introduce data races—a guarantee C++ tests cannot make.50

## **10\. Conclusion**

The ROS 2 ecosystem has successfully achieved its goal of polyglot support, but the choice of client library dictates the architectural constraints of the system. rclcpp remains the industry standard for production robotics, offering the most mature ecosystem, deepest feature set (IPC, Lifecycle, Components), and highest performance. rclpy is great for tooling, AI integration, and rapid prototyping but is structurally limited by the GIL for real-time control.

rclrs represents the future of secure robotics. By enforcing memory safety and thread safety at compile time, it eliminates entire classes of bugs that plague C++ development. While its ecosystem is younger and the build process requires adaptation (colcon-cargo-ros2), its performance parity with C++ makes it a viable candidate for replacing rclcpp in safety-critical nodes. The ideal modern ROS 2 architecture is likely hybrid: rclcpp for legacy/navigation stacks, rclrs for new safety-critical logic, and rclpy for high-level mission planning and introspection, all bound together by the common rcl core and the DDS data plane.

#### **Works cited**

1. Core Stack Developer Overview — ros\_core crystal documentation, accessed on December 19, 2025, [https://docs.ros2.org/crystal/developer\_overview.html](https://docs.ros2.org/crystal/developer_overview.html)  
2. @ people using ROS2 professionally: what's your percentage of Python vs C++ code, and how has that changed changed during the course of your project/product? : r/ROS \- Reddit, accessed on December 19, 2025, [https://www.reddit.com/r/ROS/comments/1lx00iq/people\_using\_ros2\_professionally\_whats\_your/](https://www.reddit.com/r/ROS/comments/1lx00iq/people_using_ros2_professionally_whats_your/)  
3. How to correctly implement the ROS2 architecture (rclcpp, rcl, rmw) \- Stack Overflow, accessed on December 19, 2025, [https://stackoverflow.com/questions/58336737/how-to-correctly-implement-the-ros2-architecture-rclcpp-rcl-rmw](https://stackoverflow.com/questions/58336737/how-to-correctly-implement-the-ros2-architecture-rclcpp-rcl-rmw)  
4. Dynamic Priority Scheduling of Multi-Threaded ROS 2 Executor with Shared Resources \- Computer Science : Texas State University, accessed on December 19, 2025, [https://userweb.cs.txstate.edu/\~k\_y47/webpage/pubs/emsoft24.pdf](https://userweb.cs.txstate.edu/~k_y47/webpage/pubs/emsoft24.pdf)  
5. Executors — ROS 2 Documentation: Foxy documentation, accessed on December 19, 2025, [https://docs.ros.org/en/foxy/Concepts/About-Executors.html](https://docs.ros.org/en/foxy/Concepts/About-Executors.html)  
6. ROS 2 Client Interfaces (Client Libraries) — ROS 2 Documentation: Dashing documentation, accessed on December 19, 2025, [https://docs.ros.org/en/dashing/Concepts/About-Client-Interfaces.html](https://docs.ros.org/en/dashing/Concepts/About-Client-Interfaces.html)  
7. About ROS 2 client libraries \- ROS documentation, accessed on December 19, 2025, [https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Client-Libraries.html](https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Client-Libraries.html)  
8. examples/rclcpp/executors/multithreaded\_executor/multithreaded\_executor.cpp at rolling · ros2/examples \- GitHub, accessed on December 19, 2025, [https://github.com/ros2/examples/blob/rolling/rclcpp/executors/multithreaded\_executor/multithreaded\_executor.cpp](https://github.com/ros2/examples/blob/rolling/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp)  
9. Using Callback Groups — ROS 2 Documentation: Galactic documentation, accessed on December 19, 2025, [https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html](https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html)  
10. Using Callback Groups — ROS 2 Documentation: Rolling documentation, accessed on December 19, 2025, [https://docs.ros.org/en/rolling/How-To-Guides/Using-callback-groups.html](https://docs.ros.org/en/rolling/How-To-Guides/Using-callback-groups.html)  
11. Executors — ROS 2 Documentation: Xin 文档, accessed on December 19, 2025, [https://daobook.github.io/ros2-docs/xin/Concepts/About-Executors.html](https://daobook.github.io/ros2-docs/xin/Concepts/About-Executors.html)  
12. rclcpp/rclcpp/test/rclcpp/executors/test\_events\_executor.cpp at rolling · ros2/rclcpp \- GitHub, accessed on December 19, 2025, [https://github.com/ros2/rclcpp/blob/rolling/rclcpp/test/rclcpp/executors/test\_events\_executor.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/test/rclcpp/executors/test_events_executor.cpp)  
13. A first look at ROS 2 applications written in asynchronous Rust \- arXiv, accessed on December 19, 2025, [https://arxiv.org/pdf/2505.21323](https://arxiv.org/pdf/2505.21323)  
14. Setting up efficient intra-process communication — ROS 2 Documentation, accessed on December 19, 2025, [https://docs.ros.org/en/kilted/Tutorials/Demos/Intra-Process-Communication.html](https://docs.ros.org/en/kilted/Tutorials/Demos/Intra-Process-Communication.html)  
15. Setting up efficient intra-process communication — ROS 2 Documentation, accessed on December 19, 2025, [https://docs.ros.org/en/galactic/Tutorials/Demos/Intra-Process-Communication.html](https://docs.ros.org/en/galactic/Tutorials/Demos/Intra-Process-Communication.html)  
16. Configure Zero Copy Loaned Messages — ROS 2 Documentation: Jazzy documentation, accessed on December 19, 2025, [https://docs.ros.org/en/jazzy/How-To-Guides/Configure-ZeroCopy-loaned-messages.html](https://docs.ros.org/en/jazzy/How-To-Guides/Configure-ZeroCopy-loaned-messages.html)  
17. Zero Copy via Loaned Messages \- ROS2 Design, accessed on December 19, 2025, [https://design.ros2.org/articles/zero\_copy.html](https://design.ros2.org/articles/zero_copy.html)  
18. Configure Zero Copy Loaned Messages — ROS 2 Documentation, accessed on December 19, 2025, [https://docs.ros.org/en/rolling/How-To-Guides/Configure-ZeroCopy-loaned-messages.html](https://docs.ros.org/en/rolling/How-To-Guides/Configure-ZeroCopy-loaned-messages.html)  
19. composition: Rolling 0.37.4 documentation, accessed on December 19, 2025, [https://docs.ros.org/en/rolling/p/composition/](https://docs.ros.org/en/rolling/p/composition/)  
20. Composing multiple nodes in a single process — ROS 2 Documentation, accessed on December 19, 2025, [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html)  
21. Impact of ROS 2 Node Composition in Robotics | by makarand mandolkar | Medium, accessed on December 19, 2025, [https://medium.com/@mandolkarmakarand94/impact-of-ros-2-node-composition-in-robotics-8ad2295ea0fe](https://medium.com/@mandolkarmakarand94/impact-of-ros-2-node-composition-in-robotics-8ad2295ea0fe)  
22. Deadlocks in rclpy and how to prevent them with use of callback groups \- Karelics, accessed on December 19, 2025, [https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/](https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/)  
23. Parallelization across multiple processors with ROS2 Python \- Robotics Stack Exchange, accessed on December 19, 2025, [https://robotics.stackexchange.com/questions/113560/parallelization-across-multiple-processors-with-ros2-python](https://robotics.stackexchange.com/questions/113560/parallelization-across-multiple-processors-with-ros2-python)  
24. Technical Report TR–2023–03 Performance Analysis of ROS2, accessed on December 19, 2025, [https://sbel.wisc.edu/wp-content/uploads/sites/569/2023/04/TR-2023-03.pdf](https://sbel.wisc.edu/wp-content/uploads/sites/569/2023/04/TR-2023-03.pdf)  
25. ROS2 Performance: rclpy is 30x-100x slower than rclcpp \- Robotics Stack Exchange, accessed on December 19, 2025, [https://robotics.stackexchange.com/questions/98772/ros2-performance-rclpy-is-30x-100x-slower-than-rclcpp](https://robotics.stackexchange.com/questions/98772/ros2-performance-rclpy-is-30x-100x-slower-than-rclcpp)  
26. How to bridge rclpy with Python's asyncio? \- Robotics Stack Exchange, accessed on December 19, 2025, [https://robotics.stackexchange.com/questions/24304/how-to-bridge-rclpy-with-pythons-asyncio](https://robotics.stackexchange.com/questions/24304/how-to-bridge-rclpy-with-pythons-asyncio)  
27. Mastering Rust Arc and Mutex: A Comprehensive Guide to Safe Shared State in Concurrent Programming | by Syed Murtza | Medium, accessed on December 19, 2025, [https://medium.com/@Murtza/mastering-rust-arc-and-mutex-a-comprehensive-guide-to-safe-shared-state-in-concurrent-programming-1913cd17e08d](https://medium.com/@Murtza/mastering-rust-arc-and-mutex-a-comprehensive-guide-to-safe-shared-state-in-concurrent-programming-1913cd17e08d)  
28. Arc and Mutex in Rust | It's all about the bit, accessed on December 19, 2025, [https://itsallaboutthebit.com/arc-mutex/](https://itsallaboutthebit.com/arc-mutex/)  
29. jerry73204/colcon-cargo-ros2 \- GitHub, accessed on December 19, 2025, [https://github.com/jerry73204/colcon-cargo-ros2](https://github.com/jerry73204/colcon-cargo-ros2)  
30. Recommend colcon-cargo-ros2 as simplified alternative to current build setup \#553 \- GitHub, accessed on December 19, 2025, [https://github.com/ros2-rust/ros2\_rust/issues/553](https://github.com/ros2-rust/ros2_rust/issues/553)  
31. message\_generation \- ROS Wiki, accessed on December 19, 2025, [https://wiki.ros.org/message\_generation](https://wiki.ros.org/message_generation)  
32. Long compile time for ROS2 interface package \- Robotics Stack Exchange, accessed on December 19, 2025, [https://robotics.stackexchange.com/questions/92127/long-compile-time-for-ros2-interface-package](https://robotics.stackexchange.com/questions/92127/long-compile-time-for-ros2-interface-package)  
33. CHANGELOG — rosidl\_generator\_rs: Rolling 0.4.9 documentation, accessed on December 19, 2025, [https://docs.ros.org/en/rolling/p/rosidl\_generator\_rs/\_\_CHANGELOG.html](https://docs.ros.org/en/rolling/p/rosidl_generator_rs/__CHANGELOG.html)  
34. Roboticists: Have you used Rust with ROS\[2\]? What were your experiences like? \- Reddit, accessed on December 19, 2025, [https://www.reddit.com/r/rust/comments/173upma/roboticists\_have\_you\_used\_rust\_with\_ros2\_what/](https://www.reddit.com/r/rust/comments/173upma/roboticists_have_you_used_rust_with_ros2_what/)  
35. ROS 2 Navigation2 Configuration: Complete Guide to Optimizing Your Robot Navigation Stack \- ThinkRobotics.com, accessed on December 19, 2025, [https://thinkrobotics.com/blogs/learn/ros-2-navigation2-configuration-complete-guide-to-optimizing-your-robot-navigation-stack](https://thinkrobotics.com/blogs/learn/ros-2-navigation2-configuration-complete-guide-to-optimizing-your-robot-navigation-stack)  
36. How to Use ROS 2 Lifecycle Nodes \- Foxglove, accessed on December 19, 2025, [https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes](https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes)  
37. Lifecycle Manager — Nav2 1.0.0 documentation, accessed on December 19, 2025, [https://docs.nav2.org/configuration/packages/configuring-lifecycle.html](https://docs.nav2.org/configuration/packages/configuring-lifecycle.html)  
38. Blackboard and ports \- BehaviorTree.CPP, accessed on December 19, 2025, [https://www.behaviortree.dev/docs/tutorial-basics/tutorial\_02\_basic\_ports/](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_ports/)  
39. BehaviorTree issue when SubTrees are used · Issue \#3640 · ros-navigation/navigation2, accessed on December 19, 2025, [https://github.com/ros-navigation/navigation2/issues/3640](https://github.com/ros-navigation/navigation2/issues/3640)  
40. How to correctly incorporate built-in plugins in Nav2 (or from thridparty vendors) into a cusotm behavior tree running using BehaviorTree.CPP · Issue \#5415 · ros-navigation/navigation2 \- GitHub, accessed on December 19, 2025, [https://github.com/ros-navigation/navigation2/issues/5415](https://github.com/ros-navigation/navigation2/issues/5415)  
41. Navigation Concepts — Nav2 1.0.0 documentation, accessed on December 19, 2025, [https://docs.nav2.org/concepts/index.html](https://docs.nav2.org/concepts/index.html)  
42. Impact of ROS 2 Node Composition in Robotic Systems, RA-L, accessed on December 19, 2025, [https://discourse.openrobotics.org/t/impact-of-ros-2-node-composition-in-robotic-systems-ra-l/31474](https://discourse.openrobotics.org/t/impact-of-ros-2-node-composition-in-robotic-systems-ra-l/31474)  
43. Foxglove WebSocket bridge for ROS 1 \- GitHub, accessed on December 19, 2025, [https://github.com/foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge)  
44. Announcing the New Foxglove Bridge for Live ROS Data | by Esther S. Weon | Medium, accessed on December 19, 2025, [https://medium.com/@esthersweon/announcing-the-new-foxglove-bridge-for-live-ros-data-189340407be](https://medium.com/@esthersweon/announcing-the-new-foxglove-bridge-for-live-ros-data-189340407be)  
45. foxglove\_bridge \- ROS Package Overview, accessed on December 19, 2025, [https://index.ros.org/p/foxglove\_bridge/](https://index.ros.org/p/foxglove_bridge/)  
46. Connect Reliably to ROS 1 and 2 Stacks with High-Rate Data and Large Messages, accessed on December 19, 2025, [https://discourse.openrobotics.org/t/connect-reliably-to-ros-1-and-2-stacks-with-high-rate-data-and-large-messages/29700](https://discourse.openrobotics.org/t/connect-reliably-to-ros-1-and-2-stacks-with-high-rate-data-and-large-messages/29700)  
47. ros humble \- ROS2 \- Initialize rclcpp within a class (for tests) \- Robotics Stack Exchange, accessed on December 19, 2025, [https://robotics.stackexchange.com/questions/25178/ros2-initialize-rclcpp-within-a-class-for-tests](https://robotics.stackexchange.com/questions/25178/ros2-initialize-rclcpp-within-a-class-for-tests)  
48. launch\_testing\_examples \- ROS Package Overview, accessed on December 19, 2025, [https://index.ros.org/p/launch\_testing\_examples/](https://index.ros.org/p/launch_testing_examples/)  
49. Writing Basic Integration Tests with launch\_testing — ROS 2 Documentation, accessed on December 19, 2025, [https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html)  
50. Integration testing \- Rust By Example, accessed on December 19, 2025, [https://doc.rust-lang.org/rust-by-example/testing/integration\_testing.html](https://doc.rust-lang.org/rust-by-example/testing/integration_testing.html)  
51. Integration Testing Rust Binaries \- Unwound Stack, accessed on December 19, 2025, [https://www.unwoundstack.com/blog/integration-testing-rust-binaries.html](https://www.unwoundstack.com/blog/integration-testing-rust-binaries.html)

While Rust’s memory model is theoretically perfect for zero-copy, the current state of the ROS 2 ecosystem makes rclcpp the "path of least resistance" for this specific optimization222.

\+1  
---

### **Why rclcpp is easier for Zero-Copy**

The C++ client library has built-in, first-class support for the patterns required to bypass the middleware transport layer3333.

\+3

* **Native std::unique\_ptr Support:** The ROS 2 middleware (RMW) is specifically designed to recognize std::unique\_ptr\<T\> in C++4. When you publish a unique pointer, the system knows it has exclusive ownership and can simply "hand off" that pointer to a subscriber in the same process5555.  
  \+2  
* **Component Composition:** The infrastructure for loading multiple nodes into a single process container (to enable zero-copy) is a standard, highly documented part of rclcpp6666.  
  \+1  
* **Loaned Message API:** rclcpp provides a stable borrow\_loaned\_message() API that allows you to allocate memory directly in shared segments (like Iceoryx), which is essential for true zero-copy across process boundaries777777777.  
  \+2

### **The Challenges in Rust (rclrs)**

While you *can* do zero-copy in Rust, you will face more "manual labor" and architectural friction8888.

\+1

* **Emerging Tooling:** Zero-copy support in Rust is still classified as "emerging"9. Many of the convenient abstractions that make it "automatic" in C++ are still being finalized in rclrs10.  
* **Borrow Checker Complexity:** To pass data safely without copying, you must immediately master complex Rust patterns like Arc\<Mutex\<T\>\> or Box111111. While this makes the code safer, it significantly steepens the learning curve compared to C++ smart pointers12121212.  
  \+3  
* **Build System Friction:** To use these advanced features, you often have to navigate the friction between **Cargo** and **Colcon**, sometimes requiring specialized tools like colcon-cargo-ros2 to get your components to play nice in a single process13131313.  
  \+1

---

### **Comparison at a Glance**

| Feature | rclcpp (C++) | rclrs (Rust) |
| :---- | :---- | :---- |
| **Ease of Setup** | **High:** Standard "Node Options" flag14. | **Low:** Requires careful ownership management15. |
| **Maturity** | **Industry Standard:** Mature ecosystem161616. \+1 | **Emerging:** Still evolving17. |
| **Safety** | **Manual:** RAII helps, but segfaults are possible18. | **Compile-Time:** Prevents data races entirely19191919. \+1 |
| **Performance** | **High:** Native zero-copy support20. | **High:** Parity with C++, but harder to implement21. |

