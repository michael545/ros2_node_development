

# **Architectural Principles and Advanced Patterns for Professional ROS 2 Development**

## **Executive Summary**

The maturation of the Robot Operating System (ROS 2\) from an academic framework to a production-grade middleware has introduced a complex landscape of architectural choices. For professional development, the "happy path" tutorials provided in standard documentation are insufficient. They fail to address the critical, non-functional requirements of real-world robotics: determinism, fault tolerance, bandwidth optimization, and rigorous testability. This report provides an exhaustive technical analysis of the "hard questions" facing senior robotics engineers. It deconstructs the trade-offs between C++, Python, and Rust, details the implementation of robust testing pyramids for hardware-dependent code, and provides a blueprint for high-performance architectures linking web interfaces to coverage path planning and sensor fusion subsystems.

The analysis highlights that professional ROS 2 development is less about writing code and more about managing system complexity through strict interfaces, state management (Lifecycle Nodes), and resource-aware composition. It addresses specific "footguns" such as the Python Global Interpreter Lock (GIL) interaction with ROS Executors, the hidden latency costs of JSON-based web bridges, and the mathematical necessities of dynamic covariance tuning in Extended Kalman Filters (EKF).

---

## **1\. The Polyglot Runtime: Strategic Language Selection**

In a professional context, the choice of programming language is the single most significant architectural decision, dictating the system's performance envelope, safety guarantees, and maintainability profile. A modern robot is rarely a mono-glot system; rather, it is a polyglot orchestration where C++, Python, and increasingly Rust, each occupy a specific functional niche.

### **1.1 C++: The Deterministic Core**

C++ remains the non-negotiable standard for the "critical path"—the chain of nodes responsible for sensor acquisition, perception, state estimation, and actuation control. The primary driver for this dominance is not merely raw execution speed, but **memory determinism** and **concurrency control**.

#### **1.1.1 The Performance Imperative and Zero-Copy Transport**

In high-bandwidth applications, such as processing LiDAR point clouds or 4K video streams, the overhead of serialization—converting in-memory objects to a byte stream for transport—is prohibitive. C++ in ROS 2 enables **Intra-Process Communication (IPC)**, a mechanism where messages are passed between nodes in the same process via std::shared\_ptr rather than being serialized through the DDS (Data Distribution Service) layer.1

This capability is unique to C++ (and to a lesser extent Rust). When nodes are implemented as rclcpp::Node components and loaded into a single threaded or multi-threaded component container, a message published by a camera driver is accessible to the perception node instantly, with zero memory copying. In contrast, Python nodes, even when running on the same machine, must serialize data to pass it across the Python/C interface, incurring significant CPU penalties and latency. For a control loop running at 1kHz, or a vision pipeline processing 300MB/s, this serialization overhead is often the difference between a functional system and one that lags dangerously.2

#### **1.1.2 Modern C++ Standards (C++17/20) and Safety**

The "footguns" of C++—memory leaks, dangling pointers, and segmentation faults—are largely mitigated in modern professional development through strict adherence to C++14, C++17, and C++20 standards. Professional ROS 2 codebases (like Nav2) enforce the use of smart pointers (std::shared\_ptr, std::unique\_ptr) for all resource management. This aligns with the RAII (Resource Acquisition Is Initialization) idiom, ensuring that resources like file handles or hardware ports are released deterministically when they go out of scope.

Key features of modern C++ relevant to ROS 2 include:

* **Structured Bindings (C++17):** Drastically improve the readability of code handling ROS parameters or service responses, allowing developers to unpack tuples naturally.  
* **std::optional (C++17):** Eliminates the dangerous practice of using "magic numbers" (e.g., returning \-1 to signal failure). If a sensor reading is invalid, the function returns std::nullopt, forcing the caller to handle the missing data explicitly at compile time.4  
* **Concepts (C++20):** While newer to ROS, Concepts allow for defining clearer interfaces for template-based ROS message handlers, improving compiler error messages and code correctness.4

### **1.2 Python: Orchestration, Tooling, and The GIL Trap**

Python (rclpy) is indispensable for system orchestration, high-level mission planning, and test infrastructure due to its rapid iteration speed and vast ecosystem of libraries (NumPy, SciPy). However, using Python for real-time nodes introduces severe risks.

#### **1.2.1 The Global Interpreter Lock (GIL) and Executor Starvation**

The most critical "footgun" in professional ROS 2 Python development is the interaction between the Global Interpreter Lock (GIL) and ROS 2 Executors. The GIL ensures that only one native thread executes Python bytecode at a time. This creates a deceptive concurrency model when using the MultiThreadedExecutor.

In a C++ MultiThreadedExecutor, multiple callbacks can truly execute in parallel on different CPU cores. In Python, even with a MultiThreadedExecutor, callbacks are time-sliced. If a high-priority callback (e.g., processing an image) takes 50ms of CPU time, it holds the GIL, preventing *any* other callback in that process—including critical heartbeats or control signals—from executing.5

The Deadlock Scenario:  
A frequent production failure occurs when a Python node attempts to make a synchronous service call (client.call()) from within a callback.

1. **The Setup:** A TimerCallback runs.  
2. **The Action:** It calls client.call() to request data from another node.  
3. **The Deadlock:** The callback blocks, waiting for the response. Because it blocks, it does not yield execution back to the Executor. The Executor, starved of cycles (or blocked by the same thread in SingleThreadedExecutor), cannot process the incoming service response. The node hangs indefinitely.6

**Mitigation Strategy:**

* **Use call\_async:** Always prefer asynchronous service calls.  
* **ReentrantCallbackGroups:** Assign heavy callbacks to a ReentrantCallbackGroup and run them in a MultiThreadedExecutor. While this doesn't bypass the GIL, it allows the Executor to interleave other callbacks if the heavy one performs I/O (releasing the GIL).5  
* **Process Isolation:** For true parallelism in Python, avoid threading. Use the multiprocessing library or simply run nodes in separate OS processes.

#### **1.2.2 When to Avoid Python**

Avoid Python for:

* **High-Frequency Control Loops (\>100Hz):** The jitter introduced by the Python garbage collector makes PID loops unstable.2  
* **Large Data Handling:** Subscribing to raw image topics in Python forces a memory copy into the Python heap. This can saturate memory bandwidth and trigger aggressive GC cycles.8

### **1.3 Rust: The Safety-Critical Future**

Rust is emerging as a powerful alternative for "hard" robotics questions—specifically, how to ensure memory safety without the overhead of garbage collection.

#### **1.3.1 The Value Proposition: Compile-Time Guarantees**

Rust’s ownership model (the borrow checker) enforces thread safety at compile time. In C++, a common race condition involves one thread reading a message while another modifies it. Rust makes this impossible to compile unless explicit synchronization primitives (Mutexes, Arc) are used. This "fearless concurrency" is ideal for multi-threaded drivers.9

#### **1.3.2 Ecosystem Maturity and Production Readiness**

The ROS 2 Rust ecosystem is split between ros2\_rust (bindings to the official rcl C library) and r2r (a pure Rust implementation).

* **ros2\_rust:** Aims for parity with rclcpp. It is suitable for production *nodes*, but lacks deep integration with C++ frameworks. You cannot easily write a Rust plugin for the C++-based Nav2 Controller Server.10  
* **r2r:** Focuses on async/await ergonomics. It is excellent for "glue" nodes or standalone logic that needs to handle complex sequences of asynchronous events (e.g., service calls) without the "callback hell" of C++.12

**Verdict for 2024/2025:** Use Rust for standalone, safety-critical nodes (e.g., a safety monitor or a specialized driver). Stick to C++ for code that must integrate tightly with existing stacks like Nav2, MoveIt, or Control.14

### **1.4 Language Selection Matrix**

| Feature | C++ (rclcpp) | Python (rclpy) | Rust (r2r / ros2\_rust) |
| :---- | :---- | :---- | :---- |
| **Primary Use Case** | Drivers, Perception, Control, Fusion | Orchestration, Mission Planning, Tests | Safety Monitors, Standalone Logic |
| **Performance** | Native, Zero-Copy capable | Interpreted, High Serialization Cost | Native, Near C++ speeds |
| **Memory Safety** | Manual (RAII helps), Risk of Leaks | Managed (GC), High Overhead | Compile-time Enforced, Zero Cost |
| **Concurrency** | True Multithreading | GIL Limited, Cooperative | Fearless Multithreading (Async) |
| **Ecosystem** | First-class, Required for Plugins | First-class, Standard for Tools | Second-class, No Plugin ABI |
| **Build System** | CMake (colcon) | Setuptools (colcon) | Cargo (colcon extension) |

---

## **2\. Professional Testing Methodologies**

Writing "good tests" for robotics requires a paradigm shift from standard software testing. You must verify not just logic, but behavior under temporal and physical constraints. A professional strategy employs a tiered approach: Unit, Integration, and Hardware-in-the-Loop (HIL).

### **2.1 Unit Testing: The Library-Node Separation Pattern**

To test effectively, you must decouple your **Robotics Logic** from the **ROS Middleware**.

#### **2.1.1 The Pattern**

Do not implement your algorithm inside the rclcpp::Node class. This makes unit testing impossible because instantiating the class requires a running ROS system, parameters, and DDS discovery.

* **Refactor:** Create a standard C++ library (or Python module) that contains the logic.  
  * *Example (GNSS):* Create a class NmeaParser. It has a method parse(std::string\_view frame) \-\> LatLon. It knows nothing about ROS.  
  * *Example (Control):* Create PurePursuit. It has compute\_cmd(Pose current, Path path) \-\> Twist. It knows nothing about TF buffers.  
* **The Wrapper Node:** The ROS node becomes a thin wrapper. It subscribes to a topic, calls NmeaParser::parse, and publishes the result. This wrapper needs minimal testing.

#### **2.1.2 Testing the Logic**

Now, use GoogleTest (GTest) or PyTest to verify the library.

* **GNSS Test:** Feed the NmeaParser garbage strings, empty strings, and valid sentences. Assert it throws exceptions or returns correct structs. This catches parsing bugs instantly without needing a log file playback.15  
* **Control Test:** Feed the PurePursuit controller a straight path and a robot pose slightly offset. Assert that the steering angle output is non-zero and in the correct direction to correct the error.

### **2.2 Integration Testing with launch\_testing**

Integration tests verify that nodes interact correctly: publishing on the right topics, respecting QoS, and managing lifecycles.

#### **2.2.1 The launch\_testing Framework**

Use launch\_testing to spin up a driver and a consumer node in an isolated environment.

* **Scenario:** Testing the "Control System with Nav2" mentioned in the query.  
* **Setup:** Write a Python launch test that starts the nav2\_controller\_server and your custom path\_follower plugin.  
* **Active Testing:** The test script (acting as a node) publishes a nav\_msgs/Path to the controller's input topic. It then subscribes to /cmd\_vel.  
* **Assertion:** The test asserts that /cmd\_vel messages are received at the expected frequency (e.g., 20Hz) and that the velocity values are within safety limits (e.g., linear.x \< max\_speed).  
* **Isolation:** Use ROS\_DOMAIN\_ID isolation in CI to prevent tests from cross-talking.16

### **2.3 Mocking Hardware and Data**

How do you test a GNSS driver without a GPS receiver? **Mocking.**

* **Serial Port Mocking:** Use socat to create virtual serial port pairs (/dev/ttyV0 \<-\> /dev/ttyV1).  
  * *Test Harness:* Opens ttyV1 and writes NMEA strings.  
  * *Driver:* Configured to read ttyV0.  
  * *Verification:* Assert the driver publishes NavSatFix messages corresponding to the NMEA data.  
* **UDP/TCP Mocking:** For LiDARs or Ethernet sensors, write a simple Python script that opens a socket and replays a pcap (packet capture) file to the localhost loopback interface. The driver connects to localhost and processes the data as if it were real.18

---

## **3\. Advanced System Architecture: Web, Coverage, and Fusion**

The user query presents a sophisticated architecture: A Web UI for coverage planning, a backend coverage server, a path executor (Nav2), and a sensor fusion layer.

### **3.1 Web UI Integration: The "Bridge Bottleneck"**

**The "Hard Question":** How to efficiently design the Web UI to not overload the ros\_web\_bridge.

#### **3.1.1 The Problem: JSON and Bandwidth**

The standard rosbridge\_server communicates via WebSockets using JSON. To send a nav\_msgs/OccupancyGrid (a map) or a camera image, the bridge must Base64-encode the binary data. This increases data size by \~33% and creates massive CPU load for string allocation and parsing. A 1080p image stream can easily consume 100% of a CPU core on the bridge alone.19

#### **3.1.2 The Solution: Binary Bridges and Split Channels**

To "not overload" the bridge, you must stop sending heavy data through it.

1. **Foxglove Bridge:** Replace rosbridge\_suite with foxglove\_bridge. It uses a binary protocol (CBOR or raw binary) over WebSockets. It supports ROS 2 message definitions natively, allowing zero-copy serialization in C++.21  
2. **Map Tiles (The UI Layer):** The user asks about a "satellite map." **Do not stream satellite tiles from ROS.**  
   * **Architecture:** The Web UI should fetch satellite tiles directly from a Tile Server (e.g., OpenStreetMap, Mapbox, or a self-hosted GeoServer) using standard HTTP XYZ tile requests.  
   * **Datum Sync:** The ROS system only publishes the Map Datum (the GPS coordinate of the map frame origin). The Web UI uses this datum to overlay the robot's icon (from NavSatFix or TF) onto the satellite layer. This offloads megabytes of traffic from the ROS bridge to the browser's native network stack.22

### **3.2 Coverage Path Planning Architecture**

**Requirement:** "Select an area, process it, how should the coverage path planner handle all of this?"

#### **3.2.1 The "Coverage Server" Pattern**

Do not hack coverage logic into a UI callback. Implement a dedicated **Action Server** node, ideally using the opennav\_coverage framework (which wraps Fields2Cover).

1. **The Interface (Custom Action):** Define an action ComputeCoveragePath.action.  
   * *Goal:* geometry\_msgs/Polygon (The area), float32 headland\_width, float32 swath\_width.  
   * *Result:* nav\_msgs/Path (The generated path).  
2. **The Workflow:**  
   * **Web UI:** User draws a polygon. The UI converts this to a list of GPS points.  
   * **Bridge:** Transmits the ComputeCoveragePath goal.  
   * **Server Processing:**  
     1. **Transform:** The server listens to tf to convert WGS84 (GPS) points into the robot's map frame (Cartesian meters). This requires a valid robot\_localization setup.  
     2. **Planning:** It calls Fields2Cover to generate swaths (lines) and connects them (turns).  
     3. **Output:** It returns a nav\_msgs/Path covering the area.24

#### **3.2.2 Communicating with the Path Executor (Nav2)**

Once the Coverage Server generates a path, it acts as a client to the **Nav2 Controller Server**.

* **Action Chaining:** The Coverage Server (or a superior Mission Control node) sends the generated path to the FollowPath action server provided by Nav2.  
* **Controller Selection:** Use RegulatedPurePursuit (RPP) or MPPI as the controller plugin. These are designed for path following. RPP is excellent for Ackermann/Car-like robots often used in agriculture/coverage.26

### **3.3 Sensor Fusion and Covariance Tuning**

**Requirement:** "How do I do sensor fusion properly, where should I do it?"

#### **3.3.1 Where?**

Fusion happens **on the robot**, specifically on the main compute unit (Jetson/NUC/RPi). It never happens in the UI or cloud, as latency would destroy the stability of the control loop.

#### **3.3.2 The Architecture: Dual EKF**

Use robot\_localization (RL) in a **Dual EKF** configuration. This is the industry standard.28

1. **Local EKF (ekf\_local):**  
   * *Frame:* odom \-\> base\_link.  
   * *Inputs:* Wheel Encoders (Twist), IMU Gyro (Twist).  
   * *Output:* A smooth, continuous transform.  
   * *Purpose:* This is what the Nav2 Controller uses for local obstacle avoidance and velocity control. It *must not jump*.  
2. **Global EKF (ekf\_global):**  
   * *Frame:* map \-\> odom.  
   * *Inputs:* Wheel Encoders, IMU, **GNSS (Pose)**.  
   * *Output:* The map-to-odom transform.  
   * *Purpose:* This handles the drift. When a GPS fix arrives, this filter "jumps" the map frame to align with reality. This jump is acceptable for global planning but not for local control.

#### **3.3.3 The "Hard Question" of Covariance**

How do you "really" do this? The secret is **Dynamic Covariance Tuning**.

* **The Problem:** Standard drivers output static covariance. If a GPS enters an urban canyon, it might still report high confidence while drifting 5 meters. The EKF trusts it and jumps the robot into a wall.  
* **The Solution:** Write a **Pre-processor Node**.  
  * This node subscribes to the raw GNSS NavSatFix.  
  * It checks the position\_covariance or DOP (Dilution of Precision) fields.  
  * *Logic:* if (msg.position\_covariance \> threshold) { inflate\_covariance(msg, factor=100.0); }  
  * It republishes the modified message to a filtered/fix topic which the EKF subscribes to. This effectively "disconnects" the GPS from the fusion engine when signal quality drops, forcing the robot to rely on Dead Reckoning (Odometry/IMU) until the signal improves.30

---

## **4\. Design Patterns and Code Cleanliness**

To keep code "short/clean/maintainable," apply standard software engineering patterns adapted for ROS 2\.

### **4.1 The Factory Pattern (Pluginlib)**

Scenario: You have different Coverage Algorithms (Spiral, Boustrophedon).  
Pattern: Don't use if/else chains. Use pluginlib.

* **Base Class:** Define an abstract C++ class CoveragePlanner with a virtual method makePlan().  
* **Plugins:** Implement SpiralPlanner and BoustrophedonPlanner as separate classes compiled into shared libraries (.so).  
* **Runtime:** The Coverage Server reads a string parameter ("spiral") and uses the pluginlib::ClassLoader to instantiate the correct class at runtime. This allows you to add new algorithms without recompiling the server node.32

### **4.2 The Strategy Pattern (Behavior Trees)**

Scenario: Mission Control (Go to A, then Scan B, if Battery Low then Dock).  
Pattern: Use Behavior Trees (BT.CPP or py\_trees).

* **Implementation:** Each action (Drive, Scan, Dock) is a leaf node in the tree. The "Strategy" is the structure of the tree (Sequence vs Fallback nodes). This separates the *logic* of the mission from the *implementation* of the actions.34

### **4.3 The Singleton Pattern (Hardware Managers)**

Footgun: Global Singletons are bad for testing.  
ROS Pattern: Use a Node-based Singleton.

* **Concept:** A "Hardware Manager" node owns the exclusive rights to the hardware resource (e.g., the serial port for the GNSS).  
* **Access:** Other nodes do not instantiate the driver class. They access the hardware *only* via Topics/Services exposed by the Manager. The "Singleton" nature is enforced by the OS process boundary, not by a static instance variable in your code.

### **4.4 Composition and Lifecycle**

**Composition:** Build all C++ nodes as rclcpp::Node subclasses and compile them into shared libraries.

* **Deploy:** Use a launch file to load these components into a **Component Container**. This allows you to run the Driver, Pre-processor, and Fusion nodes in a *single process*.  
* **Benefit:** This enables the zero-copy IPC mentioned in Section 1.1, drastically reducing CPU load compared to separate processes.35

**Lifecycle Nodes:** Use rclcpp\_lifecycle::LifecycleNode for drivers.

* **States:** Unconfigured \-\> Inactive \-\> Active.  
* **Usage:** The launch system triggers configure(). If the GNSS fails to open the port, the transition fails, and the system halts deterministically. This prevents the "silent failure" where a driver starts up broken but the rest of the stack keeps running.37

---

## **5\. Configuration Management: Launch and Parameters**

### **5.1 Organizing Launch Files**

Avoid the monolithic "start\_everything.launch.py". Use a **Cascading Include Strategy**.

* **Level 1 (Package):** gnss\_driver/launch/gnss.launch.py (Starts only the driver).  
* **Level 2 (Subsystem):** sensing/launch/sensors.launch.py (Includes gnss.launch.py, lidar.launch.py).  
* **Level 3 (Robot):** my\_robot\_bringup/launch/robot.launch.py (Includes sensors.launch.py, nav2\_bringup, control.launch.py).

Advanced Technique: OpaqueFunction  
Use launch.actions.OpaqueFunction in Python launch files to execute arbitrary Python logic (e.g., parsing a complex config file to generate a list of node names) before the launch description is finalized.39

### **5.2 Parameter Management**

**Footgun:** Hardcoding parameters in C++. *Always* use ROS parameters.

* **Cascading YAMLs:**  
  1. nav2\_defaults.yaml: The generic config provided by the Nav2 package.  
  2. robot\_overrides.yaml: Specific PID gains and footprint for your robot.  
  3. **Launch Logic:** The launch file loads the defaults first, then the overrides. ROS 2 merges the dictionaries, so the overrides take precedence.  
* **Wildcards:** Use /\*\*:ros\_\_parameters in YAML files to apply logging levels or use\_sim\_time to *all* nodes in a namespace, savingh undreds of lines of config.40

---

## **6\. Custom Messages: When and Why**

**Requirement:** "When to use custom messages / when to not."

### **6.1 The Boundary**

* **Do Not Use:** If a standard message exists that is semantically close.  
  * *Example:* Don't create MyGps.msg with lat, lon. Use sensor\_msgs/NavSatFix. The ecosystem (Rviz, Foxglove, plotting tools) already knows how to visualize standard messages. Custom messages break this compatibility.  
* **Do Use:** When the *semantics* of the data are fundamentally different or when bundling improves atomicity.  
  * *Example:* CropStatus.msg containing float32 moisture, string crop\_type, float32 height. Sending these as three separate topics introduces synchronization headaches (time matching). Sending them as one custom message guarantees they represent the same measurement instant.

---

## **7\. Conclusion**

Professional ROS 2 development is a discipline of rigorous architecture. It moves beyond the "scripting" phase of prototyping into a phase of systems engineering.

* **Select C++** for the data plane (Drivers, Fusion, Control) to leverage zero-copy and determinism.  
* **Use Rust** for safety-critical "black box" nodes.  
* **Isolate Python** to the control plane (Orchestration, Tests) and beware the GIL.  
* **Test** by separating logic from middleware (Library-Node pattern) and mocking hardware.  
* **Architect** for bandwidth (Binary Bridges, Tile Servers) and stability (Dual EKF, Dynamic Covariance).  
* **Manage** complexity with Composition, Lifecycle Nodes, and Cascading Launch files.

By adhering to these patterns, the resulting system will not only meet the functional requirements of coverage planning and web integration but will possess the robustness and maintainability required for commercial deployment.

### **Summary of Best Practices Table**

| Area | Prototype / Hobbyist Approach | Professional / Industrial Approach |
| :---- | :---- | :---- |
| **Language** | Python everywhere for speed. | C++ (Critical Path) \+ Python (Tools) \+ Rust (Safety). |
| **Testing** | "It works in Rviz." | GTest (Logic), launch\_testing (Integration), HIL. |
| **Architecture** | Monolithic Nodes, Process-based. | Component Composition, Lifecycle Management. |
| **Web UI** | rosbridge sending huge JSONs. | foxglove\_bridge (Binary), Tile Server separation. |
| **Fusion** | Static Covariance, Single EKF. | Dual EKF, Dynamic/Adaptive Covariance Tuning. |
| **Config** | Hardcoded values, single launch file. | Cascading YAMLs, Wildcards, Hierarchical Launch. |
| **Design** | if/else logic chains. | Plugins (Factory), Behavior Trees (Strategy). |

#### **Works cited**

1. C++ is faster and has better performance than Python, but by how much? : r/ROS \- Reddit, accessed on November 27, 2025, [https://www.reddit.com/r/ROS/comments/ofkidz/c\_is\_faster\_and\_has\_better\_performance\_than/](https://www.reddit.com/r/ROS/comments/ofkidz/c_is_faster_and_has_better_performance_than/)  
2. Performance Comparison Between Python and C++ in ROS2 for Real-Time Applications, accessed on November 27, 2025, [https://robotics.stackexchange.com/questions/103656/performance-comparison-between-python-and-c-in-ros2-for-real-time-applications](https://robotics.stackexchange.com/questions/103656/performance-comparison-between-python-and-c-in-ros2-for-real-time-applications)  
3. DDS Tuning for ROS 2 \- breq.dev, accessed on November 27, 2025, [https://breq.dev/2024/05/17/dds](https://breq.dev/2024/05/17/dds)  
4. Mastering Modern C++: Essential Features in C++17 and C++20 | Federico Sarrocco, accessed on November 27, 2025, [https://federicosarrocco.com/blog/modern-cpp-features-blog-post](https://federicosarrocco.com/blog/modern-cpp-features-blog-post)  
5. Using Callback Groups — ROS 2 Documentation: Galactic documentation, accessed on November 27, 2025, [https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html](https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html)  
6. Deadlocks in rclpy and how to prevent them with use of callback groups \- Karelics, accessed on November 27, 2025, [https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/](https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/)  
7. ros2 spinning problem\[SOLVED\] \- Robotics Stack Exchange, accessed on November 27, 2025, [https://robotics.stackexchange.com/questions/105974/ros2-spinning-problemsolved](https://robotics.stackexchange.com/questions/105974/ros2-spinning-problemsolved)  
8. Release Python GIL (Global interpreter lock) in \`Subscription::take\_message\` · Issue \#1025 · ros2/rclpy \- GitHub, accessed on November 27, 2025, [https://github.com/ros2/rclpy/issues/1025](https://github.com/ros2/rclpy/issues/1025)  
9. Rust vs C++ Performance: Can Rust Actually Be Faster? \[03:25\] : r/theprimeagen \- Reddit, accessed on November 27, 2025, [https://www.reddit.com/r/theprimeagen/comments/1n5noqs/rust\_vs\_c\_performance\_can\_rust\_actually\_be\_faster/](https://www.reddit.com/r/theprimeagen/comments/1n5noqs/rust_vs_c_performance_can_rust_actually_be_faster/)  
10. ros2-rust/ros2\_rust: Rust bindings for ROS 2 \- GitHub, accessed on November 27, 2025, [https://github.com/ros2-rust/ros2\_rust](https://github.com/ros2-rust/ros2_rust)  
11. A first look at ROS 2 applications written in asynchronous Rust \- arXiv, accessed on November 27, 2025, [https://arxiv.org/html/2505.21323v1](https://arxiv.org/html/2505.21323v1)  
12. Release: rust ROS2 library r2r \- ROS General \- Open Robotics Discourse, accessed on November 27, 2025, [https://discourse.openrobotics.org/t/release-rust-ros2-library-r2r/22180](https://discourse.openrobotics.org/t/release-rust-ros2-library-r2r/22180)  
13. r2r \- Rust \- Docs.rs, accessed on November 27, 2025, [https://docs.rs/r2r](https://docs.rs/r2r)  
14. What's the state of Rust for Robotics in 2024? \- Reddit, accessed on November 27, 2025, [https://www.reddit.com/r/robotics/comments/1eon5of/whats\_the\_state\_of\_rust\_for\_robotics\_in\_2024/](https://www.reddit.com/r/robotics/comments/1eon5of/whats_the_state_of_rust_for_robotics_in_2024/)  
15. Testing — ROS 2 Documentation: Foxy documentation, accessed on November 27, 2025, [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/Testing-Main.html](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/Testing-Main.html)  
16. Writing Basic Integration Tests with launch\_testing — ROS 2 Documentation, accessed on November 27, 2025, [https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Integration.html](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Integration.html)  
17. Integration and unit testing in ROS 2 | Arne Baeyens' website, accessed on November 27, 2025, [https://arnebaeyens.com/blog/2024/ros2-integration-testing/](https://arnebaeyens.com/blog/2024/ros2-integration-testing/)  
18. ros2 control \- Best practise for unit/integration/system testing of ros2\_control hw\_interfaces, accessed on November 27, 2025, [https://robotics.stackexchange.com/questions/105839/best-practise-for-unit-integration-system-testing-of-ros2-control-hw-interfaces](https://robotics.stackexchange.com/questions/105839/best-practise-for-unit-integration-system-testing-of-ros2-control-hw-interfaces)  
19. Increase ros2 web bridge buffer size, possible? \- ROS Answers archive, accessed on November 27, 2025, [https://answers.ros.org/question/353066/](https://answers.ros.org/question/353066/)  
20. Announcing the New Foxglove Bridge for Live ROS Data | by Esther S. Weon | Medium, accessed on November 27, 2025, [https://medium.com/@esthersweon/announcing-the-new-foxglove-bridge-for-live-ros-data-189340407be](https://medium.com/@esthersweon/announcing-the-new-foxglove-bridge-for-live-ros-data-189340407be)  
21. Foxglove WebSocket bridge for ROS 1 \- GitHub, accessed on November 27, 2025, [https://github.com/foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge)  
22. nav2\_map\_server \- ROS Package Overview, accessed on November 27, 2025, [https://index.ros.org/p/nav2\_map\_server/](https://index.ros.org/p/nav2_map_server/)  
23. Mapviz for Map Based Visualization in ROS2 \- Robotics Knowledgebase, accessed on November 27, 2025, [https://roboticsknowledgebase.com/wiki/tools/mapviz/](https://roboticsknowledgebase.com/wiki/tools/mapviz/)  
24. ComputeCoveragePath — Nav2 1.0.0 documentation, accessed on November 27, 2025, [https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputeCoveragePath.html](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputeCoveragePath.html)  
25. \[Nav2\] New Complete Coverage Planning Capabilities\! \- Open Robotics Discourse, accessed on November 27, 2025, [https://discourse.openrobotics.org/t/nav2-new-complete-coverage-planning-capabilities/34625](https://discourse.openrobotics.org/t/nav2-new-complete-coverage-planning-capabilities/34625)  
26. Racing With ROS 2 A Navigation System for an Autonomous Formula Student Race Car, accessed on November 27, 2025, [https://arxiv.org/html/2311.14276](https://arxiv.org/html/2311.14276)  
27. Iron to Jazzy — Nav2 1.0.0 documentation, accessed on November 27, 2025, [https://ros.ncnynl.com/en/nav2/migration/Iron.html](https://ros.ncnynl.com/en/nav2/migration/Iron.html)  
28. AI-Driven Dynamic Covariance for ROS 2 Mobile Robot Localization \- ResearchGate, accessed on November 27, 2025, [https://www.researchgate.net/publication/391665895\_AI-Driven\_Dynamic\_Covariance\_for\_ROS\_2\_Mobile\_Robot\_Localization](https://www.researchgate.net/publication/391665895_AI-Driven_Dynamic_Covariance_for_ROS_2_Mobile_Robot_Localization)  
29. Navigating using GPS Localization — Nav2 1.0.0 documentation, accessed on November 27, 2025, [https://docs.nav2.org/tutorials/docs/navigation2\_with\_gps.html](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)  
30. AI-Driven Dynamic Covariance for ROS 2 Mobile Robot Localization \- MDPI, accessed on November 27, 2025, [https://www.mdpi.com/1424-8220/25/10/3026](https://www.mdpi.com/1424-8220/25/10/3026)  
31. AI-Driven Dynamic Covariance for ROS 2 Mobile Robot Localization \- PubMed, accessed on November 27, 2025, [https://pubmed.ncbi.nlm.nih.gov/40431821/](https://pubmed.ncbi.nlm.nih.gov/40431821/)  
32. IntelligentSystemsLabUTV/ros2-examples: ROS 2 example packages and course materials. \- GitHub, accessed on November 27, 2025, [https://github.com/IntelligentSystemsLabUTV/ros2-examples](https://github.com/IntelligentSystemsLabUTV/ros2-examples)  
33. Creating and using plugins (C++) — ROS 2 Documentation: Foxy documentation, accessed on November 27, 2025, [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)  
34. Nav2 implementation \- Fixposition Documentation, accessed on November 27, 2025, [https://docs.fixposition.com/fd/nav2-implementation](https://docs.fixposition.com/fd/nav2-implementation)  
35. Best Practices for Managing and Orchestrating Multiple ROS 2 Tasks Efficiently, accessed on November 27, 2025, [https://robotics.stackexchange.com/questions/117654/best-practices-for-managing-and-orchestrating-multiple-ros-2-tasks-efficiently](https://robotics.stackexchange.com/questions/117654/best-practices-for-managing-and-orchestrating-multiple-ros-2-tasks-efficiently)  
36. Impact of ROS 2 Node Composition in Robotics | by makarand mandolkar | Medium, accessed on November 27, 2025, [https://medium.com/@mandolkarmakarand94/impact-of-ros-2-node-composition-in-robotics-8ad2295ea0fe](https://medium.com/@mandolkarmakarand94/impact-of-ros-2-node-composition-in-robotics-8ad2295ea0fe)  
37. Managed nodes \- ROS2 Design, accessed on November 27, 2025, [https://design.ros2.org/articles/node\_lifecycle.html](https://design.ros2.org/articles/node_lifecycle.html)  
38. How to Use ROS 2 Lifecycle Nodes \- Foxglove, accessed on November 27, 2025, [https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes](https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes)  
39. ROS2 Launch File Tutorial \- Large Projects (remapping, namespaces, parameters), accessed on November 27, 2025, [https://www.youtube.com/watch?v=aL\_oM8QhTVI](https://www.youtube.com/watch?v=aL_oM8QhTVI)  
40. Parameters and launch files: creating configurable Nodes \- (Murilo's) ROS2 Tutorial, accessed on November 27, 2025, [https://ros2-tutorial.readthedocs.io/en/latest/parameters\_and\_launch.html](https://ros2-tutorial.readthedocs.io/en/latest/parameters_and_launch.html)  
41. Managing large projects — ROS 2 Documentation: Humble documentation, accessed on November 27, 2025, [https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)