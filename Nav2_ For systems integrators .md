

# **Nav2 Architecture and Implementation: A Systems Engineering Analysis**

## **1\. Executive Summary and Strategic Overview**

The transformation of mobile robotics from academic curiosity to industrial ubiquity has necessitated a parallel evolution in the software frameworks that govern autonomous movement. At the forefront of this evolution stands Nav2, the second-generation navigation stack built on the Robot Operating System 2 (ROS 2). To define Nav2 simply as "a path planner" is a categorical error that obscures its function as a comprehensive autonomy kernel. From the perspective of a systems integrator, Nav2 represents a highly modular, extensible, and managed framework designed to orchestrate the complex interplay between perception, decision-making, and actuation in dynamic environments.1

Nav2 answers the fundamental "build vs. buy" question for modern robotics companies. Historically, navigation stacks were proprietary differentiators; today, basic point-to-point navigation is a commodity. Nav2 provides a production-grade, open-source baseline that allows engineering teams to focus on application-specific logic—whether that be a warehouse-picking algorithm or agricultural harvesting patterns—rather than reinventing the primitives of obstacle avoidance and global planning.4 Its popularity is driven not merely by its cost (free) but by its adherence to modern systems engineering principles: lifecycle management for reliability, behavior trees for deterministic logic flow, and a plugin-based architecture that decouples algorithms from the execution framework.1

However, the adoption of Nav2 is not without friction. It imposes a significant cognitive load regarding configuration and architecture. The framework requires a shift from the monolithic state machines of the past to a distributed, server-client architecture orchestrated by XML-defined behaviors. This report provides an exhaustive, expert-level analysis of Nav2, deconstructing its "Dijkstra" mythology, detailing its capabilities for isolated path execution, examining the governance structure under Open Navigation LLC, and critically evaluating its suitability for various industrial applications.

## **2\. Architectural Deconstruction: The Move from Node to Framework**

To understand Nav2, one must first dismantle the mental model inherited from ROS 1's move\_base. In the legacy architecture, navigation was largely a black box: a single executable that ingested sensor data and goals, and outputted velocity commands. While effective for simple differential drive robots, this monolithic design proved brittle for complex industrial applications requiring custom error handling, distinct failure modes, or non-standard kinematic constraints.1

Nav2 explodes this black box into a constellation of discrete, managed services. It is not a node; it is a framework of cooperating servers.

### **2.1 The Server-Client Paradigm**

The architecture is fundamentally built around ROS 2 Actions. Heavy computational tasks are encapsulated in "Servers," which expose standard action interfaces. This separation of concerns is the primary enabler for the system's modularity.

| Server Component | Action Interface | Responsibility | Integrator Relevance |
| :---- | :---- | :---- | :---- |
| **Planner Server** | ComputePathToPose | Computes a feasible global path from start to goal (A to B) considering the static map and global costmap. | Swappable engine. Can be replaced by external fleet managers or custom graph search algorithms. 1 |
| **Controller Server** | FollowPath | Generates high-frequency local velocity commands (cmd\_vel) to track the global path while avoiding dynamic obstacles. | The "Driver." Handles kinematic compliance and local collision avoidance. Supports multiple plugin slots. 1 |
| **Smoother Server** | SmoothPath | Post-processes the planned path to improve quality (e.g., removing jagged turns) before execution. | Optional optimization layer. Crucial for Ackermann vehicles to ensure steering continuity. 2 |
| **Behavior Server** | Spin, BackUp, Wait | Executes primitive recovery actions to extricate the robot from stuck conditions. | The escape mechanism. Integrators can add custom behaviors like "Call Operator" or "Blink Lights." 1 |
| **Route Server** | ComputeRoute | Computes topological routes through a graph (e.g., lane networks) rather than free-space planning. | Essential for structured environments (warehouses, roadways) where robots must adhere to traffic rules. 7 |
| **Waypoint Follower** | FollowWaypoints | Orchestrates the execution of multiple sequential goals. | High-level mission execution. Often replaced by custom application logic in complex deployments. 10 |

1

### **2.2 Lifecycle Management: Determinism in Startup**

A critical differentiator for systems integrators is Nav2's adoption of the ROS 2 lifecycle management standard. In a complex robotic system, the order of operations matters. A controller cannot calculate velocities if the costmap is not initialized; a costmap cannot initialize if the map server hasn't loaded the occupancy grid.

Nav2 nodes are not simply "running" or "stopped." They exist in a state machine managed by a central Lifecycle Manager:

1. **Unconfigured:** The node is instantiated but has no configuration.  
2. **Inactive:** The node is configured (parameters loaded) but is not processing data or publishing.  
3. **Active:** The node is fully operational.  
4. **Finalized:** The node is undergoing safe shutdown.

This architectural choice eliminates the "race conditions" prevalent in ROS 1, where nodes would crash because they attempted to access a resource (like a transform) that wasn't yet available. For an industrial integrator, this ensures a deterministic bring-up sequence. The lifecycle\_manager transitions the map\_server to Active, then the planner\_server, then the controller\_server in a strict order, ensuring system stability before the robot ever attempts to move.1

### **2.3 The Orchestrator: The Behavior Tree Navigator**

If the servers are the muscles, the bt\_navigator is the brain. It does not perform planning or control itself; it orchestrates them. It parses a Behavior Tree (BT) defined in XML and executes it tick-by-tick.

This shift to BTs from Finite State Machines (FSMs) is pivotal. FSMs scale poorly; as the number of failure states increases, the number of transitions explodes, leading to "spaghetti code." BTs, conversely, are hierarchical and modular. A "Recovery" subtree can be defined once and reused across multiple navigation contexts.

The bt\_navigator interacts with the servers via the Action interfaces (ComputePathToPose, FollowPath). This decoupling means the "brain" doesn't need to know *how* the path is calculated, only *that* it has been calculated. This abstraction allows integrators to inject arbitrary logic into the navigation loop—such as checking battery status, verifying RFID tags, or waiting for a door to open—without modifying the underlying C++ source code of the planners or controllers.1

## **3\. Algorithmic Reality: The "Dijkstra" Myth vs. Kinematic Feasibility**

The user query raises a specific technical suspicion: *"Is it really built around Dijkstra?"*

This question touches on a common misconception rooted in the legacy of the ROS 1 Navigation Stack. The answer is a definitive **no**, although the framework retains the capability to use Dijkstra if requested. The default planning algorithms in Nav2 have evolved significantly to address the requirements of modern, non-circular robots.

### **3.1 The Legacy: NavFn and Grid Search**

The legacy global planner, NavFn, effectively ports the ROS 1 global planner logic. It utilizes Dijkstra's algorithm or A\* to find the shortest path in a grid.

* **Mechanism:** It treats the robot as a single pixel (or a circular footprint) and minimizes traversal cost through the occupancy grid.  
* **Limitation:** It is unaware of orientation constraints. It produces paths that may require the robot to turn in place or move sideways—maneuvers impossible for Ackermann (car-like) vehicles or industrial tuggers. While mathematically optimal for distance, it is often kinematically infeasible for constrained platforms.12

### **3.2 The Modern Standard: Smac Hybrid-A\***

For systems integrators dealing with industrial vehicles, the **Smac Planner** is the primary reason to adopt Nav2. Developed largely by Steve Macenski (hence "Smac"), this planner implements a Hybrid-A\* algorithm.

* **Kinematic Feasibility:** Unlike Dijkstra, Hybrid-A\* searches in continuous coordinates $(x, y, \\theta)$. It uses "motion primitives"—short, feasible arc segments that respect the robot's minimum turning radius.  
* **Heuristics:** It employs sophisticated heuristics (such as Dubins curves or Reeds-Shepp curves) to guide the search, accounting for the cost of reversing and turning.  
* **Result:** The output path is guaranteed to be drivable by the vehicle. A car-like robot using Smac Planner can autonomously navigate a 3-point turn or parallel park, behaviors that a Dijkstra-based planner simply cannot generate.12

### **3.3 State Lattice and Theta\***

Nav2 offers further specialization:

* **Smac State Lattice:** This planner uses pre-computed control sets for arbitrary robot shapes and kinematics. It is highly efficient for uniform environments and supports complex maneuvering for omnidirectional or highly constrained drivetrains.13  
* *Theta:*\* This is an "Any-Angle" path planner. It improves upon A\* by allowing path segments to cross grid cells at arbitrary angles (Line of Sight), producing smoother, more natural paths than the zig-zag patterns of grid-based A\*, without the full computational weight of Hybrid-A\*. It is ideal for differential drive robots in open spaces.12

**Insight:** The integration of these advanced planners signifies Nav2's transition from a hobbyist tool to an industrial framework. The "Dijkstra" myth implies a lack of sophistication; in reality, Nav2's planning capabilities rival those of proprietary autonomous driving stacks, specifically in unstructured environment handling.13

## **4\. The Control Subsystem: Execution and Feedback**

Once a path is planned, it must be executed. The Controller Server is responsible for generating the cmd\_vel messages that drive the motors. This subsystem has seen perhaps the most dramatic technological leap in Nav2.

### **4.1 DWB: The Reactive Standard**

The DWB (Dynamic Window Approach) controller is the direct successor to the ROS 1 dwa\_local\_planner.

* **Mechanism:** It is a "Critic" based controller. At every time step, it generates a spray of feasible velocity commands (trajectories) based on the robot's acceleration limits. It then scores each trajectory using a weighted sum of critics: PathAlign, GoalAlign, ObstacleDist, etc.  
* **Pros:** It is computationally cheap and highly tunable.  
* **Cons:** It is purely reactive. It lacks "foresight" and can get stuck in local minima (e.g., oscillating in a u-shaped trap) because it optimizes only for the immediate next step. It struggles with dynamic obstacles that require anticipation.1

### **4.2 MPPI: The Predictive Powerhouse**

The **Model Predictive Path Integral (MPPI)** controller represents the cutting edge of Nav2's control capabilities.

* **Mechanism:** MPPI is a sampling-based predictive controller. Instead of generating a fixed fan of trajectories, it generates thousands of random perturbations (noise) to the control inputs and forward-simulates them using the robot's motion model. It scores these thousands of futures and computes a weighted average to determine the optimal control action.  
* **Dynamic Handling:** Because it simulates forward in time (predictive horizon), it naturally accounts for system dynamics and can smoothly navigate around moving obstacles without the jerky "stop-and-go" behavior of reactive planners.  
* **Non-Convex Optimization:** Unlike traditional MPC which requires convex cost functions (smooth bowls), MPPI can handle arbitrary, discontinuous cost functions. This allows an integrator to write complex logic like "stay in the lane center, but if you see a human, hug the right wall," without worrying about mathematical derivatives.  
* **Hardware Acceleration:** While it runs effectively on modern CPUs (i5/i7), its batched nature makes it a candidate for GPU acceleration, aligning with trends in edge-AI compute.9

### **4.3 Regulated Pure Pursuit (RPP)**

For many industrial AGVs (Automated Guided Vehicles), predictability is more valuable than agility.

* **Mechanism:** RPP tracks a "carrot" point on the path ahead of the robot.  
* **Regulation:** It improves upon standard Pure Pursuit by actively regulating linear velocity based on path curvature. It slows down automatically when approaching sharp turns or blind corners (using a collision lookahead).  
* **Use Case:** Ideal for "teach-and-repeat" applications or warehouse tuggers where the robot must strictly adhere to a defined lane \[24, S\_S139 in 24\].

## **5\. Implementation Deep Dive: Path Execution Only**

A specific requirement of the user is to understand if Nav2 can be used for **path execution only**. This is a common requirement for systems integrators who may have an external "Fleet Management System" or a separate AI planner generating the route, and only need Nav2 to handle the local control and obstacle avoidance.

The answer is **Yes**, and the modularity of Nav2 makes this a native capability.

### **5.1 The "Headless" Architecture**

In this configuration, the Planner Server is essentially bypassed or removed. The integrator supplies the path directly to the Controller Server via the Behavior Tree.

**Mechanism:**

1. **Input:** The system requires a mechanism to ingest a nav\_msgs/Path. This is typically done via a ROS 2 topic subscription.  
2. **Blackboard:** The path must be written to the Behavior Tree's "Blackboard" (a shared memory space for the BT).  
3. **Action:** The FollowPath action node in the BT takes the path from the Blackboard and sends it to the Controller Server.

### **5.2 Designing the Execution-Only Behavior Tree**

To implement this, the integrator must create a custom Behavior Tree XML. This tree removes the ComputePathToPose (planning) node and replaces it with logic that waits for a path and then executes it.

**Example XML Structure:**

XML

\<root main\_tree\_to\_execute\="MainTree"\>  
    \<BehaviorTree ID\="MainTree"\>  
        \<PipelineSequence name\="PathExecutionSequence"\>  
            \<ControllerSelector selected\_controller\="{selected\_controller}"   
                                default\_controller\="FollowPath"   
                                topic\_name\="controller\_selector"/\>  
              
            \<FollowPath path\="{path}" controller\_id\="{selected\_controller}"/\>  
              
            \<ReactiveFallback name\="RecoveryFallback"\>  
                \<GoalUpdated/\>  
                \<ClearEntireCostmap name\="ClearLocalCostmap" service\_name\="local\_costmap/clear\_entirely\_local\_costmap"/\>  
                \<Wait wait\_duration\="5"/\>   
            \</ReactiveFallback\>  
        \</PipelineSequence\>  
    \</BehaviorTree\>  
\</root\>

19

**Critical Integration Detail:** The standard NavigateToPose action expects a *Pose* (Goal), not a *Path*. To perform path execution, the integrator typically uses the NavigateThroughPoses action or interacts directly with the FollowPath action server. Alternatively, a custom BT node can be written to subscribe to a topic (e.g., /external\_path), update the blackboard variable {path}, and trigger the sequence. The snippet 42 highlights a common pattern: writing a custom BT plugin to subscribe to a nav\_msgs/Path topic and set it on the blackboard, enabling the FollowPath node to consume it immediately.

## **6\. Systems Integration Framework: The "Glue" of Autonomy**

The user asks: *"What is the system/framework to integrate all of it together?"*

Nav2 *is* the integration framework. It provides the slots (Plugins) and the signals (ROS Topics/Actions) that bind disparate hardware and software components into a cohesive system.

### **6.1 The Plugin Architecture: Infinite Extensibility**

Nav2 relies heavily on pluginlib. Almost every functional block is a plugin loaded at runtime. This is the mechanism that allows switching control systems or planners without recompiling Nav2.

* **Switching Planners:** In the planner\_server configuration (YAML), one can list multiple planners.  
  YAML  
  planner\_server:  
    ros\_\_parameters:  
      planner\_plugins:  
      GridBased:  
        plugin: "nav2\_navfn\_planner/NavfnPlanner"  
      SmacHybrid:  
        plugin: "nav2\_smac\_planner/SmacPlannerHybrid"

* **Switching Controllers:** Similarly, the controller\_server can load DWB, MPPI, and RPP simultaneously. The active controller can be selected dynamically via the Behavior Tree using the ControllerSelector node, which listens to a ROS topic. This allows the robot to use MPPI for high-speed hallway travel and switch to RPP for precision docking.8

### **6.2 Costmap 2D: The Sensor Fusion Hub**

The Costmap 2D system is the primary integration point for perception. It layers data from various sources to create a unified representation of the world.

* **Layered Approach:**  
  * *Static Layer:* The known map from SLAM.  
  * *Obstacle Layer:* Real-time Lidar/Depth Camera data.  
  * *Voxel Layer:* 3D representation of obstacles (crucial for detecting overhangs or tables).  
  * *Inflation Layer:* Mathematically expands obstacles to account for the robot's radius.  
  * *Filter Layers:* Nav2 allows custom filters. For example, a **Speed Filter Layer** can be added to enforce slow zones in specific map regions (e.g., high-traffic intersections).22  
  * *Keepout Zones:* Virtual walls defined by the user to prevent the robot from entering hazardous areas.

For a systems integrator, this means integrating a new sensor (e.g., a radar or ultrasonic array) is as simple as configuring a new layer in the costmap YAML. The planners and controllers automatically respect the new data without code changes.1

### **6.3 The "Shim" Controller Concept**

A prime example of systems engineering elegance in Nav2 is the "Shim" controller (e.g., RotationShimController). This component sits *before* the main controller in the execution chain.

* **Function:** It checks if the robot is aligned with the path orientation. If the heading error is too large, the Shim takes over and rotates the robot in place. Once aligned, it passes control to the main controller (e.g., MPPI).  
* **Benefit:** It allows integrators to compose behaviors. One can use a predictive controller like MPPI for trajectory tracking but wrap it in a Shim to handle the "start-from-rest" condition, preventing the main controller from generating awkward spiraling maneuvers. This is "integration" at the logic level.9

## **7\. Governance and Stewardship: The "Steve" Factor**

The user explicitly asks: *"Who is Steve?"*

In the open-source ecosystem, the viability of a project often hinges on its leadership. **Steve Macenski** is the Project Lead and primary maintainer of Nav2. His role is analogous to a "Benevolent Dictator" for the project, ensuring architectural consistency and code quality.

### **7.1 Professional Bio and Influence**

Steve Macenski is a full-stack roboticist with a history of leading open-source initiatives.

* **History:** He was the Open Source Robotics Engineering Lead at **Samsung Research** and previously a lead at **Simbe Robotics** (creator of the Tally retail robot). At Simbe, he developed early versions of the SLAM and navigation algorithms that would influence his later work.25  
* **Open Navigation LLC:** Recognizing the risk of corporate abandonment (as seen when Intel reduced its ROS investment), Macenski founded **Open Navigation LLC**. This entity serves as the commercial steward of Nav2. It provides a legal and financial structure to support long-term maintenance, ensuring the project does not stagnate.4

### **7.2 The Significance for Integrators**

For a robotics company, "Who is Steve?" is a risk assessment question. The existence of Open Navigation LLC and Macenski's leadership provides:

1. **Accountability:** There is a dedicated entity responsible for the roadmap.  
2. **Standards:** Macenski enforces rigorous testing and quality standards (e.g., typically requiring 90%+ test coverage for new packages).  
3. **Partnerships:** Open Navigation has secured sponsorships and partnerships with major industry players like **AMD**, **Dexory**, and **3Laws**. The partnership with **3Laws** is particularly notable for integrators, as it introduces "dynamic safety guardrails," bridging the gap between open-source functionality and safety-critical certification requirements.27

## **8\. Industry Adoption: Who Uses It and Why?**

Nav2 has successfully crossed the chasm from academic research to industrial deployment. It is the dominant framework for mobile robotics in the ROS 2 ecosystem.

### **8.1 Key Industry Adopters**

The user asks: *"Who uses it?"* The list is extensive and spans multiple verticals:

* **Logistics & Warehousing:**  
  * **Dexory:** Uses Nav2 for autonomous inventory scanning robots in massive warehouses.  
  * **Polymath Robotics:** Automates industrial vehicles.  
  * **Karelics** and **Seasony:** Utilize Nav2 for material handling solutions.29  
* **Agriculture:**  
  * **Firefly Automatix:** Uses Nav2 (specifically Smac Planner) for turf harvesting robots. The ability to plan precise, kinematically feasible paths is critical here to avoid damaging crops.30  
* **Aerospace & Manufacturing:**  
  * **Boeing** and **BMW Group** are listed as users, leveraging Nav2 for internal logistics and assembly line automation.31  
* **Retail & Service:**  
  * **Simbe Robotics:** (Via historical connection and continued ROS usage).  
  * **Kiwibot:** Delivery robots operating in urban environments.30

### **8.2 The "Why Not" Analysis: Critical Downsides**

A balanced report must address the friction points. Why might a systems engineer *reject* Nav2?

#### **8.2.1 Complexity and Learning Curve**

Nav2 is notoriously complex. It is not "plug and play."

* **Configuration Hell:** A functional stack requires tuning dozens of files: nav2\_params.yaml, behavior tree XMLs, costmap plugins, lifecycle manager lists. Understanding the interaction between inflation\_radius, cost\_scaling\_factor, and controller\_frequency requires deep domain expertise.32  
* **Behavior Tree Abstraction:** Debugging a Behavior Tree can be difficult. If a robot stalls, identifying which node in the XML returned FAILURE vs RUNNING often requires specialized visualization tools like **Groot**. The blackboard data flow is implicit, making it harder to trace than explicit function calls.35

#### **8.2.2 Resource Overhead**

Nav2 is a heavyweight framework.

* **Compute Requirements:** Running the full stack (Planner, Controller, Smoother, Costmaps, AMCL, Lifecycle Manager) consumes significant CPU and RAM. While optimizations (like component composition) exist, it generally requires a distinct class of compute (e.g., NVIDIA Jetson Orin, Intel NUC). It is ill-suited for low-power microcontrollers or highly constrained edge devices.2  
* **DDS/Middleware Reliance:** Nav2's performance is bound by the underlying ROS 2 middleware (DDS). Issues with multicast discovery, message latency over WiFi, or high CPU usage from DDS serialization can manifest as "Nav2 failures" (e.g., the controller missing its 20Hz deadline). This introduces a dependency on network configuration that can be fragile in industrial environments.33

#### **8.2.3 Tuning Sensitivity**

"It works out of the box" is a half-truth. It *moves* out of the box. To achieve professional-grade motion (smooth deceleration, tight cornering, deadlock avoidance), extensive tuning is required.

* **MPPI Tuning:** While powerful, MPPI requires tuning noise models (Gaussian perturbations) and critic weights. Incorrect tuning leads to erratic "jittery" behavior or failure to find paths in narrow corridors.40  
* **Costmap Tuning:** If the inflation\_layer is not perfectly tuned to the robot's physical footprint and the environment's clutter density, the planner may fail to find paths that physically exist, or the controller may clip obstacles.34

## **9\. Conclusion**

Nav2 represents the maturation of open-source robotics. It transforms navigation from a collection of algorithms into a structured, managed, and extensible capability.

For the **Robotics Systems Engineer**, Nav2 offers:

* **Capability:** Access to state-of-the-art predictive control (MPPI) and kinematic planning (Smac) that would cost millions to develop internally.  
* **Reliability:** A lifecycle-managed architecture that ensures deterministic behavior.  
* **Flexibility:** A framework that accepts anything from a simple grid search to a neural-network-based controller via standard plugin interfaces.

However, this power comes at the cost of **Complexity**. It demands an engineering team capable of managing ROS 2 lifecycles, understanding behavior tree logic, and tuning sophisticated control parameters. It is not a solution for simple toys; it is a framework for professional autonomy.

By decoupling the "brain" (Behavior Tree) from the "muscle" (Controllers), Nav2 allows integrators to build systems that are not just mobile, but intelligent—capable of context-switching, recovery, and seamless integration into larger fleet management ecosystems. Under the stewardship of Open Navigation LLC, it has become the de facto standard for the next generation of mobile robotics.

---

## **10\. Appendix: Detailed Comparison of Components**

### **10.1 Planner Comparison Matrix**

| Feature | NavFn (Legacy) | Smac Hybrid-A\* | Smac Lattice | Theta\* |
| :---- | :---- | :---- | :---- | :---- |
| **Algorithm Base** | Dijkstra / A\* | Hybrid-A\* | State Lattice A\* | Theta\* (Any-Angle) |
| **Kinematic Awareness** | None (Point robot) | High (Min turning radius) | Very High (Custom footprint) | Low (Line of Sight) |
| **Best For** | Circular, Differential Drive | Ackermann (Cars), Trucks | Omni, Arbitrary Shapes | Differential Drive (Smooth) |
| **Reversing Support** | No | Yes (Native) | Yes (Native) | No |
| **Compute Cost** | Low | Medium/High | Medium | Low/Medium |

12

### **10.2 Controller Comparison Matrix**

| Feature | DWB (Dynamic Window) | MPPI (Predictive) | Regulated Pure Pursuit |
| :---- | :---- | :---- | :---- |
| **Logic** | Reactive (Score trajectories) | Predictive (Sample & Optimize) | Geometric (Follow Carrot) |
| **Dynamic Obstacles** | Poor (Reactive only) | Excellent (Predicts motion) | Poor (Stops or hits) |
| \*\* Smoothness\*\* | Low (Can oscillate) | High (Fluid motion) | Medium (Regulated) |
| **Compute Cost** | Low | High (Benefit from GPU) | Very Low |
| **Tuning Difficulty** | Medium (Critics weights) | High (Noise & Critics) | Low (Lookahead gain) |
| **Primary Use Case** | Basic Differential Drive | High-Performance / Crowds | Industrial / Lane Following |

9

#### **Works cited**

1. Navigation2 Overview, accessed on November 25, 2025, [https://roscon.ros.org/2019/talks/roscon2019\_navigation2\_overview\_final.pdf](https://roscon.ros.org/2019/talks/roscon2019_navigation2_overview_final.pdf)  
2. Impact of ROS 2 Node Composition in Robotic Systems \- arXiv, accessed on November 25, 2025, [https://arxiv.org/pdf/2305.09933](https://arxiv.org/pdf/2305.09933)  
3. Navigation (ROS 1\) Vs Navigation 2 (ROS 2\) | by Sharad Maheshwari | Medium, accessed on November 25, 2025, [https://medium.com/@thehummingbird/navigation-ros-1-vs-navigation-2-ros-2-12398b64cd](https://medium.com/@thehummingbird/navigation-ros-1-vs-navigation-2-ros-2-12398b64cd)  
4. We're Go For Launch — Open Navigation LLC, accessed on November 25, 2025, [https://www.opennav.org/news/launch](https://www.opennav.org/news/launch)  
5. Autonomous Robot Navigation and Nav2 \- Foxglove, accessed on November 25, 2025, [https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2)  
6. AWS RoboMaker now supports ROS2 Foxy Fitzroy featuring Navigation2, accessed on November 25, 2025, [https://aws.amazon.com/blogs/robotics/aws-robomaker-now-supports-ros2-foxy-fitzroy-featuring-navigation2/](https://aws.amazon.com/blogs/robotics/aws-robomaker-now-supports-ros2-foxy-fitzroy-featuring-navigation2/)  
7. Route Server — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-route-server.html](https://docs.nav2.org/configuration/packages/configuring-route-server.html)  
8. Controller Server — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-controller-server.html](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)  
9. From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in the Robot Operating System 2 \- arXiv, accessed on November 25, 2025, [https://arxiv.org/pdf/2307.15236](https://arxiv.org/pdf/2307.15236)  
10. Autonomous robot navigation and Nav2: The first steps. \- Foxglove, accessed on November 25, 2025, [https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps)  
11. Navigation and Behaviour trees for task execution | by Rushikesh Halle | Thoughtworks: e4r™ Tech Blogs | Medium, accessed on November 25, 2025, [https://medium.com/e4r/navigation-and-behaviour-trees-for-task-execution-798ba999f8f6](https://medium.com/e4r/navigation-and-behaviour-trees-for-task-execution-798ba999f8f6)  
12. Setting Up Navigation Plugins — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/setup\_guides/algorithm/select\_algorithm.html](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)  
13. On Use of Nav2 Smac Planners \- ROS, accessed on November 25, 2025, [http://download.ros.org/downloads/roscon/2022/On%20Use%20of%20Nav2%20Smac%20Planners.pdf](http://download.ros.org/downloads/roscon/2022/On%20Use%20of%20Nav2%20Smac%20Planners.pdf)  
14. Smac Hybrid-A\* Planner — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)  
15. Smac Planner — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-smac-planner.html](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)  
16. Gentle introduction to Hybrid A star | by Boseong Jeon \- Medium, accessed on November 25, 2025, [https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869](https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869)  
17. Tuning Guide — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/tuning/index.html](https://docs.nav2.org/tuning/index.html)  
18. Model Predictive Path Integral Controller — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-mppic.html](https://docs.nav2.org/configuration/packages/configuring-mppic.html)  
19. Behavior Tree XML Nodes — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-bt-xml.html](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html)  
20. Nav2 Behavior Trees — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/behavior\_trees/index.html](https://docs.nav2.org/behavior_trees/index.html)  
21. ControllerSelector — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/bt-plugins/actions/ControllerSelector.html](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ControllerSelector.html)  
22. Navigating with Speed Limits — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/tutorials/docs/navigation2\_with\_speed\_filter.html](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html)  
23. ROS-Based Navigation and Obstacle Avoidance: A Study of Architectures, Methods, and Trends \- MDPI, accessed on November 25, 2025, [https://www.mdpi.com/1424-8220/25/14/4306](https://www.mdpi.com/1424-8220/25/14/4306)  
24. Navigation Plugins — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/plugins/](https://docs.nav2.org/plugins/)  
25. About \- Steve Macenski, accessed on November 25, 2025, [https://www.steve.macenski.com/about](https://www.steve.macenski.com/about)  
26. About Us — Open Navigation LLC, accessed on November 25, 2025, [https://www.opennav.org/about](https://www.opennav.org/about)  
27. Open Navigation and 3Laws Partner to Advance the Future of Safe, Scalable Robotics, accessed on November 25, 2025, [https://www.morningstar.com/news/accesswire/1089984msn/open-navigation-and-3laws-partner-to-advance-the-future-of-safe-scalable-robotics](https://www.morningstar.com/news/accesswire/1089984msn/open-navigation-and-3laws-partner-to-advance-the-future-of-safe-scalable-robotics)  
28. New Nav2 Sponsors & New Member of Nav2's Core Maintenance Team\! \- Navigation \- Open Robotics Discourse, accessed on November 25, 2025, [https://discourse.openrobotics.org/t/new-nav2-sponsors-new-member-of-nav2s-core-maintenance-team/51081](https://discourse.openrobotics.org/t/new-nav2-sponsors-new-member-of-nav2s-core-maintenance-team/51081)  
29. Nav2 Route Server \- Agricultural Field \- Polymath Robotics \- YouTube, accessed on November 25, 2025, [https://www.youtube.com/watch?v=dWswiHlpxCM](https://www.youtube.com/watch?v=dWswiHlpxCM)  
30. Robots Using — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/about/robots.html](https://docs.nav2.org/about/robots.html)  
31. vmayoral/ros-robotics-companies \- GitHub, accessed on November 25, 2025, [https://github.com/vmayoral/ros-robotics-companies](https://github.com/vmayoral/ros-robotics-companies)  
32. Master ROS2 Nav2: Essential Path Planning for Robots \- Robotisim, accessed on November 25, 2025, [https://robotisim.com/ros2-nav2-path-planning/](https://robotisim.com/ros2-nav2-path-planning/)  
33. This article reads as a rant about ROS (robot operating system) without mentioni... | Hacker News, accessed on November 25, 2025, [https://news.ycombinator.com/item?id=40633095](https://news.ycombinator.com/item?id=40633095)  
34. ROS 2 Navigation Tuning Guide – Nav2 \- AutomaticAddison.com, accessed on November 25, 2025, [https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)  
35. Behavior-Tree Navigator — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)  
36. Introduction To Nav2 Specific Nodes — Nav2 1.0.0 documentation, accessed on November 25, 2025, [https://docs.nav2.org/behavior\_trees/overview/nav2\_specific\_nodes.html](https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html)  
37. Memory and CPU usage report \- ROS 2 real-time benchmarks, accessed on November 25, 2025, [https://ros-realtime.github.io/ros2\_realtime\_benchmarks/benchmark\_results/galactic/reference\_system/rpi4\_rt/stress-cpu-60%25/rmw\_fastrtps\_cpp/memory\_and\_cpu\_usage\_summary\_report\_120s.html](https://ros-realtime.github.io/ros2_realtime_benchmarks/benchmark_results/galactic/reference_system/rpi4_rt/stress-cpu-60%25/rmw_fastrtps_cpp/memory_and_cpu_usage_summary_report_120s.html)  
38. Nav2 Planner and Controller Server high CPU consumption \- Robotics Stack Exchange, accessed on November 25, 2025, [https://robotics.stackexchange.com/questions/114079/nav2-planner-and-controller-server-high-cpu-consumption](https://robotics.stackexchange.com/questions/114079/nav2-planner-and-controller-server-high-cpu-consumption)  
39. Why ros2 is so frustrating? : r/ROS \- Reddit, accessed on November 25, 2025, [https://www.reddit.com/r/ROS/comments/1horkh8/why\_ros2\_is\_so\_frustrating/](https://www.reddit.com/r/ROS/comments/1horkh8/why_ros2_is_so_frustrating/)  
40. \[Nav2\] Suggestions around single plan \- controller only reactive navigation, accessed on November 25, 2025, [https://robotics.stackexchange.com/questions/103278/nav2-suggestions-around-single-plan-controller-only-reactive-navigation](https://robotics.stackexchange.com/questions/103278/nav2-suggestions-around-single-plan-controller-only-reactive-navigation)  
41. MPPI nav2 controller max speed never reached \- ROS Answers archive, accessed on November 25, 2025, [https://answers.ros.org/question/414931/](https://answers.ros.org/question/414931/)  
42. Creating a Custom Nav2 BT Plugin to Follow a Predefined Path Topic Without a Global Planner \- Robotics Stack Exchange, accessed on November 25, 2025, [https://robotics.stackexchange.com/questions/117145/creating-a-custom-nav2-bt-plugin-to-follow-a-predefined-path-topic-without-a-glo](https://robotics.stackexchange.com/questions/117145/creating-a-custom-nav2-bt-plugin-to-follow-a-predefined-path-topic-without-a-glo)