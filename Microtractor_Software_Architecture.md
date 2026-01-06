

# **A Technical Analysis of ROS 2 Navigation, State Estimation, and Mission Planning Architectures**

## **1\. Executive Summary**

The transition of agricultural machinery from manually operated, fossil-fuel-dependent systems to autonomous, data-driven platforms represents a fundamental shift in modern agronomy. This report presents a comprehensive technical analysis of developing an autonomous area processing device based on the BCS two-wheel tractor platform, leveraging the Robot Operating System 2 (ROS 2). The analysis specifically addresses the integration of advanced navigation stacks, the comparative efficacy of state estimation packages (robot\_localization versus fuse), and the development of bespoke mission planning interfaces using web technologies.

The BCS tractor, particularly models such as the 749, 853, and 779, offers a robust mechanical baseline for autonomous retrofitting due to its power take-off (PTO) versatility and reversible handlebars.1 However, transforming this "walk-behind" machine into an autonomous agent requires a sophisticated convergence of electromechanical drive-by-wire (DBW) systems, sensor fusion algorithms capable of handling substantial vibration and wheel slip, and coverage path planning (CPP) algorithms that respect the kinematic constraints of the platform.3

A critical finding of this report is the architectural dichotomy between traditional filtering approaches and modern graph-based optimization for agricultural state estimation. While robot\_localization (RL) remains the industry standard for its maturity and extensive documentation regarding GPS integration 4, the fuse package offers superior handling of delayed measurements and non-linear constraints—capabilities that are theoretically advantageous in GPS-degraded farm environments.5 Nevertheless, due to current limitations in 2D modeling within fuse and the immediate need for robust Nav2 integration, a dual-EKF (Extended Kalman Filter) architecture using robot\_localization is recommended as the primary implementation pathway, with fuse reserved for advanced research applications requiring offline trajectory optimization.

Furthermore, for the control of such a high-inertia vehicle on deformable terrain (soil, mud), the report identifies the Model Predictive Path Integral (MPPI) controller as the superior local planner over traditional gradient-based methods like TEB or DWB.6 This is coupled with the integration of fields2cover via the opennav\_coverage server to automate the generation of complex coverage patterns (boustrophedon, spiral) directly from user-defined polygons.8 Finally, the user interface requirement is addressed through a proposed architecture utilizing rosbridge\_server and Leaflet.js, enabling farmers to define operation zones on satellite imagery which are seamlessly translated into ROS-compatible navigation goals.9

---

## **2\. Mechanical Platform and Drive-by-Wire Architecture**

The foundation of any autonomous field robot is the physical platform and the electromechanical interface that allows digital software to manipulate physical actuators. The BCS tractor presents unique challenges and opportunities compared to standard four-wheeled skid-steer robots.

### **2.1 Platform Kinematics and Transmission Dynamics**

The BCS tractor is distinct in its "two-wheel" configuration, where the engine acts as a counterweight to the implement (tiller, mower, plow) cantilevered across a single axle. This results in a system that is statically stable only when an implement is attached or when the operator is balancing it. For autonomous operation, the addition of a sulky or a caster wheel is often necessary to provide static stability, effectively converting the system into an articulated or tricycle kinematic model.11

#### **2.1.1 Transmission Variants and Steering Logic**

The control strategy depends heavily on the specific BCS model, as the steering mechanisms differ fundamentally between the mechanical and hydrostatic variants.

* **Mechanical Transmission (Models 749, 853):** These models utilize an all-gear drive with a mechanical clutch and differential. Steering is achieved through "steering brakes" or differential locking. The differential allows the wheels to rotate at different speeds; when a steering brake is applied to one wheel, power is directed to the opposing wheel, causing a turn.1  
  * *Implication for Autonomy:* The steering response is non-linear. Engaging the brake does not guarantee a fixed turning radius; rather, it induces a yaw rate dependent on traction conditions. The autonomous controller must be robust to this "slip," necessitating a high-frequency control loop.12  
* **Hydrostatic Transmission (Models 779, 780):** These units replace the mechanical gearbox with a hydrostatic drive, allowing for continuously variable speed control without gear shifting. Steering is often assisted by hydraulic actuators or separate hydraulic motors for each wheel in aftermarket modifications.2  
  * *Implication for Autonomy:* Hydrostatic drives offer superior low-speed control, which is critical for precision tasks like cultivating. They eliminate the need for complex gear-shifting actuation, reducing the mechanical complexity of the DBW system.2

#### **2.1.2 The "Dualsteer" Concept**

Advanced specialized tractors in the BCS Group lineup utilize a "Dualsteer" system, which synchronizes the central articulation of the chassis with the steering of the front wheels.13 While standard two-wheel tractors do not have this, understanding this mechanism is vital if the user intends to scale to larger isodiametric tractors. Dualsteer allows for a turning angle of up to 70 degrees, significantly reducing the headland space required for turning—a critical parameter for the coverage path planners discussed in Section 5\.13

### **2.2 Drive-by-Wire (DBW) Implementation Strategy**

To interface ROS 2 with the BCS tractor, a custom DBW kit must be fabricated. This involves retrofitting linear actuators to the physical controls: the steering brakes, the clutch, and the throttle.

#### **2.2.1 Linear Actuation for Steering and Clutching**

The primary method for automating the "brake-steer" mechanism of models like the 749/853 is the use of electric linear actuators.

* **Actuator Specifications:** The brake levers on a BCS tractor require significant force to engage. Actuators rated for 12V DC, with a force capacity of at least 200 lbs (approx. 900 N) and a stroke length of 2-4 inches (50-100mm) are recommended.14 Speed is also a factor; slow actuators will result in sluggish steering response, causing oscillation in the path tracking. High-speed actuators (e.g., \>1 inch/sec) are preferred.15  
* **Mounting Topologies:**  
  * *Handlebar Mounting:* Actuators are clamped to the handlebars, pulling the existing brake cables. This is non-destructive but adds weight high up on the handlebars, increasing vibration.  
  * *Chassis Mounting:* Actuators act directly on the brake linkage arms near the axle. This is mechanically superior as it lowers the center of gravity and reduces cable stretch, but requires custom fabrication of brackets.16

#### **2.2.2 Motor Controllers and Feedback Loops**

Simply applying voltage to an actuator is insufficient for autonomous steering. "Bang-bang" control (full extension/retraction) leads to erratic behavior. Precise position control is required.

* **The Role of Potentiometers:** Selected linear actuators must include internal potentiometers (feedback) to report their current extension length.17  
* **RoboClaw Implementation:** The RoboClaw series of motor controllers is highly recommended for this application. Unlike simple H-bridges, RoboClaws have internal PID processors that can read the potentiometer feedback and perform closed-loop position control directly on the driver board.18 This offloads the high-frequency PID loop from the main ROS computer. The ROS node simply publishes a target position (e.g., "Brake 50%"), and the RoboClaw ensures the actuator reaches that position accurately.

#### **2.2.3 Safety Systems: The Dead Man's Switch**

BCS tractors are equipped with an Operator Presence Control (OPC) lever (usually red) that must be depressed for the engine to run.

* **Failsafe Design:** Automation of the OPC must be "Normally Open." An actuator should hold the lever down only while actively receiving a "heartbeat" signal from the ROS computer. If the ROS node crashes, the computer loses power, or the emergency stop (E-Stop) is pressed, the actuator must release, killing the engine instantly. This can be achieved using a spring-return actuator or a relay-controlled magnetic latch.19

---

## **3\. State Estimation: Deep Dive into robot\_localization vs. fuse**

The core of autonomous navigation is "State Estimation"—determining exactly where the robot is in the world. For an agricultural environment, which is often GPS-degraded (near trees/barns) and feature-poor (open fields), relying on a single sensor is catastrophic. Sensor fusion is mandatory. The user has specifically requested a comparison between the established robot\_localization package and the newer fuse package.

### **3.1 The Industry Standard: robot\_localization (RL)**

robot\_localization is the cornerstone of state estimation in ROS 1 and ROS 2\. It employs non-linear estimation typically via an Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF).4

#### **3.1.1 Dual-EKF Architecture**

For an outdoor robot using GPS, the standard configuration involves two distinct EKF instances running in parallel, linked by a coordinate transform node. This architecture effectively separates local continuity from global accuracy.

**Table 1: Dual-EKF Configuration Strategy**

| Filter Instance | Frame Transformation | Input Sensors | Purpose |
| :---- | :---- | :---- | :---- |
| **Local EKF** (ekf\_local) | odom $\\rightarrow$ base\_link | Wheel Encoders, IMU (Gyro/Accel) | Produces smooth, continuous odometry for local controllers. Critically, it **ignores GPS**. If GPS jumps due to multipath, this filter remains stable, preventing the robot from jerking. |
| **Global EKF** (ekf\_global) | map $\\rightarrow$ odom | Wheel Encoders, IMU, **GPS Odometry** | Fuses the absolute position from GPS. Its output acts as a correction layer, publishing the transform between the map (world) and odom (robot start) frames. |

#### **3.1.2 The navsat\_transform\_node and Circular Dependency**

The navsat\_transform\_node is the bridge between the geodetic world (Latitude/Longitude) and the Cartesian world (X/Y meters) of ROS.

* **Function:** It converts sensor\_msgs/NavSatFix into nav\_msgs/Odometry.  
* **The Circular Dependency:** A common source of confusion is the "circular" nature of this node's inputs. To accurately project GPS coordinates into the robot's frame (taking into account the robot's initial orientation), the node requires the robot's current orientation. Thus, it subscribes to the output of the *Local EKF* (odometry) to get the heading, uses that heading to interpret the GPS movement, and then outputs a global pose that is fed into the *Global EKF*.21  
* **Datum Configuration:** For repeatable farming operations, relying on the robot's startup position as (0,0) is insufficient. The wait\_for\_datum parameter should be set to true. This allows the operator to define a fixed "Datum" (e.g., the corner of the barn) with a known Lat/Lon and Heading. All future navigation maps will be relative to this permanent point, ensuring that "Row 1" is in the same physical location day after day.4

### **3.2 The Modern Contender: fuse**

The fuse package represents a paradigm shift from filtering (EKF) to **graph-based optimization** (Graph SLAM). It was developed by Locus Robotics to address limitations in the EKF approach for large-scale fleets.5

#### **3.2.1 Theoretical Divergence: Filtering vs. Smoothing**

* **EKF (RL):** Is a recursive filter. It maintains the *current* state belief. When a measurement arrives, it updates the state and the covariance matrix. Past measurements are "baked in" and cannot be revisited. If a GPS packet arrives 500ms late (latency), the EKF assumes it is current or discards it, leading to estimation errors.5  
* **Graph Optimization (Fuse):** Maintains a history of states (nodes) and measurements (constraints/edges) in a graph structure. It solves a non-linear least squares problem to find the trajectory that best minimizes the error across *all* constraints.  
  * *Fixed-Lag Smoothing:* fuse typically operates as a fixed-lag smoother, optimizing a window of the past (e.g., the last 5 seconds). This allows it to handle delayed sensor data elegantly; it simply inserts the late GPS measurement into the correct timestamp in the past and re-optimizes the trajectory, correcting the current state based on refined past knowledge.5

#### **3.2.2 Limitations for Agricultural Retrofits**

Despite its theoretical superiority, fuse presents hurdles for a BCS tractor project:

1. **2D Model Limitation:** Currently, the primary motion models in fuse are prefixed with \_2d, implying they are optimized for planar environments.5 While fields are generally flat, tractors experience significant pitch and roll (3D motion). The EKF in robot\_localization handles 3D states (X, Y, Z, Roll, Pitch, Yaw) natively, provided the sensor configuration is correct.24  
2. **Configuration Complexity:** fuse requires defining constraints and optimization plugins explicitly. It lacks the "drop-in" simplicity of RL's configuration YAMLs for standard GPS integration. The "circular dependency" logic handled internally by navsat\_transform\_node in RL must be manually architected in fuse or handled by external nodes.25

### **3.3 Recommendation**

For this specific application—a retrofitted tractor requiring robust, tutorial-supported implementation—**robot\_localization is the recommended choice.** The maturity of its navsat\_transform\_node and its seamless integration with the Nav2 TF tree outweigh the benefits of graph optimization for a single-agent system. fuse should be considered a Phase 2 upgrade if sensor latency proves problematic during field testing.5

---

## **4\. Navigation Stack (Nav2): Tuning for Unstructured Terrain**

The ROS 2 Navigation Stack (Nav2) is a behavior-tree-based navigation system. While often demonstrated on indoor robots (TurtleBots), its modularity makes it powerful for agriculture if the correct plugins are selected.

### **4.1 Costmap Layers and Environmental Modeling**

The Costmap is the robot's internal map of obstacles. In a farm field, "obstacles" are dynamic (people, animals) and static (fences, trees).

* **Spatio-Temporal Voxel Layer (STVL):** The standard ObstacleLayer is insufficient for tall grass or dust, which might register as permanent obstacles. STVL is a plugin that maintains a 3D voxel grid where obstacles have a "time to live." If a sensor detects a "ghost" obstacle (like a puff of dust), STVL decays it over time, clearing the map automatically. This prevents the tractor from getting stuck in phantom obstacles.27  
* **Static Layer:** This layer should be loaded from a pre-recorded map or a satellite image mask, defining the immutable boundaries of the field and the "keep-out" zones (ponds, ditches).28

### **4.2 The Planner: Smac Planner (Hybrid-A\*)**

The global planner calculates the path from Point A to Point B. The default NavFn planner uses Dijkstra or A\* on a 2D grid, assuming the robot is a circular point that can turn instantly.

* **Kinematic Constraints:** A BCS tractor with an implement has a large **Minimum Turning Radius**. It cannot spin in place. Standard A\* might plan a sharp 90-degree turn that the tractor physically cannot execute.  
* *Smac Hybrid-A:*\* This planner uses kinematic primitives (Dubins or Reeds-Shepp curves) to explore the search space. It generates paths that are guaranteed to be drivable by a vehicle with Ackermann or non-holonomic constraints. It effectively plans "3-point turns" if required to navigate tight corners.29

### **4.3 The Controller: MPPI (Model Predictive Path Integral)**

The controller (local planner) executes the path by sending velocity commands.

* **Why MPPI for Farming?**  
  * Traditional controllers like DWB (Dynamic Window Approach) or TEB (Timed Elastic Band) rely on gradient descent or discrete sampling that assumes a predictable friction model.  
  * **MPPI** uses a sampling-based approach. It projects thousands of randomized trajectories forward in time, utilizing a GPU or multi-core CPU. It scores these trajectories based on the costmap and the global path.  
  * *Slip Recovery:* Because it re-samples continuously (50Hz+), if the tractor slips on mud and deviates from the predicted path, MPPI immediately finds a new set of valid trajectories from the *new* state. It does not get "stuck" trying to force the previous plan. This robustness to model mismatch makes it the premier choice for off-road robotics.6  
* **Tuning MPPI:** The motion\_model should be set to DiffDrive (or Ackermann if using a steering sulky). The batch\_size should be maximized (e.g., 2000\) to ensure a dense search of the control space. The PathAlign critic should be weighted heavily to ensure the tractor sticks to the crop rows.31

---

## **5\. Coverage Path Planning: Automating the "Deep Dive"**

The user's requirement to "select the area on the map to be processed" implies **Coverage Path Planning (CPP)**. This moves beyond navigating *to* a location; it involves visiting *every* point within a polygon.

### **5.1 The fields2cover Library**

fields2cover is an open-source C++ library designed specifically for agricultural CPP. Unlike generic coverage algorithms (like simple boustrophedon), fields2cover understands agricultural constraints.

* **Headland Generation:** It can automatically generate "headlands"—a perimeter path around the field boundary. This area is processed last or first and provides the necessary turning space for the tractor to enter/exit rows without crossing the field boundary (fences).32  
* **Swath Patterns:**  
  * *Boustrophedon:* The standard back-and-forth pattern.  
  * *Spiral:* A pattern that starts outside and spirals in (or vice-versa), minimizing sharp turns.  
  * *Custom Ordering:* The library can optimize the order of rows (e.g., skip-row harvesting) to ensure that every turn is a wide, smooth curve rather than a sharp, damaging pivot.32

### **5.2 Integration: opennav\_coverage**

The opennav\_coverage package serves as the ROS 2 wrapper for fields2cover, exposing its functionality via standard ROS Actions.

* **Architecture:** It provides the ComputeCoveragePath action.  
  * **Input:** geometry\_msgs/Polygon (Field Boundary), geometry\_msgs/Polygon (Obstacles/Holes inside the field).  
  * **Parameters:** robot\_width (implement width), min\_turning\_radius (critical for BCS), headland\_width.8  
  * **Output:** nav\_msgs/Path. This path is a dense list of poses covering the entire field.  
* **Row Coverage:** A distinct module, opennav\_row\_coverage, exists for navigating pre-defined crop rows (e.g., in vineyards), utilizing fields2cover to generate U-turns connecting the rows.8

---

## **6\. Frontend Implementation: Web-Based Mission Planner**

To enable the user to "select the area," a custom user interface is required. While tools like RViz exist, they are developer-centric. A web-based interface accessible via a tablet is the industry standard for farm operations.

### **6.1 Technology Stack: Leaflet and Rosbridge**

The recommended architecture leverages standard web technologies bridged to ROS 2\.

* **Leaflet.js:** A lightweight, open-source JavaScript library for interactive maps. It supports tiling services (OpenStreetMap, Google Satellite) which provide the visual context needed for a farmer to identify field boundaries.9  
* **Leaflet.draw:** A plugin for Leaflet that adds a toolbar allowing users to draw polygons, rectangles, and markers directly on the map.10  
* **rosbridge\_server:** A ROS node that creates a WebSocket (default port 9090). It acts as a gateway, serializing ROS messages into JSON for the web client and deserializing JSON from the web client into ROS messages.9  
* **roslibjs:** The client-side JavaScript library that connects to rosbridge. It allows the web page to publish topics and call services just like a native C++ or Python node.36

### **6.2 The Coordinate Transformation Pipeline**

A critical challenge is the disparity in coordinate systems. Leaflet uses **WGS84** (Latitude/Longitude). ROS navigation uses **Cartesian Meters** (Map/Odom frames).

**Table 2: The Polygon Data Flow**

| Step | Component | Data Format | Description |
| :---- | :---- | :---- | :---- |
| 1 | **Web UI (Leaflet.draw)** | Array of L.LatLng objects | User draws a polygon on the satellite map. The vertices are geodetic coordinates (e.g., 45.123, \-93.456). |
| 2 | **Web UI (roslibjs)** | JSON (serialized geometry\_msgs/Polygon) | The JS code extracts the Lat/Lon points. Since standard ROS messages utilize x, y, z, the Lat is mapped to x and Lon to y (a temporary convention). This message is published to /gui/field\_boundary\_gps. |
| 3 | **Transformation Node** | geometry\_msgs/Polygon (Lat/Lon) | A custom Python node receives the message. It iterates through the points. |
| 4 | **Robot Localization Service** | Service Call: fromLL | The node calls the robot\_localization service fromLL (From Lat-Long). This service uses the NavSat datum to convert the geodetic point into the robot's map frame (e.g., X=150.5m, Y=20.3m).4 |
| 5 | **Coverage Planner** | geometry\_msgs/Polygon (Meters) | The transformed polygon is sent to the opennav\_coverage action server. |
| 6 | **Path Visualization** | nav\_msgs/Path (Meters) | The planner returns a path. This must be converted *back* to Lat/Lon (via toLL service) if it is to be displayed on the Leaflet map, OR visualized in RViz. |

### **6.3 Vizanti: An Alternative**

For users seeking a "turn-key" web solution, **Vizanti** offers a robust alternative to building a custom Leaflet app.

* **Architecture:** Vizanti is a web-based visualizer that runs a Flask backend on the robot. It serves a web app that connects via rosbridge.37  
* **Features:** It replicates the functionality of RViz in a browser, supporting the visualization of TF trees, costmaps, and path plans.  
* **Customization:** Vizanti supports custom widgets. A developer can add a "Mission Planner" widget that publishes the required polygon topics, leveraging the existing rosbridge infrastructure provided by the Vizanti container.38

---

## **7\. Simulation and Validation**

Before deploying a heavy mechanical tractor, simulation in Gazebo is mandatory to validate the kinematic model and control logic.

### **7.1 URDF Modeling and Physics**

The URDF (Unified Robot Description Format) must accurately represent the physical properties of the tractor.

* **Trailer Dynamics:** A BCS tractor with a sulky is an articulated vehicle. The connection between the tractor unit and the sulky must be modeled as a revolute joint (yaw only) or a spherical joint if the hitch allows roll/pitch.  
* **Friction Parameters:** In Gazebo, the mu1 and mu2 friction coefficients of the wheels should be tuned to simulate soil (lower friction than asphalt). This allows the developer to test if the MPPI controller can successfully recover from sliding events.39

### **7.2 Sensor Simulation**

Standard Gazebo plugins should be used to simulate the sensor suite:

* **gazebo\_ros\_gps\_sensor:** Configured to output sensor\_msgs/NavSatFix.  
* **gazebo\_ros\_imu\_sensor:** Configured with Gaussian noise to simulate engine vibration. *Crucially*, the initialOrientationAsReference parameter should be set to false to comply with ROS standards (REP 145), ensuring the IMU reports absolute orientation relative to a fixed world frame (magnetic north simulation).40

---

## **8\. Conclusion**

The retrofit of a BCS two-wheel tractor into an autonomous farming platform is a feasible and high-impact engineering challenge that leverages the maturity of the ROS 2 ecosystem.

By adopting a **Dual-EKF architecture** with robot\_localization, the system gains robust global positioning capabilities essential for large-scale field operations. The integration of the **Nav2 MPPI controller** addresses the specific non-linear control challenges posed by loose terrain and the "brake-steer" mechanics of the tractor. Finally, the synthesis of **fields2cover** for intelligent path planning and a **Leaflet-based web frontend** creates a user-centric workflow that empowers operators to define complex coverage tasks intuitively.

This architectural approach transitions the BCS tractor from a passive mechanical tool into an intelligent robotic agent, capable of precision agriculture tasks that reduce labor and increase operational efficiency.

#### **Works cited**

1. Tractors: Differential Drive \- BCS America, accessed on November 21, 2025, [https://www.bcsamerica.com/products/tractors/differential-drive](https://www.bcsamerica.com/products/tractors/differential-drive)  
2. Model 779 (PS) Hydrostatic \- BCS America, accessed on November 21, 2025, [https://www.bcsamerica.com/product/model-779-ps-hydrostatic](https://www.bcsamerica.com/product/model-779-ps-hydrostatic)  
3. ROS 2 Autonomous Tractor with PID Control \- GitHub, accessed on November 21, 2025, [https://github.com/sushant097/Ros2-Autonomous-Tractor](https://github.com/sushant097/Ros2-Autonomous-Tractor)  
4. Integrating GPS Data \- robot\_localization \- ROS documentation, accessed on November 21, 2025, [http://docs.ros.org/en/noetic/api/robot\_localization/html/integrating\_gps.html](http://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html)  
5. Concerns regarding robot\_localization deprecation \- Open Robotics Discourse, accessed on November 21, 2025, [https://discourse.openrobotics.org/t/concerns-regarding-robot-localization-deprecation/33512](https://discourse.openrobotics.org/t/concerns-regarding-robot-localization-deprecation/33512)  
6. Model Predictive Path Integral Controller — Nav2 1.0.0 documentation, accessed on November 21, 2025, [https://docs.nav2.org/configuration/packages/configuring-mppic.html](https://docs.nav2.org/configuration/packages/configuring-mppic.html)  
7. nav2\_mppi\_controller 1.2.10 documentation, accessed on November 21, 2025, [https://docs.ros.org/en/iron/p/nav2\_mppi\_controller/](https://docs.ros.org/en/iron/p/nav2_mppi_controller/)  
8. open-navigation/opennav\_coverage: Nav2 Compatible Complete Cover Task Server, Navigator, & BT Utils \- GitHub, accessed on November 21, 2025, [https://github.com/open-navigation/opennav\_coverage](https://github.com/open-navigation/opennav_coverage)  
9. Using Rosbridge with ROS 2 \- Foxglove, accessed on November 21, 2025, [https://foxglove.dev/blog/using-rosbridge-with-ros2](https://foxglove.dev/blog/using-rosbridge-with-ros2)  
10. Leaflet Draw Documentation, accessed on November 21, 2025, [https://leaflet.github.io/Leaflet.draw/docs/leaflet-draw-latest.html](https://leaflet.github.io/Leaflet.draw/docs/leaflet-draw-latest.html)  
11. RC Tractor Conversion, accessed on November 21, 2025, [https://www.lainefamily.com/RCTractor.html](https://www.lainefamily.com/RCTractor.html)  
12. Differential Drive and Steering Control Technology of Automatic Navigation Handling Tool Based on PLC, accessed on November 21, 2025, [https://ijomam.com/wp-content/uploads/2017/12/134-138\_DIFFERENTIAL-DRIVE-AND-STEERING-CONTROL-TECHNOLOGY-OF-AUTOMATIC-NAVIGATION.pdf](https://ijomam.com/wp-content/uploads/2017/12/134-138_DIFFERENTIAL-DRIVE-AND-STEERING-CONTROL-TECHNOLOGY-OF-AUTOMATIC-NAVIGATION.pdf)  
13. Dualsteer® steering system \- Bcs, accessed on November 21, 2025, [https://bcsagri.com/en-001/technology/dualsteer/](https://bcsagri.com/en-001/technology/dualsteer/)  
14. DIY Linear Servo Actuator, 3D Printed \- YouTube, accessed on November 21, 2025, [https://www.youtube.com/watch?v=2vAoOYF3m8U](https://www.youtube.com/watch?v=2vAoOYF3m8U)  
15. Building an Inexpensive SSQA Remote Control \- Part 2 (\#182) \- YouTube, accessed on November 21, 2025, [https://www.youtube.com/watch?v=EcrhnMC6fIE](https://www.youtube.com/watch?v=EcrhnMC6fIE)  
16. How to build your own tractor autosteer system for £700 \- Farmers Weekly, accessed on November 21, 2025, [https://www.fwi.co.uk/machinery/technology/how-to-build-your-own-tractor-autosteer-system-for-700](https://www.fwi.co.uk/machinery/technology/how-to-build-your-own-tractor-autosteer-system-for-700)  
17. Operating a Linear Actuator with RC Controls using a LAC Board and ESC \- YouTube, accessed on November 21, 2025, [https://www.youtube.com/watch?v=EO-0Ea-m0BE](https://www.youtube.com/watch?v=EO-0Ea-m0BE)  
18. DIY Linear Actuator: Full Guide \- YouTube, accessed on November 21, 2025, [https://www.youtube.com/watch?v=3CXg9WTjNEA](https://www.youtube.com/watch?v=3CXg9WTjNEA)  
19. Autonomous Riding Lawnmower Update \- News \- SparkFun Electronics, accessed on November 21, 2025, [https://news.sparkfun.com/3427](https://news.sparkfun.com/3427)  
20. Sensor Fusion Using the Robot Localization Package – ROS 2 \- AutomaticAddison.com, accessed on November 21, 2025, [https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)  
21. navsat\_transform documentation diagram · Issue \#550 · cra-ros-pkg/robot\_localization, accessed on November 21, 2025, [https://github.com/cra-ros-pkg/robot\_localization/issues/550](https://github.com/cra-ros-pkg/robot_localization/issues/550)  
22. Tractobots, my attempts at field robots \- Page 3 \- Projects \- Open Robotics Discourse, accessed on November 21, 2025, [https://discourse.openrobotics.org/t/tractobots-my-attempts-at-field-robots/1486?page=3](https://discourse.openrobotics.org/t/tractobots-my-attempts-at-field-robots/1486?page=3)  
23. navsat\_transform\_node — robot\_localization 2.6.12 documentation, accessed on November 21, 2025, [http://docs.ros.org/en/melodic/api/robot\_localization/html/navsat\_transform\_node.html](http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html)  
24. How to properly fuse IMU \+ Barometer \+ ToF Ranger? (Robot\_Localization), accessed on November 21, 2025, [https://robotics.stackexchange.com/questions/104866/how-to-properly-fuse-imu-barometer-tof-ranger-robot-localization](https://robotics.stackexchange.com/questions/104866/how-to-properly-fuse-imu-barometer-tof-ranger-robot-localization)  
25. Fuse GPS with IMU robot\_localization \- ROS Answers archive, accessed on November 21, 2025, [https://answers.ros.org/question/331762/](https://answers.ros.org/question/331762/)  
26. Sensor Fusion and Robot Localization Using ROS 2 Jazzy \- YouTube, accessed on November 21, 2025, [https://www.youtube.com/watch?v=XOQTF38lmtE](https://www.youtube.com/watch?v=XOQTF38lmtE)  
27. Using an External Costmap Plugin (STVL) — Nav2 1.0.0 documentation, accessed on November 21, 2025, [https://docs.nav2.org/tutorials/docs/navigation2\_with\_stvl.html](https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html)  
28. Costmap 2D — Nav2 1.0.0 documentation, accessed on November 21, 2025, [https://docs.nav2.org/configuration/packages/configuring-costmaps.html](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)  
29. nav2\_smac\_planner 1.2.10 documentation, accessed on November 21, 2025, [https://docs.ros.org/en/iron/p/nav2\_smac\_planner/](https://docs.ros.org/en/iron/p/nav2_smac_planner/)  
30. How to configure Nav2's SMAC Planner to maintain a safer distance from obstacles?, accessed on November 21, 2025, [https://robotics.stackexchange.com/questions/111025/how-to-configure-nav2s-smac-planner-to-maintain-a-safer-distance-from-obstacles](https://robotics.stackexchange.com/questions/111025/how-to-configure-nav2s-smac-planner-to-maintain-a-safer-distance-from-obstacles)  
31. MPPI Controller tuning tips to fix bad performance on differential drive robots in close maneuvering goals · Issue \#5375 · ros-navigation/navigation2 \- GitHub, accessed on November 21, 2025, [https://github.com/ros-navigation/navigation2/issues/5375](https://github.com/ros-navigation/navigation2/issues/5375)  
32. Tutorials — Fields2Cover latest documentation, accessed on November 21, 2025, [https://fields2cover.github.io/source/tutorials.html](https://fields2cover.github.io/source/tutorials.html)  
33. fields2cover 2.0.0 documentation, accessed on November 21, 2025, [https://docs.ros.org/en/iron/p/fields2cover/](https://docs.ros.org/en/iron/p/fields2cover/)  
34. Coverage Server — Nav2 1.0.0 documentation, accessed on November 21, 2025, [https://docs.nav2.org/configuration/packages/configuring-coverage-server.html](https://docs.nav2.org/configuration/packages/configuring-coverage-server.html)  
35. AndreasOlausson/leaflet-polydraw: Advanced Leaflet plugin for interactive polygon drawing with point-to-point creation, smart merging, and comprehensive editing tools \- GitHub, accessed on November 21, 2025, [https://github.com/AndreasOlausson/leaflet-polydraw](https://github.com/AndreasOlausson/leaflet-polydraw)  
36. ros2djs/Tutorials/VisualizingAMap \- ROS Wiki, accessed on November 21, 2025, [https://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap](https://wiki.ros.org/ros2djs/Tutorials/VisualizingAMap)  
37. MoffKalast/vizanti: A mission planner and visualizer for controlling outdoor ROS robots., accessed on November 21, 2025, [https://github.com/MoffKalast/vizanti](https://github.com/MoffKalast/vizanti)  
38. Announcing Vizanti, a web visualizer & mission planner for ROS \- Open Robotics Discourse, accessed on November 21, 2025, [https://discourse.openrobotics.org/t/announcing-vizanti-a-web-visualizer-mission-planner-for-ros/31664](https://discourse.openrobotics.org/t/announcing-vizanti-a-web-visualizer-mission-planner-for-ros/31664)  
39. vehicle trailer simulation in gazebo \- Robotics Stack Exchange, accessed on November 21, 2025, [https://robotics.stackexchange.com/questions/25128/vehicle-trailer-simulation-in-gazebo](https://robotics.stackexchange.com/questions/25128/vehicle-trailer-simulation-in-gazebo)  
40. Tutorial : Gazebo plugins in ROS, accessed on November 21, 2025, [https://classic.gazebosim.org/tutorials?tut=ros\_gzplugins](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)