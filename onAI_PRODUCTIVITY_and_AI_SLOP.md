# AI-Assisted ROS Development: Productivity With or Without Slop?

## The Core Tension

AI coding tools are genuinely incredibly useful. They can and will produce goodl-looking garbage that passes a cursory glance but fails in production, especially in robotics where bad code can physically damage hardware or injure people. The goal is to use AI as a force multiplier and a productivity booster while keeping the human as the architect, reviewer, and the ultimate safety net.

The curret meta puts it well: **~30% speed increase from proposal to production**. Not by letting AI drive, but by using it at specific stages where it excels and keeping humans in control everywhere necessary.

---

## 1. Where AI Actually Helps in ROS 2 Development

### 1.1 Boilerplate Generation (High Value, Low Risk)

ROS 2 has enormous amounts of boilerplate. AI is excellent at generating it:

- **Node scaffolding**: Lifecycle node with all 5 transition callbacks, publisher/subscriber setup, parameter declarations. This is 80+ lines of always the same structure every time.
- **CMakeLists.txt and package.xml**: Dependency declarations, install targets, ament macros. Tedious, hard by hand, and AI gets it zero-shot.
- **Launch files**: Python launch files with parameter loading, remappings.
- **Message/Service/Action definitions**: You describe what you need in natural language and the `.msg`/`.srv`/`.action` file is trivial to generate.
- **Parameter callback boilerplate**: The `on_set_parameters_callback` pattern with validation logic.

This is where you get the biggest ROI. The code is standard, the patterns are standardized and everyone uses them, and mistakes are caught immediately by the `colcon build`.

### 1.2 Test Writing (Highest Value Add, Medium to High Risk)

This mirrors the TDD approach: **have AI write the tests first, have the human understand, veify, scrutinize, fix and optimize the tests, then use AI to build the feature that passes the tests.**

For ROS 2, this means:
- Unit tests for your core logic class (the non-ROS part you should have separated out).
- Integration tests using `launch_testing` that spin up nodes and verify topic output.
- Parameter validation tests (set crazy parameterds does the node reject negative frequencies, negative accelerations..? Does it std::clamp out-of-range values?).

AI is good or maybe even exceptional at generating test structure and sometimes even edge cases you might not think of. But **you must read every test** — an AI-generated test that always passes is worse than no test!!

### 1.3 Documentation and Comments (High Value, Low Risk)

- Generating README templates for packages (the henki format: description, usage, API, parameters).
- Writing docstrings for classes and methods.
- Explaining complex callback chains distirbuted among many different components.

### 1.4 Debugging Assistance (Medium Value, Low Risk)

- "Why does this QoS configuration cause message drops?" — AI can explain QoS compatibility rules faster than reading official ROS DDS specs.
- "What does this CMake error mean?" — CMake errors are hard. AI decodes them exceptionally well.
- Explaining `tf2` transform chain issues.
- Parsing and interpreting `ros2 doctor --report` output.

---

## 2. Where AI Will Hurt You in ROS 2

### 2.1 Architecture and System Design

AI does not understand your robot. It does not know that your actuator has a 50ms latency, that your GPS drops out next to a building, that you hardware has physical limits, that your IMU drifts 2 degrees per minute. These physical constraints drive every architectural decision.

**Never let AI design your node graph, topic structure, or state machine.** This is the technical design document phase — it must come from engineers who understand the hardware, the environment, and the failure situations.

The AI driven Development process developed by META captures this right:
1. Technical design document (human)
2. Design review by senior engineers (human)
3. Subsystem documentation (human)
4. Sprint planning (human)
5. Implementation (AI-assisted)
6. Code review (human + AI-assisted)
7. Testing (human)

AI enters at step 5. Not step 1. Pleaese keep this in mind

### 2.2 Concurrency and Real-Time Code

For me AI frequently generates race conditions, especially in:
- `MultiThreadedExecutor` callback groups (mutually exclusive vs. reentrant) Avoid MultiThreadedExecutor if you can (at all cost ONLY IF YOU CAN).
- Shared state between timer callbacks and subscription callbacks.
- Hardware driver code that mixes ROS callbacks with blocking I/O interrupts pyserial / dangerous rclpy mixing.

If AI generates code using `MultiThreadedExecutor`, treat it as **wrong until proven otherwise**. Use `SingleThreadedExecutor` designs and only introduce threading when you can prove it's necessary and correct or you are convering a ROS1 design that by defaults assumes `MultiThreadedExecutor`.
### 2.3 Safety-Critical Logic

AI must never be the designer for:
- Emergency stop logic.
- Velocity/acceleratio/force/torque limiting.
- Watchdog timers.
- Data stream validation and fault detection.
- Anything that, if wrong, causes the robot to collide with or cause harm to something.

Write this by hand. Review it by hand. Test it by hand.

### 2.4 DDS and Networking Configuration

AI training data is full of outdated or wrong DDS configurations. Common AI mistakes:
- Getting QoS profiles wrong for image transport vs. control commands.
- Ignoring `ROS_LOCALHOST_ONLY`, or Zenoh configurations for multi-machine setups.

---

## 3. The Workflow: AI-Assisted ROS 2 Development Done Right

### Step 1: Design First (No AI)

Before touching any code:
- Draw the node graph on paper or whiteboard.
- Define every topic, service, and action interface with types and QoS.
- Identify which nodes need lifecycle management unconditionally.
- Sketch the TF tree.
- Write down failure modes and how each node handles them.

This is your technical design document. It's the single most important artifact in the project.

### Step 2: Generate Scaffolding (AI)

With the design locked in, use AI to generate project structure and all the boilerplate associated with it:
```
gnss_navigation_stack/
  gnss_navigation/           # Pure C++ logic (AI generates skeleton/template)
    include/gnss_navigation/
      imu_gnss_tracker.h
    src/
      imu_gnss_tracker.cpp
    CMakeLists.txt           # AI generates
    package.xml              # AI generates
  gnss_navigation_ros/       # ROS wrappers (AI generates boilerplate)
    include/gnss_navigation_ros/
      imu_gnss_tracker_ros.h
    src/
      imu_gnss_tracker_ros.cpp
    launch/
      imu_gnss_tracker.launch.py # AI generates 
    param/
      imu_gnss_tracker.yaml      # AI generates from your parameter list
    CMakeLists.txt           # AI generates
    package.xml              # AI generates
```

Review every file. The structure should match your design document exactly.

### Step 3: Write Tests First (AI-Assisted)

Following the TDD Bible approach:

1. Give AI your core class interface (the header file).
2. Have it generate comprehensive unit tests.
3. Read every test. Add edge cases if it missed any. Remove tests that don't make sense (80% of the tests will not make sense).
4. Run the tests — they should all fail first (no code implemented yet).

For ROS integration tests:
1. Give AI the node's expected topic/service API from your design doc.
2. Have it generate `launch_testing` tests that verify the node publishes to the right topics, responds to all services, and handles parameter changes.

### Step 4: Implement (AI-Assisted)

Now let AI help implement the actual logic. Key rules:

- **Work in small chunks.** One function at a time, not an entire node, Ai will want to do it all at once, this is a horrible practice.
- **Core logic first, ROS wrapper second.** Implement `NavigationCore` methods one by one, running unit tests after each. Then wire up the ROS node, the ROS wrappes should only consume 10 - 20% of your node development time.
- **Compile after every change.** `colcon build --packages-select my_pkg` after every meaningful edit. Don't accumulate 200 lines of AI-generated code before checking if it compiles.
- **Never accept code you don't understand.** If AI generates a template solution and you can't explain what it does, yet alone decipher the compiler errors it throws, reject asap and and design a simpler bottom-up approach.

### Step 5: Review (Human + AI)

- Use AI to assist code review: "Does this callback handle the case where the transform is not yet available?"
- The final approval is always human.
- Check for the common AI mistakes: unnecessary `shared_ptr` copies, missing `const&`, raw `new/delete`, wrong QoS, blocking calls in callbacks, mixing `new` with `std::make_unique<mcb_node>()` .

### Step 6: Integration Test on Hardware (No AI)

Run it on the real robot. Always. AI cannot and never will test this for you.

---

## 4. Recognizing AI Slop in ROS 2 Code

AI slop is code that looks correct but reveals shallow understanding. Common tells in ROS 2:

| Slop Pattern | What It Looks Like | Why It's Wrong |
|---|---|---|
| Default QoS everywhere | `rclcpp::QoS(10)` on every pub/sub | Sensor data should be BEST_EFFORT, commands RELIABLE. One size doesn't fit all. |
| Gratuitous `shared_ptr` | `std::shared_ptr<int> count_` | A plain `int` member works fine. AI loves smart pointers even when tottally unnecessary. |
| Copy-paste error handling | `RCLCPP_ERROR(... "Failed to do X")` then continue normally | If it truly failed, the function should return, throw, or transition the lifecycle node. |
| String-based message discrimination | `if (msg->data == "start")` on a `std_msgs/String` | Use a service, action, or proper enum message. |
| `sleep()` in callbacks | `std::this_thread::sleep_for(...)` inside a subscription callback | This blocks the executor. Use timers or async patterns. |
| Unnecessary multithreading | `MultiThreadedExecutor` + mutex everywhere | If you need a mutex in every callback, the design is wrong. |
| Monolithic `on_activate()` | 200 lines of setup in a single lifecycle callback | Break it into helper functions. AI tends to dump everything in one place. |
| Missing `const&` on callbacks | `void callback(sensor_msgs::msg::Image msg)` | Copies the entire image. Must be `const SharedPtr&`. |
| Timer + subscriber doing the same thing | A timer that polls + a subscriber that reacts | Pick one pattern. AI sometimes generates both without realizing they conflict. |

---

## 5. The Final Verdict

**AI is a junior developer with perfect syntax and zero judgment.**

It will write code that compiles. It will write code that looks professional. It will write code that passes a quick review. But it does not understand:
- Why your robot needs 200Hz IMU updates but only 10Hz path planning.
- Why `BEST_EFFORT` QoS is correct for LiDAR but would be catastrophic for emergency stop.
- Why a 50ms delay in `cmd_vel` is acceptable but a 50ms delay in `e_stop` is not.
- Why your TF tree has `base_link → laser_link` but not `base_link → camera_link → laser_link`.

These are engineering decisions born from understanding the physical system and the world the robot interacts with. No amount of training data (coding data across github) gives an AI this understanding. Your job is to bring the judgment. AI's job is to bring the syntax to life faster.

**Always start with a solid design doc architecture. Build in small chunks. write tests first.**
