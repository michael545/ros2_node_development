# ADR 0001: Language Choice Strategy for ROS2 Development

## Status
**Accepted** ✅

## Context

As we expand this ROS2 developer guide to cover multiple languages (C++, Python, Rust), we need a clear strategy for:

1. **When to use each language** in professional ROS2 development
2. **How to organize** multi-language examples in the repository
3. **Maintaining consistency** across different language implementations
4. **Documenting best practices** for each language

## Decision

### 1. Repository Organization Strategy

**Adopt a layered architecture:**

```
docs/
├── language_choice/          # Language comparison and guidance
│   ├── README.md             # Decision matrix and recommendations
│   ├── cpp/                  # C++ specific patterns and examples
│   ├── python/               # Python specific patterns and examples
│   └── rust/                 # Rust specific patterns and examples
examples/
├── basic_pub_sub/           # Core examples (primarily C++)
├── language_examples/        # Language-specific implementations
│   ├── cpp/                  # C++ versions of examples
│   ├── python/               # Python versions of examples  
│   └── rust/                 # Rust versions of examples
└── real_world_applications/  # Complex multi-language systems
```

### 2. Language Selection Guidelines

**Primary Language Recommendations:**

| Application Type          | Primary Language | Secondary Language | Reasoning |
|---------------------------|------------------|--------------------|-----------|
| **Real-time Control**     | C++              | Rust               | Deterministic performance, low-level control |
| **Sensor Processing**     | C++              | Rust               | High throughput, memory efficiency |
| **Path Planning**        | Python           | C++                | Algorithm flexibility, rapid iteration |
| **Perception**           | Python           | C++                | ML integration, data processing |
| **High-Level Coordination** | Python        | -                  | Rapid development, scripting flexibility |
| **Safety-Critical**      | Rust             | C++                | Memory safety, reliability |

### 3. Implementation Consistency Rules

**Mandatory Consistency Requirements:**

1. **Identical Interfaces**: All language implementations must expose the same ROS2 interfaces (topics, services, actions)
2. **Equivalent Functionality**: Core algorithms must produce equivalent results (within numerical tolerance)
3. **Consistent Error Handling**: Similar error conditions must be handled consistently
4. **Unified Configuration**: Use identical parameter names and structures

**Example Interface Consistency:**

```cpp
// C++ Service
class IKService : public rclcpp::Node {
public:
    IKService();
    void handle_ik_request(const SolveIK::Request& req, SolveIK::Response& res);
};
```

```python
# Python Service
class IKService(Node):
    def __init__(self):
        super().__init__('ik_solver')
    
    def handle_ik_request(self, request, response):
        # Same logic as C++ version
```

```rust
// Rust Service
struct IKService {
    node: Node,
}

impl IKService {
    fn new() -> Self { /* ... */ }
    fn handle_ik_request(&self, req: SolveIKRequest, res: &mut SolveIKResponse) { /* ... */ }
}
```

### 4. Documentation Standards

**Documentation Requirements:**

1. **Language-Specific READMEs**: Each language folder must contain:
   - Setup instructions
   - Build instructions
   - Common patterns and anti-patterns
   - Performance characteristics
   - Debugging tips

2. **Cross-Language Comparison Tables**:

| Feature                | C++ Implementation | Python Implementation | Rust Implementation |
|------------------------|--------------------|-----------------------|---------------------|
| **Performance**        | ⭐⭐⭐⭐⭐          | ⭐⭐⭐                 | ⭐⭐⭐⭐⭐           |
| **Memory Safety**      | ⭐⭐                | ⭐⭐⭐⭐               | ⭐⭐⭐⭐⭐           |
| **Development Speed**  | ⭐⭐⭐               | ⭐⭐⭐⭐⭐             | ⭐⭐⭐⭐              |

3. **Migration Guides**: Provide clear paths between languages

### 5. Build System Integration

**Multi-Language Build Strategy:**

1. **C++**: Standard `ament_cmake` build
2. **Python**: Standard `ament_python` build
3. **Rust**: Custom build integration using `cargo`

**Build Configuration:**

```cmake
# CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# C++ targets
add_executable(cpp_node src/cpp_node.cpp)

# Python targets (handled by setup.py)

# Rust targets
execute_process(
    COMMAND cargo build --release
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
)
```

### 6. Testing Strategy

**Cross-Language Testing Requirements:**

1. **Interface Testing**: Verify all language implementations produce compatible messages
2. **Performance Benchmarking**: Measure and document performance differences
3. **Integration Testing**: Test mixed-language systems
4. **Regression Testing**: Ensure updates don't break cross-language compatibility

**Test Example:**

```python
# test_interface_compatibility.py
def test_ik_interface_compatibility():
    # Test that C++, Python, and Rust IK solvers produce equivalent results
    test_cases = [...]
    
    for case in test_cases:
        cpp_result = call_cpp_ik_solver(case)
        python_result = call_python_ik_solver(case)
        rust_result = call_rust_ik_solver(case)
        
        assert results_equivalent(cpp_result, python_result, tolerance=1e-4)
        assert results_equivalent(cpp_result, rust_result, tolerance=1e-4)
```

## Consequences

### Positive Consequences

1. **Comprehensive Learning Resource**: Developers can learn ROS2 patterns in their preferred language
2. **Production Flexibility**: Teams can choose the right language for each component
3. **Performance Optimization**: Critical components can be implemented in performance-oriented languages
4. **Safety Enhancement**: Safety-critical components can use memory-safe languages
5. **Cross-Language Best Practices**: Establishes patterns for mixed-language ROS2 systems

### Negative Consequences

1. **Increased Maintenance Burden**: Multiple language implementations require more maintenance
2. **Build Complexity**: Mixed-language builds are more complex to configure
3. **Documentation Overhead**: Requires comprehensive documentation for each language
4. **Testing Complexity**: Cross-language testing adds complexity
5. **Expertise Requirements**: Team needs expertise in multiple languages

### Mitigation Strategies

1. **Automated Testing**: Comprehensive CI/CD pipelines for all languages
2. **Documentation Templates**: Standardized templates for each language
3. **Build Automation**: Scripts to handle multi-language builds
4. **Code Generation**: Where possible, generate boilerplate code
5. **Community Contributions**: Encourage community contributions for language-specific examples

## Alternatives Considered

### Alternative 1: Single Language Focus
**Rejected because:** Limits the guide's usefulness to developers with different language preferences and requirements.

### Alternative 2: Language-Specific Repositories
**Rejected because:** Fragmentation would make cross-language comparison difficult and increase maintenance overhead.

### Alternative 3: Minimal Multi-Language Support
**Rejected because:** Would not provide sufficient guidance for production multi-language systems.

## Future Considerations

1. **Additional Languages**: Consider adding Go, Java, or other languages based on community demand
2. **Language Interoperability**: Explore FFI and other interoperability mechanisms
3. **Performance Optimization**: Develop language-specific optimization guides
4. **Tooling Integration**: Integrate with language-specific tooling (pylint, clippy, etc.)

## References

- ROS2 Documentation: Multi-language support
- Real-time systems programming guides
- Sensor fusion algorithm references
- Kalman filter implementations across languages

**Approved by:** @michael
**Date:** 2024-03-15
**Last Updated:** 2024-03-15