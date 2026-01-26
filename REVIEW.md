# Repository Structure Review and Improvement Recommendations

**Date**: January 2026  
**Repository**: LCAS/topological_navigation  
**Version**: 3.0.5 (ROS 2 Humble/Iron)

This document provides an analysis of the current repository structure, identifies potential issues, and recommends improvements to guide future development and AI-assisted contributions.

---

## Executive Summary

The topological_navigation repository is a mature ROS 2 project with a well-established foundation. However, analysis reveals several areas for improvement:

1. **Code Organization**: Presence of legacy "version 2" files alongside original versions creates confusion
2. **Documentation**: Inconsistent documentation coverage across packages
3. **Testing**: Limited test coverage, especially for newer ROS 2 features
4. **Code Duplication**: Some functionality exists in multiple versions (v1 vs v2)
5. **Technical Debt**: Several TODOs and FIXMEs in critical navigation code

**Overall Assessment**: The repository is functional and well-maintained, but would benefit from modernization and consolidation efforts.

---

## Structural Analysis

### 1. Code Organization and Versioning

**Issue: Multiple File Versions**

The repository contains **15+ files with "2" suffix** (e.g., `localisation2.py`, `manager2.py`, `route_search2.py`), indicating parallel implementations:

```
topological_navigation/
├── manager.py          # Legacy version
├── manager2.py         # Current version (1538 lines)
├── route_search.py     # Legacy version
├── route_search2.py    # Current version
├── localisation.py     # Legacy version
└── scripts/
    └── localisation2.py  # Current version
```

**Impact**:
- **Confusion for Contributors**: Unclear which files are authoritative
- **Maintenance Burden**: Bug fixes may need to be applied to multiple versions
- **Code Bloat**: Doubles the codebase size unnecessarily
- **AI Agent Confusion**: AI tools may reference or modify deprecated code

**Recommendations**:

1. **Short-term (High Priority)**:
   - Add clear deprecation notices in legacy files:
     ```python
     """
     DEPRECATED: This file is maintained for backward compatibility only.
     New development should use manager2.py instead.
     Will be removed in version 4.0.0.
     """
     ```
   - Update documentation to clarify which versions to use

2. **Medium-term**:
   - Create migration guide for users still on legacy versions
   - Rename "2" files to remove suffix once legacy is removed:
     - `manager2.py` → `manager.py`
     - `route_search2.py` → `route_search.py`
   - Consider semantic versioning for major API changes instead of file suffixes

3. **Long-term (Version 4.0.0)**:
   - Remove all legacy files after deprecation period
   - Consolidate to single implementation per module

### 2. Package Structure

**Current Structure**: Well-organized multi-package workspace

```
topological_navigation/
├── topological_navigation/      # Core (Python, ~8k LOC)
├── topological_navigation_msgs/ # Messages
├── topological_rviz_tools/      # RViz tools (C++/Qt)
└── topological_utils/          # Utilities (Python, ~70+ scripts)
```

**Strengths**:
- Clear separation of concerns
- Standard ROS 2 package layout
- Logical grouping of functionality

**Issues**:

1. **Script Proliferation**: `topological_utils` contains **70+ Python scripts**, making it difficult to discover and maintain tools
   
2. **Inconsistent Naming**: Some scripts use underscores, others don't; some have `.py` extension in the name, others don't

**Recommendations**:

1. **Organize Utilities by Function**:
   ```
   topological_utils/
   └── topological_utils/
       ├── commands/          # CLI tools
       │   ├── map_management/
       │   │   ├── add_node.py
       │   │   ├── remove_node.py
       │   │   └── crop_map.py
       │   ├── visualization/
       │   │   ├── plot_map.py
       │   │   └── draw_predictions.py
       │   └── migration/
       │       ├── migrate.py
       │       └── map_converter.py
       └── lib/              # Shared library code
   ```

2. **Create Unified CLI Tool**:
   - Consider a single entry point: `topo` command with subcommands
   - Example: `topo map add-node` instead of `add_node.py`
   - Reduces cognitive load and improves discoverability
   - Similar to `git`, `docker`, or `ros2` command structure

3. **Consolidate Duplicate Functionality**:
   - Multiple map plotting tools exist (`plot_yaml.py`, `plot_yaml2.py`, `plot_topo_map.py`, `plot_topo_map2.py`)
   - Consider single `plot` command with options for different formats

### 3. Documentation

**Current State**:

| Package | README Length | Quality |
|---------|---------------|---------|
| Root | 14 lines | Minimal - just package list |
| topological_navigation | 278 lines | Good - comprehensive |
| topological_rviz_tools | 79 lines | Good - usage examples |
| topological_utils | None | **Missing** |
| topological_navigation_msgs | None | **Missing** |

**Issues**:

1. **Missing Package READMEs**: Two packages lack dedicated documentation
2. **Outdated Content**: Main README references ROS 1 concepts (MongoDB, `roslaunch`)
3. **No Architecture Diagrams**: Complex system lacks visual documentation
4. **API Documentation**: No generated API docs (Sphinx, Doxygen)

**Recommendations**:

1. **Immediate (High Priority)**:
   - Add READMEs to `topological_utils` and `topological_navigation_msgs`
   - Update root README with:
     - Quick start guide for ROS 2
     - System requirements
     - Installation instructions
     - Link to key documentation

2. **Short-term**:
   - Create `docs/` directory with:
     - Architecture overview with diagrams (use Mermaid or PlantUML)
     - Component interaction diagrams
     - Data flow diagrams
     - API reference (auto-generated from code)
   
3. **Medium-term**:
   - Set up automated documentation generation:
     - Python: Sphinx with autodoc
     - C++: Doxygen
   - Add documentation to CI/CD pipeline
   - Host documentation (GitHub Pages, Read the Docs)

4. **Enhanced Documentation Examples**:
   ```markdown
   docs/
   ├── index.md                    # Main documentation hub
   ├── getting-started/
   │   ├── installation.md
   │   ├── quick-start.md
   │   └── tutorials/
   ├── architecture/
   │   ├── overview.md
   │   ├── components.md
   │   └── diagrams/
   ├── user-guide/
   │   ├── creating-maps.md
   │   ├── navigation.md
   │   └── properties-guide.md
   ├── developer-guide/
   │   ├── contributing.md
   │   ├── coding-standards.md
   │   └── testing.md
   └── api/
       ├── python/
       └── cpp/
   ```

### 4. Testing Infrastructure

**Current State**:

- **Python**: Uses pytest and launch_pytest ✓
- **Linters**: ament_flake8, ament_pep257 ✓
- **CI/CD**: GitHub Actions testing on Humble and Iron ✓
- **Test Coverage**: Only 1-2 test files found, mostly integration tests

**Issues**:

1. **Low Unit Test Coverage**: Very few unit tests for core modules
2. **Legacy Test Documentation**: `tests/README.md` references ROS 1 tools (catkin_make, roslaunch, MongoDB, MORSE simulator)
3. **No Coverage Metrics**: No measurement of code coverage
4. **Manual Testing Required**: Some functionality requires manual verification

**Recommendations**:

1. **Immediate**:
   - Update `tests/README.md` for ROS 2 (replace catkin_make with colcon, roslaunch with ros2 launch)
   - Add unit tests for critical modules:
     - `route_search2.py` - path planning algorithms
     - `tmap_utils.py` - map utilities
     - `restrictions_impl.py` - validation logic

2. **Short-term**:
   - Add code coverage reporting:
     ```bash
     pytest --cov=topological_navigation --cov-report=html
     ```
   - Set coverage targets (aim for 70%+ for core modules)
   - Add coverage badge to README

3. **Medium-term**:
   - Create comprehensive test suite:
     - Unit tests for all public APIs
     - Integration tests for navigation scenarios
     - Property validation tests
   - Add performance/benchmark tests for route planning
   - Mock external dependencies (Nav2, TF)

4. **Example Test Structure**:
   ```
   topological_navigation/
   └── test/
       ├── unit/
       │   ├── test_route_search.py
       │   ├── test_map_validation.py
       │   └── test_properties.py
       ├── integration/
       │   ├── test_navigation_flow.py
       │   └── test_map_loading.py
       └── fixtures/
           ├── sample_maps/
           └── mock_data/
   ```

### 5. Code Quality Issues

**Technical Debt Identified**:

1. **TODOs and FIXMEs**: 8+ instances in production code
   ```python
   # localisation2.py
   # TODO: remove Temporary arg until tags functionality is MongoDB independent
   
   # navigation2.py
   # FIXME: not implemented
   
   # edge_action_manager2.py
   #TODO change this to actual
   ```

2. **Wildcard Imports**: Found in 10+ utility scripts
   ```python
   from module import *  # Anti-pattern
   ```

3. **Large Files**: Some modules exceed 1000 lines
   - `edge_action_manager2.py`: 1362 lines
   - `manager2.py`: 1538 lines

**Recommendations**:

1. **Immediate**:
   - Create GitHub issues for each TODO/FIXME
   - Add issue references in code:
     ```python
     # TODO(#123): remove Temporary arg until tags functionality is MongoDB independent
     ```

2. **Short-term**:
   - Refactor wildcard imports to explicit imports
   - Add pre-commit hooks to prevent new wildcard imports
   - Configure linters to flag these issues

3. **Medium-term**:
   - Refactor large files into smaller, focused modules:
     ```python
     # Instead of edge_action_manager2.py (1362 lines)
     edge_action_manager/
     ├── __init__.py
     ├── base.py          # Base classes
     ├── actions.py       # Action implementations
     ├── state.py         # State management
     └── utils.py         # Helper functions
     ```
   - Apply SOLID principles for better maintainability

### 6. Properties System (Positive Example)

**Strengths**:

- Excellent documentation in `PROPERTIES.md` ✓
- Flexible, extensible design ✓
- Backward compatible ✓
- Well-thought-out namespacing approach ✓

**Minor Recommendations**:

1. Add JSON Schema for property validation (optional but helpful)
2. Create property builder/validator classes for common patterns
3. Add examples directory with real-world property configurations

### 7. Dependency Management

**Current State**:
- Uses standard ROS 2 dependencies ✓
- Minimal external dependencies ✓
- Clear dependency declarations in `package.xml` ✓

**Potential Issues**:

1. **No Dependency Pinning**: Not using specific versions
2. **No Security Scanning**: No automated vulnerability checks

**Recommendations**:

1. Add Dependabot or Renovate for dependency updates
2. Add security scanning to CI/CD (e.g., `safety` for Python)
3. Document tested dependency versions

### 8. Potential Performance Concerns

**Areas to Monitor**:

1. **Large Map Handling**: Route planning on graphs with hundreds of nodes
2. **Visualization Performance**: Marker updates with many nodes/edges
3. **Memory Usage**: Loading multiple large maps simultaneously

**Recommendations**:

1. Add performance benchmarks for route planning
2. Implement map caching strategies
3. Consider lazy loading for large maps
4. Profile memory usage in long-running scenarios

---

## Priority Recommendations Summary

### High Priority (Address Soon)

1. ✅ **Add deprecation notices** to legacy "version 1" files
2. ✅ **Create READMEs** for topological_utils and topological_navigation_msgs
3. ✅ **Update test documentation** for ROS 2
4. ✅ **Convert TODOs to GitHub issues** with tracking
5. ✅ **Add basic unit tests** for core modules

### Medium Priority (Next 6 Months)

1. 📋 **Plan legacy code removal** for version 4.0.0
2. 📋 **Refactor large files** (>1000 lines) into smaller modules
3. 📋 **Organize utility scripts** into logical subdirectories
4. 📋 **Generate API documentation** (Sphinx/Doxygen)
5. 📋 **Add code coverage** reporting and targets

### Low Priority (Future Enhancements)

1. 💡 **Create unified CLI tool** (`topo` command)
2. 💡 **Add architecture diagrams** to documentation
3. 💡 **Implement performance benchmarks**
4. 💡 **Add dependency security scanning**
5. 💡 **Create migration guides** between versions

---

## Code Modernization Opportunities

### Python Code

1. **Type Hints**: Add comprehensive type hints to all public APIs
   ```python
   def find_route(start: str, end: str, map_name: str) -> List[str]:
       """Find route between nodes."""
       ...
   ```

2. **Dataclasses**: Use dataclasses for data structures
   ```python
   from dataclasses import dataclass
   
   @dataclass
   class Node:
       name: str
       pose: Pose
       properties: Dict[str, Any]
       edges: List[Edge]
   ```

3. **Async/Await**: Consider async patterns for I/O-bound operations

### C++ Code

1. **Modern C++**: Ensure consistent use of C++14/17 features
2. **Smart Pointers**: Verify consistent use of shared_ptr/unique_ptr
3. **RAII**: Ensure proper resource management

---

## AI Agent Friendliness Assessment

**Current Score: 7/10**

**Strengths**:
- ✅ Clear package structure
- ✅ Excellent properties documentation
- ✅ Standard ROS 2 patterns
- ✅ Consistent Python style (PEP 8)

**Areas for Improvement**:
- ⚠️ Legacy code confusion (version 1 vs 2)
- ⚠️ Limited inline documentation
- ⚠️ Missing API documentation
- ⚠️ Few code examples for common tasks

**To Improve AI Agent Experience**:

1. **Add more docstrings** with examples:
   ```python
   def add_edge(node_from: str, node_to: str, action: str) -> None:
       """Add edge between two nodes.
       
       Args:
           node_from: Source node name
           node_to: Target node name
           action: Action to execute (e.g., "NavigateToPose")
           
       Example:
           >>> add_edge("Start", "Goal", "NavigateToPose")
           
       Raises:
           ValueError: If either node doesn't exist
       """
   ```

2. **Create examples directory**:
   ```
   examples/
   ├── basic_navigation.py
   ├── custom_properties.py
   ├── map_creation.py
   └── edge_actions.py
   ```

3. **Add inline comments for complex algorithms** (e.g., A* implementation)

4. **Provide migration examples** between versions

---

## Conclusion

The topological_navigation repository is a well-maintained, functional ROS 2 project with a solid foundation. The primary recommendations focus on:

1. **Reducing confusion** by deprecating and eventually removing legacy code
2. **Improving discoverability** through better documentation and organization
3. **Increasing confidence** through comprehensive testing
4. **Modernizing code** to leverage current best practices

These improvements will benefit both human developers and AI coding agents, making the repository easier to understand, maintain, and extend.

**Estimated Effort**:
- High Priority Items: ~2-3 weeks of focused work
- Medium Priority Items: ~1-2 months spread over multiple sprints
- Low Priority Items: Ongoing improvements over 6-12 months

**Expected Benefits**:
- Reduced onboarding time for new contributors
- Fewer bugs from using deprecated code
- Better AI agent assistance
- Improved maintainability and extensibility
- Higher confidence in code quality

---

## Appendix: Metrics Summary

| Metric | Value | Target |
|--------|-------|--------|
| Total Python Files | 144 | - |
| Files with "2" suffix | 15 | 0 (eventually) |
| Utility Scripts | ~70 | Consolidated |
| Test Files | ~2 | 20+ |
| Code Coverage | Unknown | 70%+ |
| TODOs/FIXMEs | 8+ | 0 (converted to issues) |
| Packages with READMEs | 2/4 | 4/4 |
| API Documentation | No | Yes |
| CI/CD Pipelines | 2 | 3+ (add coverage) |

