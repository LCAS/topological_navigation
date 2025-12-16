#!/usr/bin/env python3
"""
Topological Map Schema Validator

This script validates topological map YAML files against the tmap-schema.yaml schema.
It can be used standalone (without ROS) to verify map files are correctly formatted.

Usage:
    python3 validate_map.py <map_file.yaml> [schema_file.yaml]

If schema_file is not provided, the script will look for tmap-schema.yaml in the
standard config directory.

Exit codes:
    0 - Validation successful
    1 - Validation failed
    2 - File not found or other error
"""

import sys
import os
import argparse

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required. Install with: pip install pyyaml")
    sys.exit(2)

try:
    import jsonschema
except ImportError:
    print("Error: jsonschema is required. Install with: pip install jsonschema")
    sys.exit(2)


def find_schema_file():
    """Find the schema file in standard locations."""
    # Try relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Possible locations for the schema file
    possible_paths = [
        os.path.join(script_dir, '..', '..', 'config', 'tmap-schema.yaml'),
        os.path.join(script_dir, '..', 'config', 'tmap-schema.yaml'),
        os.path.join(script_dir, 'config', 'tmap-schema.yaml'),
    ]
    
    # Try ROS2 package share directory if ament_index is available
    try:
        from ament_index_python.packages import get_package_share_directory
        package_path = get_package_share_directory('topological_navigation')
        possible_paths.insert(0, os.path.join(package_path, 'config', 'tmap-schema.yaml'))
    except (ImportError, Exception):
        pass
    
    for path in possible_paths:
        normalized_path = os.path.normpath(path)
        if os.path.isfile(normalized_path):
            return normalized_path
    
    return None


def load_yaml_file(filepath):
    """Load a YAML file and return its contents."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ValueError(f"Invalid YAML syntax: {e}")
    except FileNotFoundError:
        raise FileNotFoundError(f"File not found: {filepath}")
    except Exception as e:
        raise RuntimeError(f"Error reading file: {e}")


def validate_map(map_file, schema_file=None, verbose=False):
    """
    Validate a topological map file against the schema.
    
    Args:
        map_file: Path to the topological map YAML file
        schema_file: Path to the schema YAML file (optional)
        verbose: Print detailed validation information
    
    Returns:
        Tuple of (is_valid: bool, message: str)
    """
    # Find schema file if not provided
    if schema_file is None:
        schema_file = find_schema_file()
        if schema_file is None:
            return False, "Could not find tmap-schema.yaml. Please specify schema file path."
    
    if verbose:
        print(f"Using schema: {schema_file}")
        print(f"Validating map: {map_file}")
    
    # Load schema
    try:
        schema = load_yaml_file(schema_file)
    except Exception as e:
        return False, f"Error loading schema: {e}"
    
    # Load map
    try:
        tmap = load_yaml_file(map_file)
    except Exception as e:
        return False, f"Error loading map: {e}"
    
    # Validate
    try:
        jsonschema.validate(tmap, schema)
        
        # Additional validation checks
        warnings = []
        
        # Check for node count
        node_count = len(tmap.get('nodes', []))
        if verbose:
            print(f"Found {node_count} nodes")
        
        # Check for duplicate node names
        node_names = set()
        node_names_list = []  # Keep a list for edge validation
        for node_entry in tmap.get('nodes', []):
            node = node_entry.get('node', {})
            name = node.get('name')
            if name:
                if name in node_names:
                    warnings.append(f"Duplicate node name: {name}")
                node_names.add(name)
                node_names_list.append(name)
        
        # Check for edges pointing to non-existent nodes
        for node_entry in tmap.get('nodes', []):
            node = node_entry.get('node', {})
            for edge in node.get('edges', []):
                target = edge.get('node')
                if target and target not in node_names:
                    warnings.append(f"Edge '{edge.get('edge_id')}' points to non-existent node: {target}")
        
        message = "Validation successful"
        if warnings:
            message += f"\n\nWarnings ({len(warnings)}):\n" + "\n".join(f"  - {w}" for w in warnings)
        
        return True, message
        
    except jsonschema.ValidationError as e:
        # Format a user-friendly error message
        path = " -> ".join(str(p) for p in e.absolute_path) if e.absolute_path else "root"
        return False, f"Validation failed at '{path}':\n  {e.message}"
    except jsonschema.SchemaError as e:
        return False, f"Schema error: {e.message}"


def main():
    parser = argparse.ArgumentParser(
        description='Validate topological map YAML files against the schema.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s my_map.tmap2.yaml
  %(prog)s my_map.tmap2.yaml --schema custom_schema.yaml
  %(prog)s my_map.tmap2.yaml -v
        """
    )
    parser.add_argument('map_file', help='Path to the topological map YAML file')
    parser.add_argument('--schema', '-s', help='Path to the schema YAML file (optional)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print detailed information')
    
    args = parser.parse_args()
    
    # Check if map file exists
    if not os.path.isfile(args.map_file):
        print(f"Error: Map file not found: {args.map_file}")
        sys.exit(2)
    
    # Validate
    is_valid, message = validate_map(args.map_file, args.schema, args.verbose)
    
    if is_valid:
        print(f"✓ {message}")
        sys.exit(0)
    else:
        print(f"✗ {message}")
        sys.exit(1)


if __name__ == '__main__':
    main()
