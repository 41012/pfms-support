#!/usr/bin/env python3
"""
Waypoint generator using full mesh analysis.
This script analyzes the 3D mesh structure to find the track boundaries
and generates centerline waypoints.

python3 generate_waypoints_mesh.py --stl ../models/track/map.stl --sdf ../models/maze1/model.sdf --output ~/pfms_ws/src/a3_support/data/A3_RACING.TXT --spacing 3.0 --format txt
"""

import numpy as np
import argparse
from pathlib import Path


def load_stl_simple(filepath):
    """Load STL file and extract vertices manually."""
    vertices = []
    
    with open(filepath, 'rb') as f:
        # Skip header (80 bytes)
        f.read(80)
        
        # Read number of triangles
        num_triangles = int.from_bytes(f.read(4), byteorder='little')
        
        for _ in range(num_triangles):
            # Skip normal (12 bytes)
            f.read(12)
            
            # Read 3 vertices (each 3 floats = 12 bytes)
            for _ in range(3):
                x = np.frombuffer(f.read(4), dtype=np.float32)[0]
                y = np.frombuffer(f.read(4), dtype=np.float32)[0]
                z = np.frombuffer(f.read(4), dtype=np.float32)[0]
                vertices.append([x, y, z])
            
            # Skip attribute byte count (2 bytes)
            f.read(2)
    
    return np.array(vertices)


def analyze_track_geometry(vertices, ground_z=0.0, wall_height_min=0.2):
    """
    Analyze track geometry to find wall positions using connectivity-based clustering.
    Uses BFS with spatial grid for efficient neighbor queries.
    
    Starts from the point with highest Y coordinate and uses nearest-neighbor 
    clustering to group points into one wall. Remaining points form the other wall.
    
    Args:
        vertices: All mesh vertices
        ground_z: Z coordinate of the ground plane
        wall_height_min: Minimum height to consider as wall
    
    Returns:
        wall1, wall2: Arrays of wall points for the two walls
    """
    from collections import deque, defaultdict
    
    # Filter vertices that are above ground (wall vertices)
    wall_mask = vertices[:, 2] > (ground_z + wall_height_min)
    wall_vertices = vertices[wall_mask]
    
    # Project to 2D (x, y)
    wall_points_2d = wall_vertices[:, :2]
    
    # Remove duplicates
    wall_points_2d = np.unique(wall_points_2d, axis=0)
    
    # Build spatial grid for fast neighbor queries
    max_neighbor_distance = 0.5  # meters
    grid_size = max_neighbor_distance
    
    def get_grid_cell(point):
        return (int(point[0] / grid_size), int(point[1] / grid_size))
    
    # Map each grid cell to list of point indices
    grid = defaultdict(list)
    for idx, point in enumerate(wall_points_2d):
        cell = get_grid_cell(point)
        grid[cell].append(idx)
    
    # Find the point with highest Y coordinate (furthest in Y axis)
    start_idx = np.argmax(wall_points_2d[:, 1])
    
    # Use BFS to grow wall1 cluster
    wall1_indices = set()
    to_process = deque([start_idx])
    remaining_indices = set(range(len(wall_points_2d)))
    
    while to_process:
        current_idx = to_process.popleft()
        
        if current_idx not in remaining_indices:
            continue
        
        # Add this point to wall1
        wall1_indices.add(current_idx)
        remaining_indices.remove(current_idx)
        
        # Find neighbors in nearby grid cells
        current_point = wall_points_2d[current_idx]
        current_cell = get_grid_cell(current_point)
        
        # Check current cell and 8 surrounding cells
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor_cell = (current_cell[0] + dx, current_cell[1] + dy)
                if neighbor_cell in grid:
                    for idx in grid[neighbor_cell]:
                        if idx in remaining_indices:
                            point = wall_points_2d[idx]
                            dist = np.linalg.norm(current_point - point)
                            if dist <= max_neighbor_distance:
                                to_process.append(idx)
    
    # Remaining points form wall2
    wall2_indices = remaining_indices
    
    wall1 = wall_points_2d[list(wall1_indices)]
    wall2 = wall_points_2d[list(wall2_indices)]
    
    return wall1, wall2
    
    wall1 = wall_points_2d[wall1_indices]
    wall2 = wall_points_2d[wall2_indices]
    
    return wall1, wall2


def order_wall_points(wall_points, detect_loop_closure=True, closure_threshold=5.0):
    """Order wall points to form a continuous path.
    
    Args:
        wall_points: Array of wall points to order
        detect_loop_closure: If True, stop when path closes back to start
        closure_threshold: Distance threshold for detecting loop closure (meters)
    
    Returns:
        Ordered array of wall points
    """
    if len(wall_points) == 0:
        return wall_points
    
    # Start from point with highest Y coordinate
    start_idx = np.argmax(wall_points[:, 1])
    ordered = [wall_points[start_idx]]
    remaining_mask = np.ones(len(wall_points), dtype=bool)
    remaining_mask[start_idx] = False
    start_point = wall_points[start_idx]
    
    while np.any(remaining_mask):
        last_point = ordered[-1]
        remaining_points = wall_points[remaining_mask]
        
        # Find nearest point
        distances = np.linalg.norm(remaining_points - last_point, axis=1)
        nearest_local_idx = np.argmin(distances)
        
        # Convert to global index
        nearest_global_idx = np.where(remaining_mask)[0][nearest_local_idx]
        next_point = wall_points[nearest_global_idx]
        
        # Check if we're closing the loop (coming back to start)
        if detect_loop_closure and len(ordered) > 100:  # Only check after reasonable progress
            dist_to_start = np.linalg.norm(next_point - start_point)
            if dist_to_start < closure_threshold:
                # Loop is closing, stop here
                break
        
        ordered.append(next_point)
        remaining_mask[nearest_global_idx] = False
    
    return np.array(ordered)


def compute_centerline_from_walls(wall1, wall2, num_samples=1000):
    """
    Compute centerline by finding closest opposing points on walls.
    
    For each point on wall1, finds the closest point on wall2
    and computes the midpoint between them.
    
    Args:
        wall1, wall2: Unordered wall points
        num_samples: Number of samples along the centerline (unused, kept for compatibility)
    
    Returns:
        centerline points
    """
    # Order wall1 to form a continuous path
    wall1_ordered = order_wall_points(wall1)
    
    # For each point on wall1, find the closest point on wall2
    # and compute the midpoint
    centerline_points = []
    
    for point1 in wall1_ordered:
        # Find closest point on wall2
        distances = np.linalg.norm(wall2 - point1, axis=1)
        closest_idx = np.argmin(distances)
        point2 = wall2[closest_idx]
        
        # Compute midpoint
        midpoint = (point1 + point2) / 2.0
        centerline_points.append(midpoint)
    
    centerline = np.array(centerline_points)
    
    return centerline


def resample_path(path, spacing):
    """
    Resample a path to have uniform spacing between points.
    
    Args:
        path: Input path points
        spacing: Desired spacing between points
    
    Returns:
        Resampled path
    """
    # Compute cumulative distance
    distances = np.zeros(len(path))
    for i in range(1, len(path)):
        distances[i] = distances[i-1] + np.linalg.norm(path[i] - path[i-1])
    
    total_length = distances[-1]
    
    # Generate sample points
    sample_distances = np.arange(0, total_length, spacing)
    
    # Interpolate
    resampled = np.array([
        np.interp(sample_distances, distances, path[:, 0]),
        np.interp(sample_distances, distances, path[:, 1])
    ]).T
    
    return resampled


def read_pose_from_sdf(sdf_path):
    """
    Read pose offset from SDF file.
    Returns: (x, y, z) offset tuple
    """
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Find the visual/pose element
        pose_elem = root.find('.//visual/pose')
        if pose_elem is not None and pose_elem.text:
            pose_values = [float(x) for x in pose_elem.text.split()]
            return pose_values[0], pose_values[1], pose_values[2]
    except Exception as e:
        print(f"Warning: Could not read pose from SDF: {e}")
    
    return 0.0, 0.0, 0.0


def save_waypoints(waypoints, filepath, format='csv', pose_offset=(0, 0, 0)):
    """Save waypoints to file with optional pose offset."""
    filepath = Path(filepath)
    
    # Apply pose offset to waypoints (transform from mesh local to world coordinates)
    waypoints_world = waypoints + np.array([pose_offset[0], pose_offset[1]])
    
    if format == 'csv':
        np.savetxt(filepath, waypoints_world, delimiter=',', 
                   header='x,y', comments='', fmt='%.6f')
    elif format == 'txt':
        # Add z=0 column for txt format
        waypoints_3d = np.column_stack([waypoints_world, np.zeros(len(waypoints_world))])
        np.savetxt(filepath, waypoints_3d, fmt='%.6f')
    elif format == 'yaml':
        with open(filepath, 'w') as f:
            f.write("waypoints:\n")
            for i, (x, y) in enumerate(waypoints_world):
                f.write(f"  - {{id: {i}, x: {x:.6f}, y: {y:.6f}}}\n")
    
    print(f"Saved {len(waypoints)} waypoints to {filepath}")
    if pose_offset != (0, 0, 0):
        print(f"Applied pose offset: ({pose_offset[0]}, {pose_offset[1]}, {pose_offset[2]})")


def main():
    parser = argparse.ArgumentParser(
        description='Generate waypoints from racetrack STL using mesh analysis'
    )
    parser.add_argument('--stl', type=str,
                       default='../models/maze1/map.stl',
                       help='Path to STL file')
    parser.add_argument('--sdf', type=str,
                       default='../models/maze1/model.sdf',
                       help='Path to SDF file for pose offset')
    parser.add_argument('--output', type=str,
                       default='waypoints.txt',
                       help='Output file')
    parser.add_argument('--spacing', type=float,
                       default=0.3,
                       help='Waypoint spacing in meters')
    parser.add_argument('--format', choices=['csv', 'txt', 'yaml'],
                       default='txt',
                       help='Output format')
    parser.add_argument('--wall-height', type=float,
                       default=0.2,
                       help='Minimum wall height')
    parser.add_argument('--no-pose-offset', action='store_true',
                       help='Do not apply pose offset from SDF file')
    parser.add_argument('--manual-offset', type=str,
                       default=None,
                       help='Manual offset as "x,y" (overrides SDF pose)')
    
    args = parser.parse_args()
    
    # Resolve paths
    script_dir = Path(__file__).parent
    stl_path = (script_dir / args.stl).resolve()
    sdf_path = (script_dir / args.sdf).resolve()
    
    if not stl_path.exists():
        print(f"Error: STL file not found at {stl_path}")
        return 1
    
    # Read pose offset from SDF if available
    pose_offset = (0, 0, 0)
    if args.manual_offset:
        # Manual offset overrides everything
        x, y = map(float, args.manual_offset.split(','))
        pose_offset = (x, y, 0)
        print(f"Using manual pose offset: {pose_offset}")
    elif not args.no_pose_offset and sdf_path.exists():
        pose_offset = read_pose_from_sdf(str(sdf_path))
        print(f"Read pose offset from SDF: {pose_offset}")
    elif not args.no_pose_offset:
        print(f"Warning: SDF file not found at {sdf_path}, using zero offset")
    
    print(f"Loading STL: {stl_path}")
    vertices = load_stl_simple(str(stl_path))
    print(f"Loaded {len(vertices)} vertices")
    
    print("Analyzing track geometry...")
    wall1, wall2 = analyze_track_geometry(
        vertices, wall_height_min=args.wall_height
    )
    print(f"Found {len(wall1)} wall1 points, {len(wall2)} wall2 points")
    
    print("Computing centerline...")
    centerline = compute_centerline_from_walls(wall1, wall2)
    
    print(f"Generating waypoints with {args.spacing}m spacing...")
    waypoints = resample_path(centerline, args.spacing)
    print(f"Generated {len(waypoints)} waypoints")
    
    save_waypoints(waypoints, args.output, args.format, pose_offset)
    
    return 0


if __name__ == '__main__':
    exit(main())
