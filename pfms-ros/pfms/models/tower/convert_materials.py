#!/usr/bin/env python3
"""Convert Gazebo Classic material scripts to Ignition-compatible materials."""

import re

# Material color mappings (approximate Gazebo colors)
MATERIAL_COLORS = {
    'Gazebo/Wood': {
        'ambient': '0.6 0.4 0.2 1',
        'diffuse': '0.6 0.4 0.2 1',
        'specular': '0.1 0.1 0.1 1'
    },
    'Gazebo/Bricks': {
        'ambient': '0.7 0.3 0.2 1',
        'diffuse': '0.7 0.3 0.2 1',
        'specular': '0.1 0.1 0.1 1'
    },
    'Gazebo/CeilingTiled': {
        'ambient': '0.9 0.9 0.9 1',
        'diffuse': '0.9 0.9 0.9 1',
        'specular': '0.2 0.2 0.2 1'
    },
    'Gazebo/Grey': {
        'ambient': '0.7 0.7 0.7 1',
        'diffuse': '0.7 0.7 0.7 1',
        'specular': '0.1 0.1 0.1 1'
    }
}

def convert_material_script(match):
    """Convert a material script block to Ignition format."""
    full_match = match.group(0)
    
    # Extract material name
    name_match = re.search(r'<name>(Gazebo/\w+)</name>', full_match)
    if not name_match:
        return full_match
    
    material_name = name_match.group(1)
    
    # Get color values
    if material_name in MATERIAL_COLORS:
        colors = MATERIAL_COLORS[material_name]
    else:
        # Default gray
        colors = MATERIAL_COLORS['Gazebo/Grey']
    
    # Check if there's an existing ambient tag outside the script block
    has_ambient = '</script>' in full_match and '<ambient>' in full_match.split('</script>')[1]
    
    # Create Ignition-compatible material
    new_material = f"""          <ambient>{colors['ambient']}</ambient>
          <diffuse>{colors['diffuse']}</diffuse>
          <specular>{colors['specular']}</specular>"""
    
    return new_material

# Read the file
with open('model.sdf', 'r') as f:
    content = f.read()

# Pattern to match material script blocks
# Match from <script> to </script>, including any following <ambient> tag
pattern = r'<script>.*?</script>\s*(?:<ambient>.*?</ambient>\s*)?'

# Replace all material scripts
content = re.sub(pattern, convert_material_script, content, flags=re.DOTALL)

# Update SDF version to 1.8 for better Ignition compatibility
content = content.replace("version='1.7'", "version='1.8'")

# Write the updated file
with open('model.sdf', 'w') as f:
    f.write(content)

print("Material conversion complete!")
