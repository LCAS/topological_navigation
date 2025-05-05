'''
This Python script provides a graphical interface for editing a topological map 
represented in a YAML file. It uses `tkinter` for creating a form to edit node 
attributes like position, yaw, and properties. The nodes are plotted using 
`matplotlib`, and clicking on a node opens a dialog for editing. It includes 
functionality to convert between quaternion and yaw, update node names globally, 
scale polygons representing node vertices, and save the updated YAML file. 
The interface includes save and scale buttons to adjust polygon sizes interactively.

Author: Ibrahim Hroob 
email: ibrahim.hroub7@gmail.com
'''

import yaml
import math
import tkinter as tk
import matplotlib.pyplot as plt

from tkinter import ttk
from tkinter import filedialog
from tkinter import messagebox
from matplotlib.widgets import Button
from matplotlib.backend_bases import MouseButton


# Convert quaternion (w, z) to yaw (in degrees)
def quaternion_to_yaw(w, z):
    return math.degrees(2 * math.atan2(z, w))

# Convert yaw (in degrees) to quaternion (w, z)
def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    return math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)

# Load YAML data
def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# Save YAML data
def save_yaml(data, file_path):
    with open(file_path, 'w') as file:
        yaml.dump(data, file)

# Function to update node names
def update_node_name(nodes, old_name, new_name):
    # Traverse through all nodes
    for node in nodes:
        # Update node name in meta section
        if node['meta']['node'] == old_name:
            node['meta']['node'] = new_name
        
        # Update node name in edges
        for edge in node['node'].get('edges', []):
            if edge.get('edge_id'):
                edge['edge_id'] = edge['edge_id'].replace(old_name, new_name)
            if edge.get('node') == old_name:
                edge['node'] = new_name
        
        # Update the main node name
        if node['node'].get('name') == old_name:
            node['node']['name'] = new_name


# Display a dialog with all metadata for the node
def edit_node_info(node_data, nodes, data):
    root = tk.Tk()
    root.title(f"Edit Node: {node_data['node']['name']}")

    old_node_name = node_data['node']['name']

    # Create a frame for the metadata
    frame = ttk.Frame(root, padding="10")
    frame.grid(row=0, column=0, sticky=(tk.W, tk.E))

    # Node name (removed as per request)
    ttk.Label(frame, text="Node Name:").grid(column=0, row=0, sticky=tk.W)
    node_name = tk.StringVar(value=node_data['node']['name'])
    ttk.Label(frame, textvariable=node_name).grid(column=1, row=0)

    # Display node's current position
    ttk.Label(frame, text="Position X:").grid(column=0, row=1, sticky=tk.W)
    position_x = tk.DoubleVar(value=node_data['node']['pose']['position']['x'])
    ttk.Entry(frame, textvariable=position_x).grid(column=1, row=1)

    ttk.Label(frame, text="Position Y:").grid(column=0, row=2, sticky=tk.W)
    position_y = tk.DoubleVar(value=node_data['node']['pose']['position']['y'])
    ttk.Entry(frame, textvariable=position_y).grid(column=1, row=2)

    # Display and allow editing of yaw (orientation in degrees)
    current_yaw = quaternion_to_yaw(node_data['node']['pose']['orientation']['w'], 
                                    node_data['node']['pose']['orientation']['z'])
    ttk.Label(frame, text="Yaw (degrees):").grid(column=0, row=3, sticky=tk.W)
    yaw_deg = tk.DoubleVar(value=current_yaw)
    ttk.Entry(frame, textvariable=yaw_deg).grid(column=1, row=3)

    # Localise by topic
    ttk.Label(frame, text="Localise by Topic:").grid(column=0, row=4, sticky=tk.W)
    localise_by_topic = tk.StringVar(value=node_data['node'].get('localise_by_topic', ''))
    ttk.Entry(frame, textvariable=localise_by_topic).grid(column=1, row=4)

    # Parent frame
    ttk.Label(frame, text="Parent Frame:").grid(column=0, row=5, sticky=tk.W)
    parent_frame = tk.StringVar(value=node_data['node'].get('parent_frame', ''))
    ttk.Entry(frame, textvariable=parent_frame).grid(column=1, row=5)

    # Properties
    properties = node_data['node']['properties']
    ttk.Label(frame, text="XY Goal Tolerance:").grid(column=0, row=6, sticky=tk.W)
    xy_goal_tolerance = tk.DoubleVar(value=properties.get('xy_goal_tolerance', 0.0))
    ttk.Entry(frame, textvariable=xy_goal_tolerance).grid(column=1, row=6)

    ttk.Label(frame, text="Yaw Goal Tolerance:").grid(column=0, row=7, sticky=tk.W)
    yaw_goal_tolerance = tk.DoubleVar(value=properties.get('yaw_goal_tolerance', 0.0))
    ttk.Entry(frame, textvariable=yaw_goal_tolerance).grid(column=1, row=7)

    # Restrictions
    ttk.Label(frame, text="Restrictions Planning:").grid(column=0, row=8, sticky=tk.W)
    restrictions_planning = tk.BooleanVar(value=(node_data['node'].get('restrictions_planning') == 'True'))
    tk.Checkbutton(frame, variable=restrictions_planning,width=2).grid(column=1, row=8)

    ttk.Label(frame, text="Restrictions Runtime:").grid(column=0, row=9, sticky=tk.W)
    restrictions_runtime = tk.StringVar(value=node_data['node'].get('restrictions_runtime', ''))
    ttk.Entry(frame, textvariable=restrictions_runtime).grid(column=1, row=9)

    # Vert scale
    ttk.Label(frame, text="Vert Scale:").grid(column=0, row=10, sticky=tk.W)
    vert_scale = tk.DoubleVar(value=1.0)
    ttk.Entry(frame, textvariable=vert_scale).grid(column=1, row=10)

    # Display node's metadata
    metadata_str = tk.StringVar(value=yaml.dump(node_data.get('meta', {})))
    ttk.Label(frame, text="Metadata:").grid(column=0, row=11, sticky=tk.W)
    metadata_entry = tk.Text(frame, height=5, width=40)
    metadata_entry.insert(tk.END, metadata_str.get())
    metadata_entry.grid(column=1, row=11)

    # Display edges
    ttk.Label(frame, text="Edges:").grid(column=0, row=12, sticky=tk.W)
    edge_text = tk.Text(frame, height=10, width=40)
    edge_text.insert(tk.END, yaml.dump(node_data['node'].get('edges', [])))
    edge_text.grid(column=1, row=12)

    # Function to save the changes
    def save_changes():
        try:
            # Node name is not editable now
            # node_data['node']['name'] = node_name.get()

            # Update position
            node_data['node']['pose']['position']['x'] = position_x.get()
            node_data['node']['pose']['position']['y'] = position_y.get()

            # Update orientation based on yaw
            w, z = yaw_to_quaternion(yaw_deg.get())
            node_data['node']['pose']['orientation']['w'] = w
            node_data['node']['pose']['orientation']['z'] = z

            # Update other fields
            node_data['node']['localise_by_topic'] = localise_by_topic.get()
            node_data['node']['parent_frame'] = parent_frame.get()
            node_data['node']['properties']['xy_goal_tolerance'] = xy_goal_tolerance.get()
            node_data['node']['properties']['yaw_goal_tolerance'] = yaw_goal_tolerance.get()
            node_data['node']['restrictions_planning'] = 'True' if restrictions_planning.get() else 'False'
            node_data['node']['restrictions_runtime'] = restrictions_runtime.get()

            # Apply vert scaling
            verts = node_data['node'].get('verts', [])
            if verts:
                scale = vert_scale.get()
                for vert in verts:
                    vert['x'] *= scale
                    vert['y'] *= scale

            # Update metadata
            node_data['meta'] = yaml.safe_load(metadata_entry.get("1.0", tk.END))

            # If node name has changed, update all it name in the whole yaml file
            new_node_name = node_data['meta']['node']
            if new_node_name != old_node_name:
                update_node_name(nodes, old_node_name, new_node_name)
            
            # Update edges
            node_data['node']['edges'] = yaml.safe_load(edge_text.get("1.0", tk.END))

            root.destroy()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save changes: {str(e)}")

    # Add Save and Cancel buttons
    ttk.Button(frame, text="Save", command=save_changes).grid(column=0, row=13)
    ttk.Button(frame, text="Cancel", command=root.destroy).grid(column=1, row=13)

    root.mainloop()

# Plot the nodes, edges, and add a save button
def plot_map(data, yaml_file_path):
    fig, ax = plt.subplots(figsize=(7, 11))
    plt.subplots_adjust(bottom=0.2)

    nodes = data['nodes']
    polygon_scale = 1.0  # Default scale for polygons
    clicked_node = None  # To keep track of the clicked node

    # Plot nodes and edges
    def draw_plot():
        ax.clear()
        for node in nodes:
            x = node['node']['pose']['position']['x']
            y = node['node']['pose']['position']['y']
            name = node['node']['name']

            # Change color if this node was clicked
            node_color = 'blue' if node != clicked_node else 'red'  # Red if clicked, otherwise blue

            # Plot node position
            ax.plot(x, y, 'o', color=node_color)  # Use 'node_color' for the clicked node
            ax.text(x + 0.1, y + 0.1, name, fontsize=9, color='black')  # display waypoint name

            # Calculate and plot orientation arrow
            w, z = node['node']['pose']['orientation']['w'], node['node']['pose']['orientation']['z']
            yaw = quaternion_to_yaw(w, z)
            dx = math.cos(math.radians(yaw))
            dy = math.sin(math.radians(yaw))
            ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='r', ec='r')

            # Plot verts as a polygon
            verts = node['node'].get('verts', [])
            if verts:
                # Apply global scaling to verts
                verts_scaled = [(x + v['x'], y + v['y']) for v in verts]
                poly = plt.Polygon(verts_scaled, closed=True, fill=None, edgecolor='goldenrod')
                ax.add_patch(poly)

            # Plot edges
            for edge in node['node'].get('edges', []):
                target_node_name = edge['node']
                target_node = next((n for n in nodes if n['node']['name'] == target_node_name), None)
                if target_node:
                    target_x = target_node['node']['pose']['position']['x']
                    target_y = target_node['node']['pose']['position']['y']
                    ax.plot([x, target_x], [y, target_y], 'g-')

        ax.set_title('Topological Map')
        ax.set_aspect('equal')
        plt.tight_layout()  # Make the layout tight
        plt.draw()

    draw_plot()

    def on_click(event):
        nonlocal clicked_node
        if event.button == MouseButton.LEFT:
            for node in nodes:
                x = node['node']['pose']['position']['x']
                y = node['node']['pose']['position']['y']
                # Check if click is near a node
                if abs(event.xdata - x) < 0.5 and abs(event.ydata - y) < 0.5:
                    clicked_node = node  # Set the clicked node
                    edit_node_info(node, nodes, data)
                    draw_plot()  # Redraw the updated map with the node color change
                    break

    fig.canvas.mpl_connect('button_press_event', on_click)

    # Save button functionality
    def save_changes(event):
        save_yaml(data, yaml_file_path)
        messagebox.showinfo("Save", "Changes saved successfully!")

    # Function to scale polygons globally and update data
    def scale_polygon(factor):
        nonlocal polygon_scale
        polygon_scale *= factor
        # Update verts in the data
        for node in nodes:
            verts = node['node'].get('verts', [])
            if verts:
                for vert in verts:
                    vert['x'] *= factor
                    vert['y'] *= factor
        draw_plot()

    # Add buttons for save and scaling polygons
    ax_save = plt.axes([0.0, 0.0, 0.1, 0.05])
    btn_save = Button(ax_save, 'Save')
    btn_save.on_clicked(save_changes)

    ax_scale_up = plt.axes([0.0, 0.075, 0.1, 0.05])
    btn_scale_up = Button(ax_scale_up, 'Scale\nUp')
    btn_scale_up.on_clicked(lambda event: scale_polygon(1.2))

    ax_scale_down = plt.axes([0.0, 0.125, 0.1, 0.05])
    btn_scale_down = Button(ax_scale_down, 'Scale\nDown')
    btn_scale_down.on_clicked(lambda event: scale_polygon(0.8))
    plt.text(0.175, 1.1, 'Verts', fontweight='bold', fontsize=11)

    plt.show()

# Main function
def main(yaml_file):
    data = load_yaml(yaml_file)
    plot_map(data, yaml_file)

if __name__ == "__main__":
    # Replace with the actual path to your YAML file
    yaml_file_path = filedialog.askopenfilename(title="Select YAML file" ,filetypes=[("YAML files", "*.yaml")])
    main(yaml_file_path)

