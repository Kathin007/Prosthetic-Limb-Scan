import open3d as o3d
import plotly.graph_objs as go
import numpy as np
import math
import webbrowser
import serial
import time

# Your raw data (Distance in cm)
def send_lidar_to_py(port="COM3",baudrate=115200,num_samples=500,timeout=10):

    ser=serial.Serial(port,baudrate,timeout=10)#the function that integrates arduino and python by Serial(port,baud rate and timeout)

    data=[]#array that consists data

    buffer=""#empty string

    while len(data)<num_samples:

        if ser.in_waiting:

            readings=ser.readline().decode("UTF-8",errors="ignore").strip().split(",")#the bytes sent from esp32 is converted to string by utf encoding
            
            for i in readings:
                try:
                    if 0<int(i)<250:
                        data.append(int(i))
                        print(int(i),"\t")

                except:
                    pass

            buffer=""#refresh the line
        
    ser.close()
    print("Received data from ESP32 ")
    return data

raw_data=send_lidar_to_py("COM3")
# Filter invalid points (Distance == 0 or unbelievably large)
filtered = [d for d in raw_data if d > 0 and d < 100]

# Generate points in a circle
points = []
angle = 0
angle_increment = 2 * math.pi / len(filtered)

for dist in filtered:
    # convert cm to m
    r = dist/100.0
    
    x = r * math.cos(angle)
    y = r * math.sin(angle)
    z = 0  # flat circle; you can add variation if you wish
    
    points.append([x, y, z])
    angle += angle_increment

# Now create a point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))

# Estimate normals (required for surface reconstruction)
pcd.estimate_normals()

# Perform surface reconstruction
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=10
)

# Filter by density
vertices = np.asarray(mesh.vertices)
densities = np.asarray(densities)
filtered_indices = densities > np.mean(densities)
filtered_vertices = vertices[filtered_indices]

old_to_new = {old_idx: new_idx for new_idx, old_idx in enumerate(np.where(filtered_indices)[0])}

filtered_triangles = []
for tri in np.asarray(mesh.triangles):
    v0, v1, v2 = tri
    if filtered_indices[v0] and filtered_indices[v1] and filtered_indices[v2]:
        filtered_triangles.append([
            old_to_new[v0],
            old_to_new[v1],
            old_to_new[v2]
        ])

filtered_mesh = o3d.geometry.TriangleMesh()
filtered_mesh.vertices = o3d.utility.Vector3dVector(filtered_vertices)
filtered_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(filtered_triangles))
filtered_mesh.compute_vertex_normals()


# Plot with Plotly
pts = np.asarray(filtered_mesh.vertices)
triangles = np.asarray(filtered_mesh.triangles)

mesh_plot = go.Mesh3d(
    x=pts[:, 0],
    y=pts[:, 1],
    z=pts[:, 2],
    i=triangles[:, 0],
    j=triangles[:, 1],
    k=triangles[:, 2],
    color='lightgrey',
    flatshading=False,
    opacity=0.8
)

# Create a Plotly Figure and add the mesh to it.
fig = go.Figure([mesh_plot])

# Add layout for better visualization
fig.update_layout(
    title='3D Reconstructed Mesh',
    scene=dict(
        xaxis_title='X Axis',
        yaxis_title='Y Axis',
        zaxis_title='Z Axis',
        aspectmode='cube'
    ),
    height=700
)

# Save and view in browser
output_filename = "output.html"
fig.write_html(output_filename, auto_open=False)
print(f"Generated mesh visualization saved to {output_filename}")

print(f"Opening {output_filename} in your default browser...")
webbrowser.open(output_filename)
