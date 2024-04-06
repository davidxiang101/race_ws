import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import csv
import mplcursors  # Import the mplcursors library


MAP_FILE = 'base_map.pgm'
RACELINE_FILE = 'demoline_smooth.csv'
# 'src/f1tenth_purepursuit/path/demoline20.csv'


map_image = Image.open(MAP_FILE)
map_array = np.array(map_image)

# Map metadata (manually extracted from the YAML content)
origin_x, origin_y, _ = [-3.983246, -1.321383, 0.000000]
resolution = 0.050000

# Function to convert map coordinates to image coordinates
def map_to_image_coordinates(x, y, origin_x, origin_y, resolution):
    x_img = (x - origin_x) / resolution
    y_img = map_array.shape[0] - (y - origin_y) / resolution
    return x_img, y_img

x_coords = []
y_coords = []
data_for_tooltip = [] 
with open(RACELINE_FILE, mode='r') as csvfile:
    csv_reader = csv.reader(csvfile)
    for row in csv_reader:

        # Idk how the raceline paramters work so edit this line as needed 
        x, y, _, _ = map(float, row)

        x_img, y_img = map_to_image_coordinates(x, y, origin_x, origin_y, resolution)
        x_coords.append(x_img)
        y_coords.append(y_img)
        data_for_tooltip.append((x, y)) 


# Plotting
fig, ax = plt.subplots(figsize=(12, 8))
scatter = ax.scatter(x_coords, y_coords, c='red', s=10)

# Use mplcursors to add interactivity
cursor = mplcursors.cursor(scatter, hover=True)

@cursor.connect("add")
def on_add(sel):
    # This function is called whenever a point is hovered over
    # It updates the annotation to show the map values (x, y, speed)
    x_map, y_map = data_for_tooltip[sel.index]
    sel.annotation.set(text=f'X: {x_map:.2f}, Y: {y_map:.2f}',
                       position=(sel.target[0], sel.target[1]))
    sel.annotation.xy = sel.target

ax.imshow(map_array, cmap='gray')
ax.set_title('Race Line on Map')
ax.axis('off')
plt.show()

