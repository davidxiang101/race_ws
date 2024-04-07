import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib.image import imread
import yaml
from scipy.ndimage import rotate
import os


def weighted_average_future_velocity(
    curvature, lookahead_range, min_speed, max_speed, weight_decay=0.97
):
    """
    Calculate a weighted average of future velocities over a lookahead range.
    Closer points have more weight than farther points.
    """
    future_velocities = speed_from_curvature(curvature, min_speed, max_speed)
    weighted_velocities = np.zeros_like(future_velocities)

    for i in range(len(curvature)):
        total_weight = 0.0
        weighted_sum = 0.0
        for j in range(lookahead_range):
            if i + j < len(curvature):
                weight = weight_decay**j
                total_weight += weight
                weighted_sum += future_velocities[i + j] * weight
        if total_weight > 0:
            weighted_velocities[i] = weighted_sum / total_weight
        else:
            weighted_velocities[i] = future_velocities[i]

    return weighted_velocities


def create_dynamic_lookahead_speed_profile(
    curvature, min_speed, max_speed, lookahead_range, weight_decay=0.97
):
    # Adjusting speed based on weighted average of future velocities
    speed_profile = weighted_average_future_velocity(
        curvature, lookahead_range, min_speed, max_speed, weight_decay
    )
    return speed_profile


def calculate_curvature(x, y):
    """Calculate curvature for each point on the path"""
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvature = (
        np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt**2 + dy_dt**2) ** 1.5
    )
    return curvature


def speed_from_curvature(curvature, min_speed, max_speed):
    """
    Maps curvature values to speed, inversely proportional.
    Ensures that the speed is normalized between 0 and 1.
    """
    # Normalizing the curvature to be within the range [0, 1]
    normalized_curvature = curvature / np.max(curvature)

    # Inverting the curvature to make high curvature correspond to low speed
    inverse_curvature = 1 - normalized_curvature

    # Scaling the inverse curvature to be within the range of min_speed and max_speed
    speed_profile = min_speed + (max_speed - min_speed) * (inverse_curvature)

    scaled_speed_profile = np.where(
        speed_profile > 0.7,
        np.minimum(0.7 + 3 * (speed_profile - 0.7), 1.0),  # Cap at 1.0
        speed_profile,
    )

    return scaled_speed_profile


def calculate_future_curvature(curvature, lookahead_distance):
    # Rolling window to look ahead on the curvature array
    future_curvature = np.roll(curvature, -lookahead_distance)
    return future_curvature


def create_advanced_speed_profile(curvature, min_speed, max_speed, lookahead_distance):
    # Adjusting speed based on future curvature
    future_curvature = calculate_future_curvature(curvature, lookahead_distance)
    speed_profile = speed_from_curvature(future_curvature, min_speed, max_speed)
    return speed_profile


def create_speed_profile(curvature, min_speed, max_speed):
    speed_profile = speed_from_curvature(curvature, min_speed, max_speed)
    return speed_profile


def smooth_path(x, y, sigma=3):
    """Smooth the path using a Gaussian filter"""
    x_smooth = gaussian_filter1d(x, sigma)
    y_smooth = gaussian_filter1d(y, sigma)
    return x_smooth, y_smooth


def interpolate_path(x, y, num_points=500):
    """Interpolate to increase the number of points on the path"""
    t = np.linspace(0, 1, len(x))
    t_new = np.linspace(0, 1, num_points)
    f_x = interp1d(t, x, kind="cubic")
    f_y = interp1d(t, y, kind="cubic")
    x_interp = f_x(t_new)
    y_interp = f_y(t_new)
    return x_interp, y_interp


def smooth_and_refine_raceline(
    csv_file,
    output_file,
    sigma=3,
    num_points=500,
    lookahead_range=125,
    weight_decay=0.97,
    map_yaml_file=None,
):
    # Load raceline data
    raceline = pd.read_csv(csv_file, header=None, names=["x", "y", "z", "w"])
    x_original = raceline["x"].values
    y_original = raceline["y"].values

    # Smooth the raceline
    x_smooth, y_smooth = gaussian_filter1d(x_original, sigma), gaussian_filter1d(
        y_original, sigma
    )

    # Interpolate path
    t = np.linspace(0, 1, len(x_smooth))
    t_new = np.linspace(0, 1, num_points)
    f_x, f_y = interp1d(t, x_smooth, kind="cubic"), interp1d(t, y_smooth, kind="cubic")
    x_high_res, y_high_res = f_x(t_new), f_y(t_new)

    # Curvature calculation
    curvature = calculate_curvature(x_high_res, y_high_res)
    speed_profile = create_dynamic_lookahead_speed_profile(
        curvature, 0.0, 1.0, lookahead_range, weight_decay
    )

    # Create DataFrame for the smoothed raceline
    smoothed_raceline = pd.DataFrame(
        {
            "x": x_high_res,
            "y": y_high_res,
            "z": [0.0]
            * len(x_high_res),  # List of 0.0 with the same length as x_high_res
            "w": [1.0]
            * len(x_high_res),  # List of 1.0 with the same length as x_high_res
            "speed_factor": speed_profile,
        }
    )

    # Plotting

    plt.figure(figsize=(12, 6))

    # If a YAML file is provided, read and display the associated PGM map
    if map_yaml_file and os.path.exists(map_yaml_file):
        with open(map_yaml_file, "r") as file:
            map_data = yaml.safe_load(file)
        pgm_file = map_data.get("image")
        if pgm_file and os.path.exists(pgm_file):
            pgm_image = imread(pgm_file)
            pgm_image = rotate(pgm_image, -15, reshape=False)  # Rotate the image
            height, width = pgm_image.shape
            x_min, y_min = map_data.get("origin", [0, 0])[:2]
            x_max, y_max = x_min + width * map_data.get(
                "resolution", 0.05
            ), y_min + height * map_data.get("resolution", 0.05)
            plt.imshow(
                pgm_image,
                cmap="gray",
                extent=[x_min, x_max, y_min, y_max],
                origin="lower",
            )

    # Plot original and smoothed raceline
    plt.plot(x_original, y_original, "o", label="Original Points", zorder=1)
    scatter = plt.scatter(
        x_high_res, y_high_res, c=speed_profile, cmap=cm.gist_rainbow, zorder=2
    )
    cbar = plt.colorbar(scatter)
    cbar.set_label("Speed Factor")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("High-Resolution Raceline with Speed Factor Coloring and Track Map")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Save the high-resolution smoothed raceline to a CSV file
    smoothed_raceline.to_csv(output_file, header=False, index=False)
    return x_high_res, y_high_res


# Usage
smooth_and_refine_raceline(
    "../path/demoline_smooth_test.csv",
    "../path/demoline_smooth_test_smooth.csv",
    sigma=5,
    num_points=1000,
    lookahead_range=60,
    weight_decay=0.985,
    map_yaml_file=None,
)
