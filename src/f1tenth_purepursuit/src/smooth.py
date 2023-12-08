import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


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
    # Assuming higher curvature demands lower speed
    # This function maps curvature values to speed, inversely proportional
    normalized_curvature = curvature / np.max(curvature)  # Normalizing
    return min_speed + (max_speed - min_speed) * (1 - normalized_curvature)


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


def smooth_and_refine_raceline(csv_file, output_file, sigma=3, num_points=500):
    # Load raceline data
    raceline = pd.read_csv(csv_file, header=None, names=["x", "y", "z", "w"])
    x_original = raceline["x"].values
    y_original = raceline["y"].values

    # Smooth the raceline
    x_smooth, y_smooth = smooth_path(x_original, y_original, sigma=sigma)

    # Increase the number of points
    x_high_res, y_high_res = interpolate_path(x_smooth, y_smooth, num_points=num_points)

    # Creating a DataFrame for the high-resolution smoothed raceline
    smoothed_raceline = pd.DataFrame(
        {
            "x": x_high_res,
            "y": y_high_res,
            "z": np.interp(
                np.linspace(0, 1, num_points),
                np.linspace(0, 1, len(raceline)),
                raceline["z"],
            ),  # Interpolating z values
            "w": np.interp(
                np.linspace(0, 1, num_points),
                np.linspace(0, 1, len(raceline)),
                raceline["w"],
            ),  # Interpolating w values
        }
    )

    # Assuming you have curvature data
    curvature = calculate_curvature(x_high_res, y_high_res)

    speed_profile = create_speed_profile(curvature, 0.0, 1.0)

    smoothed_raceline["speed_factor"] = speed_profile

    # Plotting
    plt.figure(figsize=(12, 6))

    # Plot original points
    plt.plot(x_original, y_original, "o", label="Original Points", zorder=1)

    # Plot high-resolution smoothed raceline
    plt.plot(
        x_high_res, y_high_res, label="High-Resolution Smoothed Raceline", zorder=2
    )

    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("High-Resolution Raceline with Braking and Acceleration Zones")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Save the high-resolution smoothed raceline to a CSV file
    smoothed_raceline.to_csv(output_file, header=False, index=False)
    return x_high_res, y_high_res


# Usage
smooth_and_refine_raceline(
    "../path/raceline3.csv", "raceline_speed.csv", sigma=3, num_points=500
)
