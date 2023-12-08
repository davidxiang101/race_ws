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


def calculate_differential_curvature(curvature):
    return np.gradient(curvature)


def identify_turn_zones(
    curvature, differential_curvature, curvature_threshold, diff_curvature_threshold
):
    turning_points = []
    apex_points = []
    end_turn_points = []
    in_turn = False

    for i in range(1, len(curvature) - 1):
        # Detecting the start of a turn
        if (
            curvature[i] > curvature_threshold
            and differential_curvature[i] > diff_curvature_threshold
            and not in_turn
        ):
            turning_points.append(i)
            in_turn = True

        # Detecting the apex of the turn
        if (
            in_turn
            and differential_curvature[i] < 0
            and differential_curvature[i - 1] >= 0
        ):
            apex_points.append(i)

        # Detecting the end of a turn
        if (
            in_turn
            and curvature[i] < curvature_threshold
            and differential_curvature[i] < diff_curvature_threshold
        ):
            end_turn_points.append(i)
            in_turn = False

    return turning_points, apex_points, end_turn_points


def create_speed_profile(
    curvature, turning_points, apex_points, min_speed, max_speed, brake_zone_length
):
    speed_profile = np.full(len(curvature), max_speed)
    braking_zones = []
    acceleration_zones = []

    for turn in turning_points:
        for i in range(
            turn - brake_zone_length, turn
        ):  # Example: start slowing down brake_zone_length points before the turn
            speed_profile[i] = np.interp(
                i, [turn - brake_zone_length, turn], [max_speed, min_speed]
            )
            braking_zones.append(i)

    for apex in apex_points:
        for i in range(
            apex, apex + brake_zone_length
        ):  # Example: start speeding up brake_zone_length points after the apex
            if i < len(speed_profile):
                speed_profile[i] = np.interp(
                    i, [apex, apex + brake_zone_length], [min_speed, max_speed]
                )
                acceleration_zones.append(i)

    return speed_profile, braking_zones, acceleration_zones


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

    # Calculate differential curvature
    differential_curvature = calculate_differential_curvature(curvature)

    # Update turn detection
    curvature_threshold = 0.1
    diff_curvature_threshold = 0.01
    turning_points, apex_points, end_turn_points = identify_turn_zones(
        curvature, differential_curvature, curvature_threshold, diff_curvature_threshold
    )

    speed_profile, braking_zones, acceleration_zones = create_speed_profile(
        curvature, turning_points, apex_points, 0.0, 1.0, 10
    )
    smoothed_raceline["speed_factor"] = speed_profile

    # Plotting
    plt.figure(figsize=(12, 6))

    # Plot original points
    plt.plot(x_original, y_original, "o", label="Original Points", zorder=1)

    # Plot high-resolution smoothed raceline
    plt.plot(
        x_high_res, y_high_res, label="High-Resolution Smoothed Raceline", zorder=2
    )

    # Highlight braking zones (drawn on top with higher zorder)
    for idx in braking_zones:
        plt.scatter(x_high_res[idx], y_high_res[idx], color="red", s=20, zorder=3)

    # Highlight acceleration zones (drawn on top with higher zorder)
    for idx in acceleration_zones:
        plt.scatter(x_high_res[idx], y_high_res[idx], color="green", s=20, zorder=3)

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
