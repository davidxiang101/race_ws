import numpy as np
from scipy.interpolate import UnivariateSpline
import csv
import matplotlib.pyplot as plt

def read_race_line(filename):
    x, y = [], []
    with open(filename, mode='r') as csvfile:
        csv_reader = csv.reader(csvfile)
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            # speeds.append(float(row[2]))
    # x.append(x[0])
    # y.append(y[0])
    return np.array(x), np.array(y)

def save_smooth_race_line(x, y, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for xi, yi in zip(x, y):
            writer.writerow([xi, yi, 0.0, 1.0])

def smooth_race_line(x, y, s=0.5):
    
    # Fit splines to both coordinates. Adjust the smoothing factor 's' as needed.
    spline_x = UnivariateSpline(range(len(x)), x, s=s)
    spline_y = UnivariateSpline(range(len(y)), y, s=s)
    
    # Generate new, evenly spaced indices to sample from the spline
    x_range = np.linspace(0, len(x) - 1, len(x) * 10)
    
    # Sample the splines
    x_smooth = spline_x(x_range)
    y_smooth = spline_y(x_range)
    
    return x_smooth, y_smooth

# Main script
filename = 'demoline.csv'
x, y = read_race_line(filename)
x_smooth, y_smooth = smooth_race_line(x, y, s=0.5)  # Adjust smoothing factor as needed

# # Assume constant speed for the smoothed points (or interpolate based on original speeds)
# speeds_smooth = np.full_like(x_smooth, speeds.mean())

save_smooth_race_line(x_smooth, y_smooth, filename.replace('.csv', '_smooth.csv'))

# Optional: plot original and smoothed race line for comparison
plt.figure()
plt.plot(x, y, 'ro-', label='Original')
plt.plot(x_smooth, y_smooth, 'b-', label='Smoothed')
plt.legend()
plt.show()
