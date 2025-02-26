import matplotlib.pyplot as plt
import pandas as pd

# Load the CSV file
csv_filename = "Fase4.csv"
data = pd.read_csv(csv_filename)

# Extract X position and speed
x_position = data["X"]
robot_speed = data["Linear Speed"]

# Plot X position vs. speed
plt.figure(figsize=(10, 5))
plt.plot(x_position, robot_speed, marker='o', linestyle='-', color='b', label="Robot Speed")

# Labels and title
plt.xlabel("Robot Position (m)")
plt.ylabel("Speed (m/s)")
plt.title("Husky Robot Speed vs. Position")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()