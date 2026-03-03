import re
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

def fit_velocity_data():
    # Define the expected command velocities based on filenames
    # You can also parse this dynamically if needed
    file_map = {
        '0.4log.txt': 0.4,
        '0.35log.txt': 0.35,
        '0.3log.txt': 0.3,
        '0.25log.txt': 0.25,
        '0.2log.txt': 0.2,
        '0.15log.txt': 0.15,
        '0.1log.txt': 0.1
    }

    data_points = []
    
    # Regex pattern to extract the first number after "acc["
    # Matches: acc[ <number>, ...
    pattern = re.compile(r'acc\[\s*([-+]?\d*\.?\d+)')

    print("Reading files and extracting data...")
    
    for filename, cmd_val in file_map.items():
        try:
            with open(filename, 'r') as f:
                content = f.read()
                matches = pattern.findall(content)
                
                if not matches:
                    print(f"Warning: No data found in {filename}")
                    continue
                    
                # Convert strings to floats and store
                velocities = [float(m) for m in matches]
                
                # Store each point individually for scatter plot
                for v in velocities:
                    data_points.append({
                        'cmd_vel': cmd_val,
                        'measured_vx': v
                    })
                    
                print(f"Processed {filename}: {len(velocities)} points found. Mean vx: {np.mean(velocities):.4f}")
                
        except FileNotFoundError:
            print(f"Error: File {filename} not found.")
        except Exception as e:
            print(f"Error reading {filename}: {e}")

    if not data_points:
        print("No valid data points found to fit.")
        return

    # Convert to DataFrame
    df = pd.DataFrame(data_points)

    # Prepare data for fitting
    X = df[['cmd_vel']].values.reshape(-1, 1)
    y = df['measured_vx'].values

    # Perform Linear Regression
    reg = LinearRegression()
    reg.fit(X, y)
    
    slope = reg.coef_[0]
    intercept = reg.intercept_

    print("-" * 30)
    print(f"Fitting Result:")
    print(f"Slope (k): {slope:.4f}")
    print(f"Intercept (b): {intercept:.4f}")
    print(f"Equation: measured_vx = {slope:.4f} * cmd_vel + {intercept:.4f}")
    print("-" * 30)

    # Visualization
    plt.figure(figsize=(10, 6))
    
    # Plot raw data points
    plt.scatter(df['cmd_vel'], df['measured_vx'], alpha=0.3, color='blue', label='Raw Measurements')
    
    # Plot the regression line
    x_range = np.linspace(df['cmd_vel'].min(), df['cmd_vel'].max(), 100).reshape(-1, 1)
    y_pred = reg.predict(x_range)
    plt.plot(x_range, y_pred, color='red', linewidth=2, label=f'Fit: y={slope:.2f}x + {intercept:.2f}')
    
    # Calculate means for visualization
    means = df.groupby('cmd_vel')['measured_vx'].mean().reset_index()
    plt.scatter(means['cmd_vel'], means['measured_vx'], color='black', marker='x', s=100, label='Mean per Command')

    plt.title('Command Velocity (cmd_vel) vs Measured Velocity (vx)')
    plt.xlabel('Command Velocity (m/s)')
    plt.ylabel('Measured Velocity (m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()

if __name__ == "__main__":
    fit_velocity_data()