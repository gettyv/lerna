import os
import pandas as pd
import matplotlib.pyplot as plt

def get_most_recent_log_file(directory):
    """Get the most recent serial log file from the specified directory."""
    files = [f for f in os.listdir(directory) if f.startswith("serial_log") and f.endswith(".csv")]
    if not files:
        raise FileNotFoundError("No serial log files found in the directory.")
    files.sort(key=lambda x: os.path.getmtime(os.path.join(directory, x)), reverse=True)
    return os.path.join(directory, files[0])

def filter_and_plot_serial_log(directory):
    """Filter non-data entries from the most recent serial log file and plot time series graphs."""
    # Get the most recent log file
    log_file = get_most_recent_log_file(directory)
    print(f"Processing file: {log_file}")

    # Read the file and find the header row
    with open(log_file, 'r') as file:
        lines = file.readlines()

    # Find the line where the column headers start
    header_index = next(i for i, line in enumerate(lines) if line.strip().startswith("time,"))
    headers = lines[header_index].strip().split(",")
    
    # Load the data starting from the header row
    data = pd.read_csv(log_file, skiprows=header_index + 1, names=headers)

    # Filter out rows with non-numeric data
    data = data.apply(pd.to_numeric, errors='coerce').dropna()

    # Plot each column as a time series (excluding the 'time' column)
    time_column = 'time'
    for column in data.columns:
        if column != time_column:
            plt.figure()
            plt.plot(data[time_column], data[column], label=column)
            plt.xlabel('Time')
            plt.ylabel(column)
            plt.title(f'Time Series for {column}')
            plt.legend()
            plt.grid()
            plt.show()

# Example usage
if __name__ == "__main__":
    log_directory = "data"  # Adjust the path to your data directory
    filter_and_plot_serial_log(log_directory)