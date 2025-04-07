import tkinter as tk
from tkinter import messagebox
import serial
import threading
import csv
import os
from datetime import datetime

class ControlPanelGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Panel GUI")

        # Serial communication setup
        self.serial_port = "COM9"  # Change to your Arduino's COM port
        self.baud_rate = 9600
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Could not open serial port: {e}")
            self.arduino = None

        # Create buttons
        self.end_button = tk.Button(root, text="End", command=self.send_end_command, width=20)
        self.end_button.pack(pady=10)

        self.pause_button = tk.Button(root, text="Pause", command=self.send_pause_command, width=20)
        self.pause_button.pack(pady=10)

        self.resume_button = tk.Button(root, text="Resume", command=self.send_resume_command, width=20)
        self.resume_button.pack(pady=10)

        self.panel_ready_button = tk.Button(root, text="Panel Ready", command=self.send_panel_ready_command, width=20)
        self.panel_ready_button.pack(pady=10)

        # Log area
        self.log_text = tk.Text(root, height=10, width=50, state="disabled")
        self.log_text.pack(pady=10)

        # CSV file setup
        self.setup_csv()

        # Start a thread to listen for incoming messages
        if self.arduino:
            self.listening_thread = threading.Thread(target=self.listen_to_serial, daemon=True)
            self.listening_thread.start()

        # Bind the close event to call send_end_command
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_csv(self):
        """Set up the CSV file with a header if it doesn't already exist."""
        # Create the data directory if it doesn't exist
        self.data_dir = "data"
        os.makedirs(self.data_dir, exist_ok=True)

        # Generate a filename with the current date and time
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_file = os.path.join(self.data_dir, f"serial_log_{current_time}.csv")

        # Create the CSV file with a header
        with open(self.csv_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Message"])  # Add a header row

    def write_to_csv(self, message):
        """Write a message to the CSV file."""
        # with open(self.csv_file, mode="a", newline="") as file:
        #     writer = csv.writer(file, quoting=csv.QUOTE_MINIMAL)
        #     writer.writerow([message])
        with open(self.csv_file, mode="a", newline="") as file:
            file.write(message + "\n")  # Write the message directly without any quoting or escaping

    def send_command(self, command):
        if self.arduino:
            try:
                self.arduino.write((command + "\n").encode())
                self.log_message(f"Sent: {command}")
            except serial.SerialException as e:
                messagebox.showerror("Error", f"Failed to send command: {e}")
        else:
            messagebox.showerror("Error", "Serial connection not established")

    def send_end_command(self):
        self.send_command("E")

    def send_pause_command(self):
        self.send_command("P")

    def send_resume_command(self):
        self.send_command("R")

    def send_panel_ready_command(self):
        self.send_command("PANEL_READY")

    def listen_to_serial(self):
        while True:
            if self.arduino and self.arduino.in_waiting > 0:
                try:
                    message = self.arduino.readline().decode().strip()
                    self.log_message(f"Received: {message}")
                    self.write_to_csv(message)  # Save the message to the CSV file
                except serial.SerialException as e:
                    self.log_message(f"Error reading from serial: {e}")
                    break

    def log_message(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.config(state="disabled")
        self.log_text.see("end")

    def on_close(self):
        # Call send_end_command before closing
        self.send_end_command()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlPanelGUI(root)
    root.mainloop()