import serial
import json
import matplotlib.pyplot as plt
import csv

# ---------------- Serial configuration ----------------
PORT = "COM5"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------------- Plot setup ----------------
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], marker='o', linestyle='-')

ax.set_xlabel("Sequence Number")
ax.set_ylabel("Temperature (°C)")
ax.set_title("Temperature vs Sequence")
ax.grid(True)

seqs = []
temps = []

csvfile = open("log.csv", "a", newline="")
writer = csv.writer(csvfile)
writer.writerow(["seq", "temperature"])

print("Listening for data...")

# ---------------- Main loop ----------------
while True:
    try:
        # Read one line from UART
        line_in = ser.readline().decode(errors="ignore").strip()
        if not line_in:
            continue

        # Parse JSON
        data = json.loads(line_in)

        # Extract numeric values
        seq = int(data["seq"])
        temp = float(data["temperature"])

        writer.writerow([seq, temp])
        csvfile.flush()

        # Append new data
        seqs.append(seq)
        temps.append(temp)

        # Update plot
        line.set_xdata(seqs)
        line.set_ydata(temps)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

    except json.JSONDecodeError:
        print("JSON parse failed:", line_in)