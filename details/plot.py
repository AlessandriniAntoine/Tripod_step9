import csv
import matplotlib.pyplot as plt

time = []
x_input = []
x_output = []
x_tracking = []
z_input = []
z_output = []
z_tracking = []

with open("closeLoop.csv", 'r') as file:
    csvreader = csv.reader(file)
    # header = next(csvreader)
    for row in csvreader:
        time.append(float(row[0]))
        x_input.append(float(row[1]))
        x_output.append(float(row[2]))
        x_tracking.append(float(row[3]))
        z_input.append(float(row[4]))
        z_output.append(float(row[5]))
        z_tracking.append(float(row[6]))

x_open_input = []
x_open_tracking = []
z_open_input = []
z_open_tracking = []

with open("openLoop.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        x_open_input.append(float(row[1]))
        x_open_tracking.append(float(row[2]))
        z_open_input.append(float(row[3]))
        z_open_tracking.append(float(row[4]))


fig, axs = plt.subplots(2)
fig.suptitle('Open Loop and Close Loop Behavior')

axs[0].plot(time[100:600], x_open_tracking[100:600],color = 'b',label = 'Open Loop ouput')
axs[0].plot(time[100:600], x_tracking[100:600],color = 'g',label = 'Close Loop ouput')
axs[0].plot(time[100:600], x_open_input[100:600],color = 'r',label = 'command')
axs[0].legend(loc = 'upper left')
axs[0].set_title('Rotation around x axe')

axs[1].plot(time[100:600], z_open_tracking[100:600],color = 'b',label = 'Open Loop ouput')
axs[1].plot(time[100:600], z_tracking[100:600],color = 'g',label = 'Close Loop ouput')
axs[1].plot(time[100:600], z_open_input[100:600],color = 'r',label = 'command')
axs[1].legend(loc = 'upper left')
axs[1].set_title('Rotation around z axe')

# Set common labels
axs[1].set_xlabel('time(s)')
fig.text(0.06, 0.5, 'rotation (rad)', ha='center', va='center', rotation='vertical')
fig.tight_layout(pad=3.0)
plt.show()