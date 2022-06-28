import csv
import matplotlib.pyplot as plt

time = []
x_reference = []
x_close_command = []
x_close_mesure = []
z_reference = []
z_close_command = []
z_close_mesure = []

with open("data/results/closeLoop.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        time.append(float(row[0]))
        x_reference.append(float(row[1]))
        x_close_command.append(float(row[2]))
        x_close_mesure.append(float(row[3]))
        z_reference.append(float(row[4]))
        z_close_command.append(float(row[5]))
        z_close_mesure.append(float(row[6]))

x_open_mesure = []
z_open_mesure = []

with open("data/results/openLoop.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        x_open_mesure.append(float(row[2]))
        z_open_mesure.append(float(row[4]))


fig, axs = plt.subplots(2)
fig.suptitle('Open Loop and Close Loop Behavior')

axs[0].plot(time[100:130], x_open_mesure[100:130],color = 'b',label = 'Open Loop mesure')
axs[0].plot(time[100:130], x_close_mesure[100:130],color = 'g',label = 'Close Loop mesure')
axs[0].plot(time[100:130], x_reference[100:130],color = 'r',label = 'reference')
axs[0].legend(loc = 'upper left')
axs[0].set_title('Rotation around x axe')

axs[1].plot(time[100:130], z_open_mesure[100:130],color = 'b',label = 'Open Loop mesure')
axs[1].plot(time[100:130], z_close_mesure[100:130],color = 'g',label = 'Close Loop mesure')
axs[1].plot(time[100:130], z_reference[100:130],color = 'r',label = 'reference')
axs[1].legend(loc = 'upper left')
axs[1].set_title('Rotation around z axe')

# Set common labels
axs[1].set_xlabel('time(s)')
fig.text(0.06, 0.5, 'rotation (rad)', ha='center', va='center', rotation='vertical')
fig.tight_layout(pad=3.0)
plt.show()