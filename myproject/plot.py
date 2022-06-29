import csv
import matplotlib.pyplot as plt

time = []
x_reference = []
x_close_command = []
x_close_mesure = []
z_reference = []
z_close_command = []
z_close_mesure = []

with open("data/results/closedLoop.csv", 'r') as file:
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

fig, axs = plt.subplots(2)
fig.suptitle('Closed Loop Behavior')

axs[0].plot(time, x_close_mesure,color = 'g',label = 'Closed Loop mesure')
axs[0].plot(time, x_reference,color = 'r',label = 'reference')
axs[0].legend(loc = 'upper left')
axs[0].set_title('Rotation around x axis')

axs[1].plot(time, z_close_mesure,color = 'g',label = 'Closed Loop mesure')
axs[1].plot(time, z_reference,color = 'r',label = 'reference')
axs[1].legend(loc = 'upper left')
axs[1].set_title('Rotation around z axis')

# Set common labels
axs[1].set_xlabel('time(s)')
fig.text(0.06, 0.5, 'rotation (rad)', ha='center', va='center', rotation='vertical')
fig.tight_layout(pad=3.0)
plt.show()