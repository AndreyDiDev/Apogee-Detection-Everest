
# print(PYTHON_INCLUDE_PATH)

# import sys 
# help('modules')
# print(sys.path)
import sys
# sys.path.append('''C:/Users/Andrey/Documents/AHRSRepo/MadgwickAHRS/Python''')

# import imufusion
import matplotlib.pyplot as pyplot
import numpy

print("running this 2")

# Import sensor data
data = numpy.genfromtxt("C:/Users/Andrey/Documents/EverestRepo/Apogee-Detection-Everest/MadgwickLibrary/infusion.txt", delimiter=",", skip_header=1)

# with open('C:/Users/Andrey/Documents/AHRSRepo/MadgwickAHRS/newSimsTxt.txt') as fin, open('newfile.txt', 'w') as fout:
#     for line in fin:
#         fout.write(line.replace('\t', ','))

# data1 = numpy.genfromtxt("C:/Users/Andrey/Documents/AHRSRepo/MadgwickAHRS/newFile.txt", delimiter=",", skip_header=1)

sample_rate = 100  # 100 Hz

# print(data1)
# print(data[0])
# print(data[0:14, 0:19])

timestamp = data[:, 0]


euler = data[:, 1:4]
internal_states = numpy.array(data[:, 4:10])
flags = numpy.array(data[:, 10:15])

print(timestamp[-2] - timestamp[-1])
# print(timestamp[0])
# print(euler[0])
# print(internal_states[0])
# print(flags[0])

def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()


# Plot Euler angles
figure, axes = pyplot.subplots(nrows=11, sharex=True, gridspec_kw={"height_ratios": [6, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1]})

figure.suptitle("Euler angles, internal states, and flags")

axes[0].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[0].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[0].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[0].set_ylabel("Degrees")
axes[0].grid()
axes[0].legend()

# Plot initialising flag
plot_bool(axes[1], timestamp, flags[:, 0], "Initialising")

# Plot angular rate recovery flag
plot_bool(axes[2], timestamp, flags[:, 1], "Angular rate recovery")

# Plot acceleration rejection internal states and flag
axes[3].plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration error")
axes[3].set_ylabel("Degrees")
axes[3].grid()
axes[3].legend()

plot_bool(axes[4], timestamp, internal_states[:, 1], "Accelerometer ignored")

axes[5].plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration recovery trigger")
axes[5].grid()
axes[5].legend()

plot_bool(axes[6], timestamp, flags[:, 2], "Acceleration recovery")

# Plot magnetic rejection internal states and flag
axes[7].plot(timestamp, internal_states[:, 3], "tab:olive", label="Magnetic error")
axes[7].set_ylabel("Degrees")
axes[7].grid()
axes[7].legend()

plot_bool(axes[8], timestamp, internal_states[:, 4], "Magnetometer ignored")

axes[9].plot(timestamp, internal_states[:, 5], "tab:orange", label="Magnetic recovery trigger")
axes[9].grid()
axes[9].legend()

plot_bool(axes[10], timestamp, flags[:, 3], "Magnetic recovery")

pyplot.show(block="no_block" not in sys.argv)  # don't block when script run by CI

