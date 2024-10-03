import csv

# Read the IMU data from a CSV file
input_data = []
with open('Imu_Baro.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        input_data.append(row)

# Generate the C++ code
cpp_code = """
#include <vector>
"""

# Initialize time variables
time_increment = 1 / 3
current_time = 0

# Generate the IMUData and BarosData arrays
cpp_code += "\nstd::vector<std::vector<float>> imuData = {\n"
for data in input_data:
    if data[4] == '':
        break
    cpp_code += f"    {{{current_time}, {float(data[3])/1000}, {float(data[4])/1000}, {float(data[5])/1000}, {float(data[0])/1000}, {float(data[1])/1000}, {float(data[2])/1000}, {float(data[6])/1000}, {float(data[7])/1000}, {float(data[8])/1000}}},\n"
    current_time += time_increment
cpp_code += "};\n"

current_time = 0
cpp_code += "\nstd::vector<std::vector<float>> baroData = {\n"
for data in input_data:
    if data[10] == '':
        break
    cpp_code += f"    {{{current_time}, {data[10]}, 0, 0}},\n"
    current_time += time_increment
cpp_code += "};\n"

# Write the generated C++ code to a file
with open('input_data.cpp', 'w') as file:
    file.write(cpp_code)

print("input_data.cpp file has been generated.")