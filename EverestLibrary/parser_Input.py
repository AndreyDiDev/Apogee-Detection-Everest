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
typedef struct {
    float time;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float magX, magY, magZ;
    float altitude;
} IMUData;

typedef struct {
    float time;
    float pressure;
    float temperature;
    float altitude;
} BarosData;
"""

# Initialize time variables
time_increment = 1 / 3
current_time = 0

# Generate the IMUData and BarosData arrays
cpp_code += "\nIMUData imuData[] = {\n"
for data in input_data:
    cpp_code += f"    {{{current_time}, {data[3]}, {data[4]}, {data[5]}, {data[0]}, {data[1]}, {data[2]}, {data[6]}, {data[7]}, {data[8]}, {data[10]}}},\n"
    current_time += time_increment
cpp_code += "};\n"

cpp_code += "\nBarosData baroData[] = {\n"
for data in input_data:
    cpp_code += f"    {{{current_time}, {data[10]}, 0, 0}},\n"
    current_time += time_increment
cpp_code += "};\n"

# Write the generated C++ code to a file
with open('input_data.cpp', 'w') as file:
    file.write(cpp_code)

print("input_data.cpp file has been generated.")