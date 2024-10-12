import csv

# Read the scenario data from a CSV file
scenarios = []
with open('beforeSimsF2_Short.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        scenarios.append(row)

# Generate the C++ code
cpp_code = """
#include <vector>
"""

# Dictionary to hold the data for each scenario
scenario_data = {}

# Generate separate lists for each row
for i, scenario in enumerate(scenarios):
    scenario_id = i // 100 + 1  # Assuming each scenario has 100 data points
    if scenario_id not in scenario_data:
        scenario_data[scenario_id] = []
    scenario_data[scenario_id].append(scenario)

# Append data to the existing SensorData arrays
for scenario_id, data in scenario_data.items():
    # check if the data is not all NUL
    
    # Scenario 1
    cpp_code += f"\nstd::vector<std::vector<float>> sim{1} = {{\n"
    for entry in data:
        if entry[1] == '':
            break
        cpp_code += f"    {{{entry[0]}, {entry[1]}, {entry[2]}, {entry[3]}}},\n"
    cpp_code += "};\n"
    
    # Scenario 2
    cpp_code += f"\nstd::vector<std::vector<float>> sim{2} = {{\n"
    for entry in data:
        
        if entry[4] == '':
            break
            
        cpp_code += f"    {{{entry[4]}, {entry[5]}, {entry[6]}, {entry[7]}}},\n"
    cpp_code += "};\n"
    
    # Scenario 3
    cpp_code += f"\nstd::vector<std::vector<float>> sim{3} = {{\n"
    for entry in data:
        
        if entry[8] == '':
            break
        
        cpp_code += f"    {{{entry[8]}, {entry[9]}, {entry[10]}, {entry[11]}}},\n"
    cpp_code += "};\n"
    
    # Scenario 4
    cpp_code += f"\nstd::vector<std::vector<float>> sim{4} = {{\n"
    for entry in data:
        if entry[12] == '':
            break
            
        cpp_code += f"    {{{entry[12]}, {entry[13]}, {entry[14]}, {entry[15]}}},\n"
    cpp_code += "};\n"
    
    # Scenario 5
    cpp_code += f"\nstd::vector<std::vector<float>> sim{5} = {{\n"
    for entry in data:
        if entry[16] == '':
            break
        cpp_code += f"    {{{entry[16]}, {entry[17]}, {entry[18]}, {entry[19]}}},\n"
    cpp_code += "};\n"
    
    # Scenario 6
    cpp_code += f"\nstd::vector<std::vector<float>> sim{6} = {{\n"
    for entry in data:
        if entry[20] == '':
            break
        cpp_code += f"    {{{entry[20]}, {entry[21]}, {entry[22]}, {entry[23]}}},\n"
    cpp_code += "};\n"


# Write the generated C++ code to a file
with open('Data.cpp', 'w') as file:
    file.write(cpp_code)

print("Data.cpp file has been generated.")