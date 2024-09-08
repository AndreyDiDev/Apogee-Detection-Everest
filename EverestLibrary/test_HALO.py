import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('HALO1.txt', delimiter = ',')
sims = pd.read_csv('beforeSimsF2_Short.csv')

# plot Madgwick

# plot altitude (altimeter, baro, imu, everest, halo, gps, sigmaPoints, scenarios)
plt.figure(figsize=(10, 6))
# plt.plot(df['Time'], df['Altimeter_Alt'], label='Altimeter Alt')
# plt.plot(df['Time'], df['Baro_Alt'], label='Baro Alt')
# plt.plot(df['Time'], df['IMU_Alt'], label='IMU Alt')
plt.plot(df['Time'], df['Everest_Alt'], label='Everest Alt')
plt.plot(df['Time'], df['Halo_ALt'], label='HALO Alt')
# plt.plot(df['Time'], df['GPS_Alt'], label='GPS Alt')
# plt.plot(df['Time'], df['Sigma_Alt_Upper'], label='Sigma Alt Upper', marker = "x")
# plt.plot(df['Time'], df['Sigma_Alt_Lower'], label='Sigma Alt Lower', marker = "o")
plt.plot(sims['time_1'], sims['alt_1'], label='Scenarios Alt1')
plt.plot(sims['time_2'], sims['alt_2'], label='Scenarios Alt2')
plt.plot(sims['time_3'], sims['alt_3'], label='Scenarios Alt3')
plt.plot(sims['time_4'], sims['alt_4'], label='Scenarios Alt4')
plt.plot(sims['time_5'], sims['alt_5'], label='Scenarios Alt5')
# change color 
plt.plot(sims['time_6'], sims['alt_6'], color = "red", label='Altimeter')

plt.xlabel('Time')
plt.ylabel('Altitude')
plt.title('Time vs Alt')
plt.legend()
plt.grid(True)

# plot velo     (altimeter, imu/everest, halo, gps, sigma, scenarios)
plt.figure(figsize=(10, 6))
# plt.plot(df['Time'], df['Altimeter_Velo'], label='Altimeter Velo')
plt.plot(df['Time'], df['Everest_Velo'], label='Everest/IMU Velo')
plt.plot(df['Time'], df[' Halo_Velo'], label='HALO Velo')
# plt.plot(df['Time'], df['Sigma_Velo_Upper'], label='Sigma Alt Upper', marker = "x")
# plt.plot(df['Time'], df['Sigma_Velo_Upper'], label='Sigma Alt Lower', marker = "o")
# plt.plot(df['Time'], df['Scenarios_Velo1'], label='Scenarios Velo1')
# plt.plot(df['Time'], df['Scenarios_Velo2'], label='Scenarios Velo2')
# plt.plot(df['Time'], df['Scenarios_Velo3'], label='Scenarios Velo3')
# plt.plot(df['Time'], df['Scenarios_Velo4'], label='Scenarios Velo4')
# plt.plot(df['Time'], df['Scenarios_Velo5'], label='Scenarios Velo5')
# plt.plot(df['Time'], df['Scenarios_Velo6'], label='Scenarios Velo6')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Time vs Velo')
plt.legend()
plt.grid(True)

# plot accel    (altimeter, everest, halo, gps, sigma, scenarios)
plt.figure(figsize=(10, 6))
# plt.plot(df['Time'], df['Altimeter_Acc'], label='Altimeter Acc')
plt.plot(df['Time'], df['Everest_Accel'], label='Everest/IMU Acc')
plt.plot(df['Time'], df[' Halo_accel'], label='HALO Acc')
# plt.plot(df['Time'], df['Sigma_Acc_Upper'], label='Sigma Acc Upper', marker = "x")
# plt.plot(df['Time'], df['Sigma_Acc_Upper'], label='Sigma Acc Lower', marker = "o")
# plt.plot(df['Time'], df['Scenarios_Acc1'], label='Scenarios Acc1')
# plt.plot(df['Time'], df['Scenarios_Acc2'], label='Scenarios Acc2')
# plt.plot(df['Time'], df['Scenarios_Acc3'], label='Scenarios Acc3')
# plt.plot(df['Time'], df['Scenarios_Acc4'], label='Scenarios Acc4')
# plt.plot(df['Time'], df['Scenarios_Acc5'], label='Scenarios Acc5')
# plt.plot(df['Time'], df['Scenarios_Acc6'], label='Scenarios Acc6')
plt.xlabel('Time')
plt.ylabel('Acc')
plt.title('Time vs Acc')
plt.legend()
plt.grid(True)


plt.show()



