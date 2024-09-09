import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('HALO.txt', delimiter = ',')
sims = pd.read_csv('beforeSimsF2_Short.csv')
predictedValues = pd.read_csv('predictedValues.csv')
gains = pd.read_csv('gains.csv')
sigmaPoints = pd.read_csv('sigmaPoints.csv')

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
plt.plot(sims['time_1'], sims['velo_1'], label='Scenarios Velo1')
plt.plot(sims['time_2'], sims['velo_2'], label='Scenarios Velo2')
plt.plot(sims['time_3'], sims['velo_3'], label='Scenarios Velo3')
plt.plot(sims['time_4'], sims['velo_4'], label='Scenarios Velo4')
plt.plot(sims['time_5'], sims['velo_5'], label='Scenarios Velo5')
plt.plot(sims['time_6'], sims['velo_6'], label='Altimeter')
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
plt.plot(sims['time_1'], sims['acc_1'], label='Scenarios Acc1')
plt.plot(sims['time_2'], sims['acc_2'], label='Scenarios Acc2')
plt.plot(sims['time_3'], sims['acc_3'], label='Scenarios Acc3')
plt.plot(sims['time_4'], sims['acc_4'], label='Scenarios Acc4')
plt.plot(sims['time_5'], sims['acc_5'], label='Scenarios Acc5')
plt.plot(sims['time_6'], sims['acc_6'], label='Altimeter')
plt.xlabel('Time')
plt.ylabel('Acc')
plt.title('Time vs Acc')
plt.legend()
plt.grid(True)


plt.show()



