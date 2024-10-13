import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('HALO.txt', delimiter = ',')
sims = pd.read_csv('beforeSimsF2_Short.csv')
predictedValues = pd.read_csv('predictedValues.txt')
gains = pd.read_csv('gains.txt')
sigmaPoints = pd.read_csv('sigmaPoints.txt')
sigmaPoints1 = pd.read_csv('sigmaPoints1.txt')
sigmaPoints2 = pd.read_csv('sigmaPoints2.txt')
sigmaPoints3 = pd.read_csv('sigmaPoints3.txt')
sigmaPoints4 = pd.read_csv('sigmaPoints4.txt')
sigmaPoints5 = pd.read_csv('sigmaPoints5.txt')
sigmaPoints6 = pd.read_csv('sigmaPoints6.txt')
nearestScenarios = pd.read_csv('nearestScenarios.txt')

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

# # plot sPoints w/ final HALO and Truth alt ----------------------------------------------------
plt.figure(figsize=(10, 6))
theTime = df['Time']
theTime = [0, 0.33, 0.66, 1, 1.33, 1.66, 2] + theTime.tolist()

# print(theTime)
# print(sigmaPoints['alt'])

alt1 = sigmaPoints['alt']
alt1 = alt1[2:]

alt2 = sigmaPoints1['alt']
alt2 = alt2[2:]

alt3 = sigmaPoints2['alt']
alt3 = alt3[2:]

alt4 = sigmaPoints3['alt']
alt4 = alt4[2:]

alt5 = sigmaPoints4['alt']
alt5 = alt5[2:]

alt6 = sigmaPoints5['alt']
alt6 = alt6[2:]

alt7 = sigmaPoints6['alt']
alt7 = alt7[2:]

plt.plot(df['Time'], alt1, label='s1_alt')
plt.plot(df['Time'], alt2, label='s2_alt')
plt.plot(df['Time'], alt3, label='s3_alt')
plt.plot(df['Time'], alt4, label='s4_alt')
plt.plot(df['Time'], alt5, label='s5_alt')
plt.plot(df['Time'], alt6, label='s6_alt')
plt.plot(df['Time'], alt7, label='s7_alt')
plt.plot(df['Time'], df['Everest_Alt'], label='Everest/IMU Alt', marker = "o", markevery=5)
plt.plot(df['Time'], df['Halo_ALt'], label='HALO Alt', marker = "x", markevery=5)
plt.plot(sims['time_6'], sims['alt_6'], label='Altimeter', marker = "s", markevery=5)
plt.plot()
plt.xlabel('Time')
plt.ylabel('Alt')
plt.title('Time vs Alt')
plt.legend()
plt.grid(True)

# # plot sPoints w/ final HALO and Truth velo ----------------------------------------------------
plt.figure(figsize=(10, 6))

vel1 = sigmaPoints['velo']
vel1 = vel1[2:]

vel2 = sigmaPoints1['velo']
vel2 = vel2[2:]

vel3 = sigmaPoints2['velo']
vel3 = vel3[2:]

vel4 = sigmaPoints3['velo']
vel4 = vel4[2:]

vel5 = sigmaPoints4['velo']
vel5 = vel5[2:]

vel6 = sigmaPoints5['velo']
vel6 = vel6[2:]

vel7 = sigmaPoints6['velo']
vel7 = vel7[2:]

plt.plot(df['Time'], vel1, label='s1_vel')
plt.plot(df['Time'], vel2, label='s2_vel')
plt.plot(df['Time'], vel3, label='s3_vel')
plt.plot(df['Time'], vel4, label='s4_vel')
plt.plot(df['Time'], vel5, label='s5_vel')
plt.plot(df['Time'], vel6, label='s6_vel')
plt.plot(df['Time'], vel7, label='s7_vel')
plt.plot(df['Time'], df['Everest_Velo'], label='Everest/IMU Velo', marker = "o")
plt.plot(sims['time_6'], sims['velo_6'], label='Altimeter', marker = "s", markevery=5)
plt.plot(df['Time'], df[' Halo_Velo'], label='HALO Velo', marker = "x")
plt.xlabel('Time')
plt.ylabel('Velo')
plt.title('Time vs Velo')
plt.legend()
plt.grid(True)

residual_halo = sims['alt_6'] - df['Halo_ALt']
residual_everest = sims['alt_6'] - df['Everest_Alt']

plt.figure(figsize=(10, 6))
plt.plot(df['Time'], residual_halo, label='residual_halo')
plt.plot(df['Time'], residual_everest, label='residual_everest')
plt.legend()
plt.grid(True)

# -------------------------------------------------------------------------------

# plot scenario choices
plt.figure(figsize=(10, 6))
plt.plot(nearestScenarios['firstScenario'], 'ok', label='Scenario 1', )
plt.plot(nearestScenarios['SecondScenario'], 'o', label='Scenario 2')
plt.legend()
plt.grid(True)

# plot distances
plt.figure(figsize=(10, 6))
plt.plot(nearestScenarios['lowestDistance'], label='lowestDistance 1')
plt.plot(nearestScenarios['secondLowestDistance'], label='secondLowestDistance 2')
plt.legend()
plt.grid(True)

plt.show()



