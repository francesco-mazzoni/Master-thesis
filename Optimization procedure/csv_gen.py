# TRIAL CODE DOCUMENT FOR THESIS BEGINNING
#
# In this code I will try to import some libraries that can be useful for the thesis.
# I'll also use the proper python version to see if everything works. In future, this can be seen as a debug
# file to work on, in order to see if everything works

import fastf1
import os
from fastf1 import plotting
from matplotlib import pyplot as plt
import pandas as pd

path      = '/Users/francescomazzoni/Documents/Thesis_coding/Ch_data'

print(os.path.isdir(path))

fastf1.Cache.enable_cache(path)

plotting.setup_mpl()

# We are interested in the circuits of:
# Spielberg (Austria, 11th GP)
# Monza (Italy, 16th GP) 
# Austin (USA, 19th GP)

# Extract circuit name for saving purposes and circuit data

circuit_list = ['Spielberg', 'Monza', 'Austin']
CIRCUIT = circuit_list[1]
circuit = fastf1.get_session(2022, CIRCUIT, 'Q')

nation_list = ['_aut_', '_ita_', '_usa_']
NATION_ABB = nation_list[1]

# Check consistency of race event number and event name
print(circuit.event['EventName'])

# Extract timelaps for a given driver
driver_list  = ['VER', 'PER', 'LEC', 'SAI', 'HAM', 'RUS']
DRIVER = driver_list[2]

# Warnings on Austrian GP
# Notes on Q1: SAI was recorded at its second launched lap following the very first tentative
# Notes on Q2: PER and HAM recorded at second trial due to track limits and to crash in Q3 respectively,
#              HAM recorded twice
#              SAI onboard changed during launched lap in Q2, better to change

# Warnings on Italian GP
# Notes on Q1: SAI onboard was recorded from his helmet thus data are available, but video may be bad;
#               thus another recording is suggested
# Notes on Q2: LEC onboard was recorded from his helmet thus data are available, but video may be bad;
#               thus another recording is suggested
#              HAM onboard not perfectly on top of helmet (don't know if it is the case to record another video)

# Warnings on American GP
# Notes on Q1: HAM onboard changed during Q1 session launched lap, better to add another recording
#              VER onboard not perfectly on top of helmet (don't know if it is the case to record another video)
# Notes on Q2: LEC onboard lost the signal during launched lap (better to add another video)
#              RUS onboard positioned on front left wing instead of being on top of the helmet (better to change)
#              VER onboard not perfectly on top of helmet (don't know if it is the case to record another video)
#              Moreover, onboard changed during one single lap
#

q_session_list = ['Q1', 'Q2', 'Q3']
Q_SESSION = q_session_list[2]
print(DRIVER+'\n'+ Q_SESSION + '\n') #sanity check 

# any session specific data like lap timing, telemetry, … is not yet loaded. For this, you will need to call load() on the returned object.
circuit.load()
#print(circuit.laps.columns) # what are the columns available in this dataset
print('\n ------ \n Show the lap times of ' + DRIVER + ' then choose the correct lap \n ------\n')
print(circuit.laps.pick_driver(DRIVER).loc[:,['Time','Sector2Time','LapTime']])
print('\n')

lap_number = 11

# Now I select the rows referring to the recorded track in Qualifying
print(circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().columns)
print('\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
date_ax = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Date']
#print(circuit.laps.pick_driver(DRIVER).loc[1,:].get_pos_data().loc[:,['SessionTime','Time','X','Y','Z']])
print('--------------------------\n')
print(circuit.laps.pick_driver(DRIVER))
q = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,['Time','Speed','Distance']]
print(q)

print('\n')

timeax = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Time']
speedax = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Speed']
distax = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Distance']
coord_x = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'X']
coord_y = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Y']
coord_z = circuit.laps.pick_driver(DRIVER).loc[lap_number,:].get_telemetry().loc[:,'Z']
print('\n')
print(coord_x/10-coord_x[2]/10)
print('\n')
coords_df = pd.DataFrame({'Time': timeax,
                          'X': coord_x,
                          'Y': coord_y,
                          'Z': coord_z})

coords_df2 = pd.DataFrame({'Data': date_ax,
                          'X': coord_x,
                          'Y': coord_y,
                          'Z': coord_z})

output_df = pd.DataFrame({'Time': timeax,
                          'Speed': speedax,
                          'Distance': distax,
                          'X': coord_x,#coord_x/10-coord_x[2]/10,
                          'Y': coord_y,#coord_y/10-coord_y[2]/10,
                          'Z': coord_z})#coord_z/10-coord_z[2]/10})


#coords_df.to_csv('COORDS_' + Q_SESSION + NATION_ABB + DRIVER + '.csv')
output_df.to_csv(CIRCUIT + '/' + Q_SESSION + '/' + Q_SESSION + NATION_ABB + DRIVER + '_complete2' + '.csv')
#output_df.to_csv('DOVE_CAVOLO_SI_SALVA.csv')
#output_df.to_csv(CIRCUIT + '/' + Q_SESSION + '/' + Q_SESSION + NATION_ABB + DRIVER + '2.csv')
