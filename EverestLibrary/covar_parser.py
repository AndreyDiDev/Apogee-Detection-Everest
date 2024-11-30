import pandas as pd

df = pd.read_csv('covar.csv')

# iterate and for every up to 0.1 average 
for i in range(len(df)):
    