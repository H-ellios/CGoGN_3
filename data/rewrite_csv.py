import pandas as pd
import os
from os import listdir
from os.path import isfile, join
import sys

filepath = sys.argv[1]

onlyfiles = [f for f in listdir(filepath) if isfile(join(filepath, f))]

onlyfiles.sort()

for f in onlyfiles : 
    if f.endswith('.csv') :
        
        dataframe = pd.read_csv(filepath + f , sep=',')
        if(dataframe.columns.size == 1) : 
            dataframe = pd.read_csv(filepath + f , sep=';')

        columns = [col for col in dataframe.columns if col.startswith(" AU28_c") or col.startswith("AU28_c") or col.startswith("face_id") or col.startswith("confidence") or col.startswith("success")]
        csv = False
        if(len(columns) > 0) : 
            dataframe = dataframe.drop(columns=columns)
            csv = True

        if(csv) : 
            dataframe.to_csv(join(filepath,f) , index=None , sep=',')
