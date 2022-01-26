import numpy
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import datetime
import numpy as np

import sys

# parameters
stiff_config    = ['low_stiffness', 'variable_stiffness']
exps            = ['exp0', 'exp1', 'exp2', 'exp3', 'exp4']
weights         = ['weight_1', 'weight_2', 'weight_3']

for weight in weights:
    for setting in stiff_config:
        for exp in exps:

            # load the files
            filepath = str(Path(__file__).parents[0] / weight / exp / setting) + "_output.csv"
            data     = pd.read_csv(filepath, delimiter=',')
            
            tvec     = np.zeros((data.shape[0], 1))
            counter  = 0

            for t in data['Time']:
                temp = t.split(':')

                tvec[counter,0] = int(temp[0]) * 3600 + int(temp[1]) * 60 + int(temp[2]) + int(temp[3])/1e6
                counter += 1

            tvec = tvec - tvec[0,0]
            print(np.diff(data['Y'], axis=0))
            plt.plot(tvec[1:,0], np.divide(np.diff(-data['Y']+data['handle_disp']), np.diff(tvec)))
            plt.show()