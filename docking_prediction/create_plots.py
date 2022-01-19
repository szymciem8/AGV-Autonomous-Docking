import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os

np.set_printoptions(precision=3, suppress=True)

path = os.path.join(os.path.dirname(__file__), 'training_data.csv')
df = pd.read_csv(path)

train_dataset = df.sample(frac=0.8, random_state=0)
test_dataset = df.drop(train_dataset.index)

plot=sns.pairplot(train_dataset[[ 
                            # 'base_speed',
                            'distance_from_wall', 
                            'rotation_angle',
                            # 'distance_setpoint', 
                            'docking_time', 
                            'docking_distance']], 
                            diag_kind='kde'
                            )

plt.show()