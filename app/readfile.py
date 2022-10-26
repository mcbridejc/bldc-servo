#%%
%matplotlib widget
import json
import matplotlib.pyplot as plt
import math
import numpy as np

with open('caldata.json', 'r') as f:
    data = json.loads(f.read())
data = np.array(data)[5:, :]
encoder_angle = data[:, 0]
motor_angle = data[:, 1]

#%%
first_move = np.min(np.where(motor_angle > 0.0))
# Average the second half of the stable at zero period 
# (first have is excluded because it may not have stabilizaed yet)
zero_angle = encoder_angle[int(first_move/2):first_move].mean()



# %%
theta = encoder_angle - zero_angle
theta[theta < 0.0] += 360
plt.figure()
plt.plot(motor_angle / 7, theta)
plt.plot([0, 360], [0, 360], 'g')

plt.figure()
err = theta - motor_angle / 7

# %%
