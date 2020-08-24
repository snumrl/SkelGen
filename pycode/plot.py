import numpy as np
import sys
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os.path
import time
from time import sleep

short_colors=['r','g','b']
colors=['red','green','blue']
labels=['std','rtgCalibrate','rtgInit']

# red_patch = mpatches.Patch(color='red', label='p uniform samples')
# green_patch = mpatches.Patch(color='green', label='normal samples')
# plt.legend(handles=[red_patch, green_patch])
#

# plt.ion()
# fig = plt.figure()
# plt.clf()
n = m = 0
X=[]
Y=[]
while not os.path.exists(sys.argv[1]):
    sleep(0.01)

fig,ax=plt.subplots()
if os.path.isfile(sys.argv[1]):
    # read file
    with open(sys.argv[1],'r') as f:
        fig.suptitle(f.readline())
        plt.xlabel(f.readline())
        plt.ylabel(f.readline())
        line = f.readline().split()
        n = int(line[0])
        m = int(line[1])
        Y = []
        for i in range(m):
            Y.append([])
        for i in range(n):
            t = f.readline().split()
            x = float(t[0])
            X.append(x)
            for j in range(m):
                y = float(t[j+1])
                Y[j].append(y)

print("Drawing")
print(sys.argv[1][38:-4]);

patches=[]
for j in range(m):
    ax.plot(X, Y[j], short_colors[j]+'-')
    patches.append( mpatches.Patch(color=colors[j], label=labels[j]) )
plt.legend(handles=patches)
plt.savefig('../fig'+sys.argv[1][38:-4]+'.png')