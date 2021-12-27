import numpy as np 
import io
import os

data = np.empty((0,3), int)
with io.open("t1.txt", mode="r", encoding="utf-8") as f:
    for line in f:
        a = np.array([np.double(b) for b in line.split()])
        if(a.size == 0):
            continue
        data = np.append(data,np.array([a]),axis=0)
print(data)
np.savetxt("t2.csv", data, delimiter=",")
with open('t2.csv', 'r') as f1, open('t3.csv', 'w+') as f2:
    for line in f1:
        f2.writelines(line.replace(os.linesep, "") + ',' + os.linesep)

