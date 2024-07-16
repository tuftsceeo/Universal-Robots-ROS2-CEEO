import numpy as np

def makeCircle(r,steps):
    cords = []
    for t in range(steps):
        x = r * np.cos(t*2*np.pi/(steps))
        y = r * np.sin(t*2 * np.pi / (steps))
        cords.append([0.2+x,.2+y,0.3,0,0,1])
    return cords

print(makeCircle(.1,50))
