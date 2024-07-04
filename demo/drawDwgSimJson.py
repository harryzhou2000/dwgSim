import matplotlib.patches
import numpy as np
import json
import matplotlib
import matplotlib.pyplot as plt
import argparse


parser = argparse.ArgumentParser(prog="drawDwgSimJson")

parser.add_argument("input")


args = parser.parse_args()


fin = open(args.input, "r")
doc = json.load(fin)
fin.close()

fig = plt.figure("main")
ax = plt.axes()


for ent in doc["modelSpaceEntities"]:
    if(ent["type"] == "LINE"):
        ax.plot([ent["start"][0], ent["end"][0]], [ent["start"][1], ent["end"][1]])
    if ent["type"] == "LWPOLYLINE" or ent["type"] == "POLYLINE_2D":
        pointsData = np.array(ent["vertex"])
        pointsDataT = pointsData.transpose()
        ax.plot(pointsDataT[0], pointsDataT[1])
    if(ent["type"] == "ARC"):
        arc = matplotlib.patches.Arc(
            ent["center"],
            ent["radius"] * 2,
            ent["radius"] * 2,
            theta1=ent["start_angle"] / np.pi * 180,
            theta2=ent["end_angle"] / np.pi * 180,
        )
        ax.add_patch(arc)
    if(ent["type"] == "CIRCLE"):
        circ = matplotlib.patches.Circle(ent["center"], ent["radius"])
        circ.set_fill(False)
        ax.add_patch(circ)


ax.axis("equal")

plt.show()
