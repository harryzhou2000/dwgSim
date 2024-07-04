import numpy as np
import json
import matplotlib.pyplot as plt
import argparse


parser = argparse.ArgumentParser(prog="drawDwgSimJson")

parser.add_argument("input")


args = parser.parse_args()


fin = open(args.input, "r")
doc = json.load(fin)
fin.close()


print(json.dumps(doc))
