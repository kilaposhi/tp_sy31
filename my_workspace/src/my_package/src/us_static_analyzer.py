#! /usr/bin/env python3

from rosbag import Bag
from argparse import ArgumentParser

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy.stats import norm

def load_bag():
    # Retrieve bag path
    argparser = ArgumentParser()
    argparser.add_argument("PATH", help="Path to the rosbag to study.")
    args = argparser.parse_args()

    time_values = []
    with Bag(args.PATH) as bag:
        for _, msg, t in bag.read_messages(["/ultrasound"]):
            time_values.append([t.to_sec(), msg.data])

    bag = np.array(time_values)
    bag[:, 0] -= bag[0, 0]
    return bag[:, 0] - bag[0, 0], bag[:, 1]

if __name__=="__main__":
    times, values = load_bag()
    print(times, values)

    # TODO: Analysis
    
