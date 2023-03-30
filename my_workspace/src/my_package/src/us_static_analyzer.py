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
<<<<<<< Updated upstream
    print(times, values)
    plt.hist(values)

    filetered_values =medfilt(values, 100)
    plt.hist(filetered_values)
=======
    #plt.hist(values)
    
    mean = np.mean(values)
    sigma = np.std(values)
    
    x = np.linspace(values.min(), values.max(), 100)
    
    p1 = plt.plot(x, norm.pdf(x, mean, sigma))
    p2 = plt.hist(values, density = True)
    
    
    
    #plt.plot(x, norm.pdf(x))
    plt.show()

    
    print("La moyenne est de ", mean, "est l'écart type est de ", sigma)
    
    
    
    #print(times, values)
>>>>>>> Stashed changes

    # TODO: Analysis
    
