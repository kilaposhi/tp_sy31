#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 30 16:03:19 2023

@author: ubuntu
"""

#! /usr/bin/env python3

from us_static_analyzer import load_bag

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy.stats import norm



if __name__=="__main__":
    
    
    times, values = load_bag()

    # Split in steps based on begin/end times
    steps = [
        #( cm,  tbeg,  tend)
        (0.05,   0.0,  17.0),
        (0.10,  20.5,  32.0),
        (0.15,  34.0,  49.5),
        (0.20,  51.8,  64.0),
        (0.25,  66.0,  77.5),
        (0.30,  79.3,  95.3),
        (0.35,  96.5, 110.0),
        (0.40, 114.0, 126.8),
        (0.45, 128.7, 143.1),
        (0.50, 145.0, 157.0),
        (0.55, 157.7, 174.2),
        (0.60, 175.0, 188.0),
        (0.65, 189.8, 203.0),
        (0.70, 204.2, 217.4),
        (0.75, 218.5, 231.8),
        (0.80, 232.8, 246.0),
        (0.85, 246.7, 259.6),
        (0.90, 262.2, 275.0),
        (0.95, 276.3, 290.0),
        (1.00, 290.5, 305.0),
        (1.05, 306.0, 319.0),
        (1.10, 320.0, 329.0),
    ]

    step_times = [times[(times>tbeg) & (times<tend)] for dtheor, tbeg, tend in steps]
    step_values = [values[(times>tbeg) & (times<tend)] for dtheor, tbeg, tend in steps]
    step_distances = [np.full(len(values), d)
                        for (d, _, _), values in zip(steps, step_values)]
    x = np.hstack(step_distances)
    


    # TODO: Analysis
    #p1 = plt.plot(values)
    
    
    
    #plt.show()
    
    p2 = plt.plot(x)
    #p3 = plt.plot(step_values)
    
    plt.show()
    y = np.hstack(step_values)
    

    x_mean = np.mean(x)
    y_mean = np.mean(y)
    a_hat = np.sum((x-x_mean)*(y-y_mean)) / np.sum((x-x_mean)**2)
    b_hat = y_mean - a_hat*x_mean
    print("Droite des moindres carrés (y = a*x+b) : a =", a_hat, "; b =", b_hat)
    
    _, ax = plt.subplots(1, 1)
    ax.scatter(x, y)
    ax.set_xlim((0, None))
    ax.set_ylim((0, None))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    x_max = ax.get_xlim()[1]
    ax.plot([0, x_max], [b_hat, a_hat*x_max+b_hat], color='red')
    plt.show()
    



