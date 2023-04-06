#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 30 16:03:19 2023

@author: ubuntu
"""

#! /usr/bin/env python3

from us_static_analyzer import load_bag
from moindre_carre import moindre_carre, plot_moindre_carre

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy.stats import norm




if __name__=="__main__":
    
    

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
    
    measured_times, values = load_bag() # Retrieve measured times and values 

    # Keeping only each steps measures     
    step_times = [measured_times[(measured_times>tbeg) & (measured_times<tend)] for dtheor, tbeg, tend in steps]
    step_values = [values[(measured_times>tbeg) & (measured_times<tend)] for dtheor, tbeg, tend in steps]
    # for (theoric_length, time_beg, time_end) in steps :   
    # is_step_times = [(measured_times>time_beg)& (measured_times<time_end) for theoric_length, time_beg, time_end in steps] #[False  True  True ... False False False] Bool array
    # steps_times2=[measured_times[is_step_times] for theoric_length, time_beg, time_end in steps]
    # steps_values2=[values[is_step_times] for theoric_length, time_beg, time_end in steps]
    

        # steps_times2= np.append(steps_times2,measured_times[is_step_times]) 
        # steps_values2= np.append(steps_values2, measured_times[is_step_times])

    # print(step_times== steps_times2)
    # print(steps_values2==step_values)
    # print(steps_values2)

    step_distances = [np.full(len(values), d)
                        for (d, _, _), values in zip(steps, step_values)]
    x = np.hstack(step_distances)
    

    #plt.show()
    
    p2 = plt.plot(x)
    #p3 = plt.plot(step_values)
    
    plt.show()
    y = np.hstack(step_values)
    

    a_hat, b_hat = moindre_carre(x,y)

    plot_moindre_carre(a_hat, b_hat, x, y)

    plt.show()
    



