import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
from scipy.stats import norm

def moindre_carre(x, y):

    #fontion qui calcule la droite des moindres carrés
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    a_hat = np.sum((x-x_mean)*(y-y_mean)) / np.sum((x-x_mean)**2)
    b_hat = y_mean - a_hat*x_mean
    
    return a_hat, b_hat
    """
    
    print("Droite des moindres carrés (y = a*x+b) : a =", a_hat, "; b =", b_hat)
    
    ax = plt.subplot(1, 1)
    ax.scatter(x, y)
    ax.set_xlim((0, None))
    ax.set_ylim((0, None))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    x_max = ax.get_xlim()[1]
    ax.plot([0, x_max], [b_hat, a_hat*x_max+b_hat], color='red')
    plt.show()"""