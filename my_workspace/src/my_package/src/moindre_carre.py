import numpy as np
import matplotlib.pyplot as plt

def plot_moindre_carre(slope, intercept, x, y):

    print("Droite des moindres carrés (y = a*x+b) : a =", slope, "; b =", intercept)

    _, ax = plt.subplots(1, 1)
    ax.scatter(x, y)
    ax.set_xlim((0, None))
    ax.set_ylim((0, None))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    x_max = ax.get_xlim()[1]
    ax.plot([0, x_max], [intercept, slope*x_max+intercept], color='red')

def moindre_carre(x,y):
    """Calcul les paramètres de la droite des moindres carrés, 
    alpha et beta
    Args:
        x (np.array): X-coordinates of points
        y (np.array): Y-coordinates of points
    """
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    slope = np.sum((x-x_mean)*(y-y_mean)) / np.sum((x-x_mean)**2)
    intercept = y_mean - slope*x_mean

    return slope, intercept


