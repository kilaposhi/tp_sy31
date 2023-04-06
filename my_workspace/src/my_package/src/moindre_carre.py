import numpy as np
import matplotlib.pyplot as plt

def moindre_carre(x ,y):
    """Calcule les estimations de la droite des moindres carrés, 
    alpha et beta tel que y = alpha*x+beta
    Args:
        x (np.array): X-coordinates of points
        y (np.array): Y-coordinates of points
    Return: estim_slope, estim_intercept
    """
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    slope = np.sum((x-x_mean)*(y-y_mean)) / np.sum((x-x_mean)**2)
    intercept = y_mean - slope*x_mean

    return slope, intercept


def plot_moindre_carre(estim_slope, estim_intercept, x, y, x_label = "x", y_label = "y"):
    """Plot la droite des moidres carrées
    Args:
        x (np.array): X-coordinates of points
        y (np.array): Y-coordinates of points
    Return: estim_slope, estim_intercept
    """

    title = f"Droite des moindres carrés (y = a*x+b) : a ={estim_slope:.2f} ; b = {estim_intercept:.2f}"

    figure, axes = plt.subplots(1, 1)
    axes.scatter(x, y,c='purple') #scatter plot == graphe de dispersion
    # Display settings
    axes.set_title(title)
    axes.set_xlim((0, None))
    axes.set_ylim((0, None))
    axes.set_xlabel(x_label)
    axes.set_ylabel(y_label)
    x_max = axes.get_xlim()[1]
    axes.plot([0, x_max], [estim_intercept, estim_slope*x_max+estim_intercept], color='orange')
