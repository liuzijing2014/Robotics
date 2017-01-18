"""
Zijing Liu 445 - Programming Assignment 1 Task 3
"""

import numpy as np
import matplotlib.pyplot as plt

def mysum2(inputs):
    """
    Return the sum of integers in the list

    Arguments:
        inputs: a list of integers
    """
    return np.sum(inputs)

def plotcircle1():
    """
    Draw a circle with radius of 1 using matplotlib.plot()
    """
    x = np.cos(np.radians([i for i in range(360)]))
    y = np.sin(np.radians([i for i in range(360)]))

    plt.plot(x,y)
    plt.show()

def plotnorm1():
    """
    Draw a histogram of a normal distribution of 10000 data
    """
    rands = np.random.normal(0, 0.1, 10000)
    
    plt.hist(rands, 20)
    plt.show()

if __name__ == "__main__":
    #TEST
    #mysum2 tests
    print(mysum2([1,5,6]))
    print(mysum2([]))
    print(mysum2([10,50,60]))

    #plotcircle1 test
    plotcircle1()

    #plotnorm1 test
    plotnorm1()