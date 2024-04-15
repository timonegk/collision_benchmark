import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from benchmark.utils import result_to_df


def plot_results(results):
    # Transform to pandas dataframes
    for solver, data in results.items():
        results[solver] = result_to_df(data[0])

    plt.title("Success rates")
    plt.bar(results.keys(), [np.sum(data.reached == True) / len(data) for data in results.values()])
    plt.show()
    plt.title("Solve times (ms)")
    plt.boxplot([data.ik_time[data.reached == True] * 1000 for data in results.values()],
                labels=results.keys())
    plt.show()
