import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_results(results):
    # Transform to pandas dataframes
    for solver, data in results.items():
        df = pd.DataFrame()
        df['reached'] = [d.reached for d in data]
        df['ik_time'] = [d.ik_time for d in data]
        results[solver] = df

    plt.title("Success rates")
    plt.bar(results.keys(), [np.sum(data.reached == True) / len(data) for data in results.values()])
    plt.show()
    plt.title("Solve times (ms)")
    plt.boxplot([data.ik_time[data.reached == True] * 1000 for data in results.values()],
                labels=results.keys())
    plt.show()
