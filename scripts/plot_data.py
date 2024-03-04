#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

results_dir = Path(get_package_share_directory('benchmark')) / 'results'
results_dict = {}
for file in results_dir.iterdir():
    ik_name = file.stem
    data = pd.read_csv(file)
    results_dict[ik_name] = data

plt.title("Success rates")
plt.bar(results_dict.keys(), [np.sum(data.found_ik == True) / len(data) for data in results_dict.values()])
plt.show()
plt.title("Solve times (ms)")
plt.boxplot([data.solve_time[data.found_ik == True] / 1000 for data in results_dict.values()], labels=results_dict.keys())
plt.show()

