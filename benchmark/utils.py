import subprocess
import pandas as pd


def get_xacro(xacro_file):
    return subprocess.check_output(['xacro', xacro_file], universal_newlines=True)


def result_to_df(result):
    df = pd.DataFrame()
    df['reached'] = [d.reached for d in result]
    df['ik_time'] = [d.ik_time for d in result]
    return df