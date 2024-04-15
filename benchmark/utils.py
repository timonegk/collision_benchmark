import subprocess


def get_xacro(xacro_file):
    return subprocess.check_output(['xacro', xacro_file], universal_newlines=True)
