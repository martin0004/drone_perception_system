import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from typing import Tuple


def clean_df_headers(df: pd.DataFrame) -> pd.DataFrame:
    """Remove leading and trailing spaces in DataFrame headers."""

    headers = pd.Series(df.columns)

    new_headers = [header.strip() for header in headers]
    new_headers = pd.Series(new_headers)

    df.columns = new_headers

    return df

def configure_ax(ax: plt.axes,
                 df: pd.DataFrame = None,
                 xlabel: str = None,
                 ylabel: Tuple[int,int] = None,
                 ylim: str = None,
                 title: str = None,
                 legend: bool = False
                 ) -> plt.axes:
    """Configure Matplotlib axe."""

    if df is not None:
        x = df.index
        for h in df.columns:
            y = df[h]
            ax.plot(x, y,label=h)

    if xlabel is not None:
        ax.set_xlabel(xlabel)

    if ylabel is not None:
        ax.set_ylabel(ylabel)  

    if ylim is not None:
        ax.set_ylim(ylim)     

    if title is not None:
        ax.set_title(title)

    if legend is not None:
        ax.legend()    

    return ax


if __name__ == "__main__":


    # Load sensor data
    df_01 = pd.read_csv("step_02_-_scenario_07_-_body_rates.txt")
    df_02 = pd.read_csv("step_02_-_scenario_07_-_euler_forward.txt")

    # Remove leading and trailing spaces in df headers
    df_01 = clean_df_headers(df_01)
    df_02 = clean_df_headers(df_02)

    # Set "time" column as DataFrame index
    df_01 = df_01.set_index("time")
    df_02 = df_02.set_index("time")

    # Plot results
    fig = plt.figure()
    fig.suptitle("True & Estimated States \n (Global Frame)")

    # Method 1 - Estimated errors
    ax = plt.subplot(4,2,1)
    df = df_01[["quad.est.e.roll","quad.est.e.pitch","quad.est.e.yaw"]]
    ax = configure_ax(ax, df = df, ylabel = "Error [rad]", ylim = (-0.2, 0.5), title = "Before Tuning \n (Body Rates Approximation)", legend = True)
    ax.axhline(-0.1, color="r", label="Max Allowable Error")
    ax.axhline(0.1, color="r")
    ax.legend()

    # Method 2 - Estimated errors
    ax = plt.subplot(4,2,2)
    df = df_02[["quad.est.e.roll","quad.est.e.pitch","quad.est.e.yaw"]]
    ax = configure_ax(ax, df = df, ylim = (-0.2, 0.5), title = "After Tuning \n (Euler Forward Method)", legend = True)
    ax.axhline(-0.1, color="r", label="Max Allowable Error")
    ax.axhline(0.1, color="r")
    ax.legend()

    # Method 1 - Estimated / true roll
    ax = plt.subplot(4,2,3)
    df = df_01[["quad.roll","quad.est.roll"]]
    ax = configure_ax(ax, df = df, ylabel = "Roll [rad]", legend = True)

    # Method 2 - Estimated / true roll
    ax = plt.subplot(4,2,4)
    df = df_02[["quad.roll","quad.est.roll"]]
    ax = configure_ax(ax, df = df, legend = True)

    # Method 1 - Estimated / true pitch
    ax = plt.subplot(4,2,5)
    df = df_01[["quad.pitch","quad.est.pitch"]]
    ax = configure_ax(ax, df = df, ylabel = "Pitch [rad]", legend = True)

    # Method 2 - Estimated / true pitch
    ax = plt.subplot(4,2,6)
    df = df_02[["quad.pitch","quad.est.pitch"]]
    ax = configure_ax(ax, df = df, legend = True)

    # Method 1 - Estimated / true yaw
    ax = plt.subplot(4,2,7)
    df = df_01[["quad.yaw","quad.est.yaw"]]
    ax = configure_ax(ax, df = df, ylabel = "Yaw [rad]", xlabel = "Time [s]", legend = True)

    # Method 2 - Estimated / true yaw
    ax = plt.subplot(4,2,8)
    df = df_02[["quad.yaw","quad.est.yaw"]]
    ax = configure_ax(ax, df = df, xlabel = "Time [s]", legend = True)
    
    plt.show()

