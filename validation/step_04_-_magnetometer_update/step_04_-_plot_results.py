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
    df_01 = pd.read_csv("step_04_-_scenario_10_-_before_tuning.txt")
    df_02 = pd.read_csv("step_04_-_scenario_10_-_after_tuning.txt")

    # Remove leading and trailing spaces in df headers
    df_01 = clean_df_headers(df_01)
    df_02 = clean_df_headers(df_02)

    # Set "time" column as DataFrame index
    df_01 = df_01.set_index("time")
    df_02 = df_02.set_index("time")

    # Interpolate NaN values
    #  (can happen if some sensors have different sampling rates)

    df_01 = df_01.interpolate()
    df_02 = df_02.interpolate()

    # Plot results
    fig = plt.figure()
    fig.suptitle("True & Measured & Estimated States \n (Global Frame)")

    # Estimated errors - Before tuning
    ax = plt.subplot(2,2,1)
    df = df_01[["quad.est.e.yaw","quad.est.s.yaw"]]
    ax = configure_ax(ax, df = df, ylabel = "Error [rad] \n Standard Deviation [rad]", ylim = (-0.5, 0.5), title = "Before Tuning", legend = True)
    ax.axhline(-0.1, color="r", label="Max Allowable Error")
    ax.axhline(0.1, color="r")
    ax.legend()

    # Estimated errors - After tuning
    ax = plt.subplot(2,2,2)
    df = df_02[["quad.est.e.yaw","quad.est.s.yaw"]]
    ax = configure_ax(ax, df = df, ylim = (-0.5, 0.5), title = "After Tuning", legend = True)
    ax.axhline(-0.1, color="r", label="Max Allowable Error")
    ax.axhline(0.1, color="r")
    ax.legend()

    # True / predicted / measured yaw - Before tuning
    ax = plt.subplot(2,2,3)
    df = df_01[["quad.magyaw", "quad.yaw","quad.est.yaw"]]
    ax = configure_ax(ax, df = df, ylabel = "Yaw [rad]", xlabel = "Time [s]", legend = True)

    # True / predicted / measured yaw - Before tuning
    ax = plt.subplot(2,2,4)
    df = df_02[["quad.magyaw", "quad.yaw","quad.est.yaw"]]
    ax = configure_ax(ax, df = df, xlabel = "Time [s]", legend = True)
    
    plt.show()

