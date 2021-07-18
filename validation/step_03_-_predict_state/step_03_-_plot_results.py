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
    df_data = pd.read_csv("step_03_-_scenario_08_-_after_tuning.txt")

    # Remove leading and trailing spaces in df headers
    df_data = clean_df_headers(df_data)

    # Set "time" column as DataFrame index
    df_data = df_data.set_index("time")

    # Plot results
    fig = plt.figure()
    fig.suptitle("True & Predicted States \n (Global Frame)")

    # X-Position and X-Speed
    ax = plt.subplot(3,1,1)
    df = df_data[["quad.pos.x", "quad.est.x", "quad.vel.x", "quad.est.vx"]]
    ax = configure_ax(ax, df = df, ylabel = "X-Positions [m] \n X-Velocities [m/s]", title = "After Tuning", legend = True)

    # Y-Position and Y-Speed
    ax = plt.subplot(3,1,2)
    df = df_data[["quad.pos.y", "quad.est.y", "quad.vel.y", "quad.est.vy"]]
    ax = configure_ax(ax, df = df, ylabel = "Y-Positions [m] \n Y-Velocities [m/s]", legend = True)

    # Z-Position and Z-Speed
    ax = plt.subplot(3,1,3)
    df = df_data[["quad.pos.z", "quad.est.z", "quad.vel.z", "quad.est.vz"]]
    ax = configure_ax(ax, df = df, xlabel = "Time [s]", ylabel = "Z-Positions [m] \n Z-Velocities [m/s]", legend = True)
    
    plt.show()

