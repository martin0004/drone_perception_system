import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def clean_df_headers(df: pd.DataFrame):
    """Remove leading and trailing spaces in DataFrame headers."""

    headers = pd.Series(df.columns)

    new_headers = [header.strip() for header in headers]
    new_headers = pd.Series(new_headers)

    df.columns = new_headers

    return df
    

def get_df_cumop(df: pd.DataFrame, op: str = None):
    """"Perform cumulative operation of each row of DataFrame."""

    df_op = df.copy()
    for i in range(len(df)):
        # Upper part of DataFrame df, from line 0 to i.
        df_upper = df.iloc[:i+1,:] 
        # Cumulative operation on upper DataFrame.
        code = "df_upper." + str(op) + "(skipna = True)"
        df_op.iloc[i,:] = eval(code)   

    return df_op

def get_df_cummean(df: pd.DataFrame):
    """"Perform cumulative mean on each row of DataFrame."""
    return get_df_cumop(df, "mean")


def get_df_cumstd(df: pd.DataFrame):
    """"Perform cumulative standard deviation on each row of DataFrame."""
    return get_df_cumop(df, "std")


if __name__ == "__main__":


    # Load sensor data
    df = pd.read_csv("step_01_-_scenario_06_-_sensor_noise.txt")

    # Remove leading and trailing spaces in df headers
    df = clean_df_headers(df)

    # Set "time" column as DataFrame index
    df = df.set_index("time")

    # Plot sensor data (line plots, one subplot)
    df.interpolate().plot(kind = "line", xlabel = "Time", ylabel = "Sensor Data", title = "Sensor Data vs Time", subplots = True, layout = (5,3), sharey = True)
    plt.show()

    # Plot sensor data (histograms, many subplots)
    df.interpolate().plot(kind = "hist", subplots = True, layout = (5,3), bins=100)
    plt.show()

    # Get cumulative mean and standard deviation of each column
    df_cummean = get_df_cummean(df)
    df_cumstd = get_df_cumstd(df)

    # Plot cumulative mean and standard deviation
    df_cummean.plot(kind = "line", xlabel = "Time", ylabel = "Mean", title = "Sensor Data Mean vs Time")
    plt.show()
    df_cumstd.plot(kind = "line", xlabel = "Time", ylabel = "Standard Deviation", title = "Sensor Data Standard Deviation vs Time")
    plt.show()

    # Print mean and standard deviation of each column
    df_stats = df.describe().transpose()  # df.describe() ignores "nan" values by default
    df_stats.index.name = "sensor"        # add title to index column
    df_stats = df_stats[["mean", "std"]]  # keep only columns we care about
    df_stats = df_stats.round(2)          # round results to 2 decimals
    print(df_stats)                       # print results

