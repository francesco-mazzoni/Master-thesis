# Optimization procedure

In this folder one can find the source codes used for parameters identification via custom optimization procedure using "fmincon".
There are two repositories: one is for the optimization via clothoids usage, where clothoids are used for points fitting and sampling in order to get the matching pairs for the parameters identification; the other one contains the files used to get the first preliminary results, it does not use any clothoid for borer points but it mainly uses sample distances. The altertative approach was adopted due to computational burden and errors in clothoids usage.

Notice that in this repository there are also other elements. "csv_gen.py" is a Python script used to obtain some telemetry data of interest. The file runs with the cache directory Ch_data and uses the FastF1 python library (https://docs.fastf1.dev). Use this pyhton script to get the position guess and the lap timing of Leclerc, Q3 session, Monza GP 2022.
