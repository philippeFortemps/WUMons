"""
CircuitPython Online IDE Plot example
Select/unselect "Use first column as x-axis" to see the difference.
"""
import math
from time import sleep

# print start indicator -----------------------------------------------
print('startplot:', 't*cos(t)', 't*sin(t)', 't*sin(-t)')
# `startplot:` is the start indicator
# column names separated by `,` in `print()`, no space in names.
# ---------------------------------------------------------------------

N = 100 # number of points to plot
for x in range(N):
    sleep(0.05) # for slower animation, and stable serial communication
    t = 2 * math.pi / (N - 1) * x

    # print a row of data (a dot on the graph) ------------------------
    print(t * math.cos(t), t * math.sin(t), t * math.sin(-t))
    # print plot data
    # column separated by `,` in `print()`
    # -----------------------------------------------------------------