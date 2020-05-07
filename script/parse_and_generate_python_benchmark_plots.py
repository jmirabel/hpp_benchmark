#!/usr/bin/env python
import os, sys
from parse_benchmark import parseDirectories, generateCSV
from generate_benchmark_plot import readCSV, generatePlot

# To generate this list:
# find 20*/* -type d | sed 's/^[^\/]*\///' | sort | uniq

if len(sys.argv) > 1:
    benchmarks = sys.argv[1:]
else:
    benchmarks=[
            "baxter-manipulation-boxes",
            "baxter-manipulation-boxes-easy",
            "baxter-two-arms-one-box",
            "baxter-two-arms-three-boxes",
            "baxter-two-arms-two-boxes",
            "construction-set",
            "hrp2-on-the-ground",
            "kawada",
            "path_validation_manipulation",
            "pr2-in-iai-kitchen",
            "pr2-ini-iai-kitchen",
            "pr2-manipulation-kitchen",
            "pr2-manipulation-two-hand",
            "romeo-placard",
            "ur3-spheres",
            "ur5_path_projection",
            "ur5-spline-optimization",
            ]

root_dir="."
output_dir="./results"
script_dir="./script"

if not os.path.isdir(output_dir): os.mkdir(output_dir)

for bench in benchmarks:
    csv = os.path.join (output_dir, bench + ".csv")

    keys, values = parseDirectories (root_dir, bench)

    if len(keys) == 0:
        print("No data for bench " + bench)
        continue

    with open(csv, 'w') as csvfile:
        generateCSV (keys, values, csvfile)

    legends, labels, rows = readCSV(csv)
    if len(rows) == 0:
        print("This should not happen: No data in file " + csv)
        continue
    svg = os.path.join (output_dir, bench + ".svg")
    generatePlot (legends, labels, rows, bench, svg)
