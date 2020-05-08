#!/usr/bin/env python
from __future__ import print_function
import sys, csv, numpy as np
from os.path import basename

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

def readCSV (file):
    labels = []
    rows   = []
    with open(file, 'r') as f:
        csvReader = csv.reader(f, delimiter=';')
        legends = next(csvReader)
        for row in csvReader:
            labels.append(row[0])
            rows  .append(row[1:])

    rows = np.array(rows)
    return legends, labels, rows

def stringsToDates (strs):
    from datetime import datetime
    dts = [ None ] * len(strs)
    for i,s in enumerate(strs):
        if s in [ "today", "next" ]:
            dts[i] = datetime.now()
            continue
        if s[-1] in 'ab':
            ss = s[:-1]
            h = 12 if s[-1] == 'b' else 0
        else:
            ss = s
            h = 0
        ymd = [int(x) for x in ss.split('-')]
        dts[i] = datetime(ymd[0], ymd[1], ymd[2], hour=h)
    return dts

def generatePlot (legends, labels, rows, title, dstfile):
    n = len(legends) - 1

    rcfigsize = plt.rcParams["figure.figsize"]
    nrows = (n+1>>1)
    figsize = (rcfigsize[0], nrows * rcfigsize[1])
    fig, axes = plt.subplots(nrows = nrows,ncols=1,sharex=True,squeeze=False,figsize=figsize)
    # TODO check that the labels are sorted.
    # otherwise sort xticks accordingly
    xticks = range(len(labels))

    axes[0,0].set_title (title)

    for ax in axes[:,0]:
        ax.set_xlabel(legends[0])
        ax.set_xticks(xticks)
        ax.set_xticklabels(labels)
        plt.setp( ax.xaxis.get_majorticklabels(), rotation=70 )

    for k in range(n):
        if k % 2 == 0:
            ax = axes[ (k>>1),0 ]
            color = 'b'
            style = 'bD-.'
        else:
            ax = axes[ (k>>1),0 ].twinx()
            color = 'r'
            style = 'ro--'

        ax.plot(xticks, list(map (float, rows[:,k])), style)

        # Make the y-axis label, ticks and tick labels match the line color.
        ax.set_ylabel(legends[k+1], color=color)
        ax.tick_params('y', colors=color)

    fig.tight_layout()
    plt.savefig (dstfile)
    print("Saved plot to " + dstfile)

def generateErrorBars (legends, labels, rows, title, dstfile):
    n = len(legends) - 1

    rcfigsize = plt.rcParams["figure.figsize"]
    nrows = (n+1>>1)
    figsize = (rcfigsize[0], nrows * rcfigsize[1])
    fig, axes = plt.subplots(nrows = nrows,ncols=1,sharex=True,squeeze=False,figsize=figsize)
    # TODO check that the labels are sorted.
    # otherwise sort xticks accordingly
    xticks = range(len(labels))

    axes[0,0].set_title (title)

    for ax in axes[:,0]:
        ax.set_xlabel(legends[0])
        ax.set_xticks(xticks)
        ax.set_xticklabels(labels)
        plt.setp( ax.xaxis.get_majorticklabels(), rotation=70 )

    for k in range(n):
        if k % 2 == 0:
            ax = axes[ (k>>1),0 ]
            color = 'b'
            ls = '-.'
            ms = 'D'
        else:
            ax = axes[ (k>>1),0 ].twinx()
            color = 'r'
            ls = '--'
            ms = 'o'

        mean = [ np.mean(r[:,k].astype(float)) for r in rows ]
        stddev = [ np.sqrt(np.var(r[:,k].astype(float))) for r in rows ]
        ax.errorbar(xticks, mean, yerr=stddev, ls=ls, marker=ms, color=color)

        # Make the y-axis label, ticks and tick labels match the line color.
        ax.set_ylabel(legends[k+1], color=color)
        ax.tick_params('y', colors=color)

    fig.tight_layout()
    plt.savefig (dstfile)
    print("Saved plot to " + dstfile)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise RuntimeError("Expected 1 argument")

    file = sys.argv[1]
    legends, labels, rows = readCSV(file)

    if len(rows) == 0:
        print("No data in file " + file)
        sys.exit(0)

    generatePlot (legends, labels, rows, file, file + ".svg")
