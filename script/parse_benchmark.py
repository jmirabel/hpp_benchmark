#!/usr/bin/env python
from __future__ import print_function
import sys
from os.path import isdir, join, isfile

def parseDirectories(directory, bench):
    def valid (dir):
        base = join(directory,dir)
        if not isdir(base): return False
        bdir = join(base,bench)
        if not isdir(bdir): return False
        return isfile(join(bdir,"benchmark"))

    from os import listdir
    keys = []
    values = dict()

    for dir in filter (valid, listdir(directory)):
        file = join (directory,dir,bench,"benchmark")
        if not isfile(file):
            print(file + " does not exist", file=sys.stderr)
            continue
        values[dir] = dict()
        with open (file, 'r') as f:
            for line in f.readlines():
                if line.startswith("Average "):
                    k, v = line[len("Average "):].split(':',1)
                    if k not in keys: keys.append(k)
                    values[dir][k] = v
    return keys, values

def generateCSV(keys, values, csvfile, sep = ';', nan = 'nan'):
    labels = sorted(values.keys())
    print("Dates" + sep + sep.join(keys), file=csvfile)
    for d in labels:
        line = d
        val = values[d]
        # print(val)
        for k in keys:
            if k in val.keys():
                line += sep + str(float(val[k]))
            else:
                line += sep + nan
        print(line, file=csvfile)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise RuntimeError("Expected 1 argument")

    directory = sys.argv[1]
    bench = sys.argv[2]

    keys, values = parseDirectories (directory, bench)
    generateCSV (keys, values, sys.stdout)
