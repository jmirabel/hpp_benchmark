#!/usr/bin/env python

from hpp.corbaserver import Benchmark
import argparse

parser = argparse.ArgumentParser\
    (description='Print results obtained with class hpp.corbaserver.Benchmark')
parser.add_argument ('input_file', type = str, nargs = 1,
                     help = 'input file containing the saved results')
parser.add_argument ('output_file', type = str, nargs = '?', default=None,
                     help = 'output file (standard output is used if omitted)')

args = parser.parse_args ()

b = Benchmark(None, None, None)
b.resumeFrom(args.input_file[0])
if args.output_file:
    with open (args.output_file, 'w') as f:
        f.write(str(b))
else:
    print b
