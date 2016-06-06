#!/usr/bin/env python
import argparse
import commands

parser = argparse.ArgumentParser\
    (description='push to repository origin given branch to commit number specified by input file.')
parser.add_argument ('input_file', type = str, nargs = 1,
                     help = 'input file containing commit numbers')
parser.add_argument ('-b', dest='branch', type = str, nargs = 1,
                     help = 'branch into which commit will be pushed')

args = parser.parse_args ()
commits = dict ()
branch = args.branch [0]

with open (args.input_file [0], 'r') as f:
    for line in f:
        index = line.find (':')
        if index != -1:
            commit_id = line [index+2:].rstrip ('\n')
            pkg = line [:index]
            commits [pkg] = commit_id

for pkg, commit_id in commits.iteritems ():
    command = ['git --work-tree=./' + pkg + ' --git-dir=./' + pkg +
               '/.git push origin ' + commit_id + ':' + branch,]
    for c in command:
        print (c)
        res = commands.getstatusoutput (c)
        if res [0] != 0:
            print res [1]
            exit (-1);

exit (0)


