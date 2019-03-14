#!/usr/bin/env python
import argparse
import commands

parser = argparse.ArgumentParser\
    (description='Set packages to commit number specified by input file.')
parser.add_argument ('input_file', type = str, nargs = 1,
                     help = 'input file containing commit numbers')

args = parser.parse_args ()
commits = dict ()

with open (args.input_file [0], 'r') as f:
    for line in f:
        index = line.find (':')
        if index != -1:
            commit_id = line [index+2:].rstrip ('\n')
            pkg = line [:index]
            commits [pkg] = commit_id

for pkg, commit_id in commits.iteritems ():
    command = ['git --work-tree=./' + pkg + ' --git-dir=./' + pkg +
               '/.git checkout master',
               'git --work-tree=./' + pkg + ' --git-dir=./' + pkg +
               '/.git branch -D benchmark',
               'git --work-tree=./' + pkg + ' --git-dir=./' + pkg +
               '/.git checkout -b benchmark ' + commit_id,
               'if [ -f ' + pkg + '/.gitmodules ]; then cd ' + pkg +
               '; git submodule update; cd ..; fi']
    for c, i in zip (command, xrange (1000)):
        print (c)
        res = commands.getstatusoutput (c)
        # do not check result of git branch -D
        if i != 1 and res [0] != 0:
            print res [1]
            exit (-1);

exit (0)


