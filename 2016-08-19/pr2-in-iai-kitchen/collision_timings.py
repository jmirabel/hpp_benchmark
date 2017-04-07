from hpp.corbaserver.pr2 import Robot
from hpp.gepetto import PathPlayer
import os.path, pickle, sys
from math import pi

# progress bar

# Print iterations progress
def printProgress (iteration, total, prefix = '', suffix = '', decimals = 1, barLength = 100):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        barLength   - Optional  : character length of bar (Int)
    """
    formatStr       = "{0:." + str(decimals) + "f}"
    percents        = formatStr.format(100 * (iteration / float(total)))
    filledLength    = int(round(barLength * iteration / float(total)))
    bar             = '+' * filledLength + '-' * (barLength - filledLength)
    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),
    if iteration == total:
        sys.stdout.write('\n')
    sys.stdout.flush()

robot = Robot ('pr2')
# bounds = robot.getJointBounds ("root_joint")
# bounds[0:4] = [-4, -3, -5, -3]
bounds = [-4, -3, -5, -3, -pi, pi]
robot.setJointBounds ("root_joint", bounds)


from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")


path_validation_type = "Dichotomy"
path_validation_penetration = 0
# path_validation_type = "Discretized"
# path_validation_penetration = 0.05
# path_validation_type = "Progressive"
# path_validation_penetration = 0.05

configurations_filename = "random_configuration.pickle"
expected_results_filename = "expected_results." + path_validation_type + '.' + str(path_validation_penetration) + ".pickle"

ps.selectPathValidation (path_validation_type, path_validation_penetration)

if not os.path.isfile(configurations_filename):
    # from ../../script/generate_random_configuration import generate_random_configuration
    execfile("../../script/generate_random_configuration.py")
    generate_random_configuration(1000, configurations_filename)

with open(configurations_filename, 'r') as f:
    qs = pickle.load(f)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0

N = len(qs) / 2

results = dict()
printProgress(0, N-1, prefix = 'Progress:', suffix = 'Complete', barLength = 50)
for i in range (N):
    t1 = dt.datetime.now ()
    res, pid, msg = ps.directPath(qs[i], qs[N+i], True)
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    results[(i, N+i)] = {
            "valid" : res,
            "time"  : t2 - t1,
            "msg"   : msg,
            }
    ps.client.problem.erasePath(pid)
    printProgress(i, N-1, prefix = 'Progress:', suffix = 'Complete', barLength = 50)

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))

if os.path.isfile(expected_results_filename):
    with open(expected_results_filename, 'r') as f:
        expected_results = pickle.load(f)

    for key, value in expected_results.iteritems():
        if results.has_key(key):
            if not results[key]["valid"] == value["valid"]:
                print "Path between ", key, " did not give the same results"
