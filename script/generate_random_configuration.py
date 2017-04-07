def generate_random_configuration(N, filename):
    from hpp.corbaserver import Client

    cl = Client ()
    qs = []

    for i in xrange(N):
        qs.append(cl.robot.shootRandomConfig())

    import pickle
    with open(filename, 'w') as f:
        pickle.dump (qs, f)

    print "Generated", N, "random configurations in", filename
