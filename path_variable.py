import numpy

def f(x):
    return numpy.exp(-1.0/x)
def g(x):
    return f(x) / (f(x) + f(1-x))
def h(t, T):
    return g(t/T)

def path_variable(t, T, path_var_max):
    if t > 0.0 and t < T:
        return path_var_max * h(t, T)
    if t <= 0.0:
        return 0.0
    return path_var_max

if __name__ == '__main__':
    from matplotlib import pyplot
    import numpy

    T = 5.0
    tiden = numpy.linspace(0,T,100)
    s = [path_variable(t, T, 2.0) for t in tiden]
    pyplot.plot(tiden, s)
    pyplot.show()
