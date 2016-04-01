#!/usr/bin/env python

import numpy as np
import python_simulable as ps
from timer import Timer
import sys
sys.path.append('../../dad_mpc/src/')

from simulators.pendulum import Pendulum
from simulators.cartpole import CartPole

def test_pendulum():
    dt = 0.05
    num_steps = 200
    noise = 0
    x0 = np.zeros(2) + 1e-3
    u = np.zeros(1)
    pcpp = ps.Pendulum(x0)
    with Timer("cpp pendulum"):
        [pcpp.step(dt, u, noise) for _ in xrange(num_steps)]

    ppy = Pendulum(x0)
    with Timer("py pendulum"):
        [ppy.step(u=u, dt=dt,sigma=(noise, noise)) for _ in xrange(num_steps)]
    print 'Time cpp: {:.12g}'.format(pcpp.time())
    print 'Time py: {:.12g}'.format(ppy.t)
    scpp = pcpp.all_states()
    spy = ppy.get_states()
    err = scpp - spy; err_sq = err * err;
    mean_err  = np.mean(np.sqrt(np.sum(err_sq, axis=1)))
    print 'Mean Diff in States: {:.4g}'.format(mean_err)
    #print scpp
    #print spy

def test_cartpole():
    x0 = np.zeros(4) + 1e-3
    u = np.zeros(1)
    noise = 0
    pcpp = ps.Cartpole(x0)
    dt = 0.01
    num_steps = 200
    noise = 0
    with Timer("cpp cartpole"):
        [pcpp.step(dt, u, noise) for _ in xrange(num_steps)]

    ppy = CartPole(x0)
    with Timer("py cartpole"):
        [ppy.step(u=u) for _ in xrange(num_steps)]
    print 'Time cpp: {:.12g}'.format(pcpp.time())
    print 'Time py: {:.12g}'.format(ppy.t)
    scpp = pcpp.all_states()
    spy = ppy.get_states()
    err = scpp - spy; err_sq = err * err;
    mean_err  = np.mean(np.sqrt(np.sum(err_sq, axis=1)))
    print 'Mean Diff in States: {:.4g}'.format(mean_err)
    #print scpp
    #print spy

if __name__ == "__main__":
    test_cartpole()
