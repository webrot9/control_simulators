#!/usr/bin/env python

import numpy as np
import python_simulable as ps

def test_pendulum():
    dt = 0.05
    T = 200
    noise_cov = np.identity(2)
    x0 = np.zeros(2) + 1e-3
    u = np.zeros(1)
    system = ps.Pendulum(x0)
    system['length'] = 1.5 # change the pendulum's length
    system.set_noise_cov(noise_cov)
    for t in xrange(T):
        system.step(dt, u)

    states = system.all_states()
    controls = system.all_controls()

    noise_cov_get = system.get_noise_cov()
    assert(np.all(noise_cov_get == noise_cov))
    assert(system['length'] == 1.5)

if __name__ == "__main__":
    test_pendulum()
    print('Finished test!')
