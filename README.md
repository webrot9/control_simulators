# Control Simulators
C++ control simulators featuring a Python interface and an easily extendable framework for the implementation of new systems

### Requirements
- Eigen3
- PythonLibs
- Boost python, random
- OpenMP

```
sudo apt install python-dev libboost-dev libboost-python-dev libboost-random-dev libboost-system-dev
```

## Quick Build Instructions
Go to the directory where you want to clone this repositorty and run:
```
git clone https://github.com/webrot9/control_simulators
cd control_simulators
mkdir build
cd build
cmake ..
make -j8
```
If your machine is RAM limited, you should make with a lower `-j` value as there is template compilation due to Boost iin creating the Python bindings.

## Example usage
One can use this library with the C++ API or the Python API. 

### Python API
Here's an example usage of the Python API. We first go to the `build/lib` directory
```python
import python_simulable as ps
import numpy as np
x0 = np.array((np.pi, 0)) # initial state
system = ps.Pendulum(x0)
dt = 0.05 # simulate at 20 Hz
T = 100 # simulate for 100 timesteps
for t in xrange(T):
    u_t = np.random.rand(1) # Pendulum takes a 1-d control input
    x_t = system.step(dt, u_t)
X = system.all_states()
U = system.all_controls()
```
We can have additive Gaussian noise with each call to `.step` by setting the mean or covariance:
```python
system.set_noise_cov(1e-3*np.identity(2)) # We can set the mean with set_noise_mean()
``` 
You can get and set parameters of the system by using `[]` to access them
```python
system.params() # gives us a list of the parameter names (e.g. ['damping', 'length'] )
damping_coeff = system['damping']
system['damping'] = 0.1
```

#### Setting PYTHONPATH
To use this elsewhere, you likely will have to set the `PYTHONPATH` variable in your shell environment. 

On a bash shell, this will be similar to: 
<br>&emsp;`export PYTHONPATH+=:[path_to_control_simulators]/build/lib`
<br>If you are already in the `build/lib` directory, this can be accomplished by:
<br>&emsp;``export PYTHONPATH+=:`pwd````


## Notes
This project uses external dependencies [Boost.NumPy](https://github.com/personalrobotics/Boost.NumPy) 
and [Boost.NumPy_Eigen](https://github.com/personalrobotics/Boost.NumPy_Eigen).
Both of these will be automatically downloaded when calling `make`.


## Misc.
This has been tested to be working on Ubuntu 14.04. 
It should theoretically be possible to make it work on OS X, but we have been unable to get it to work yet.

File any issues on Github (https://github.com/webrot9/control_simulators/issues). 
You are welcome to also submit pull requests with additional simulator models or other upgrades. 
Note that your models must support licensing that will inherit whatever licensing we (Roberto Capobianco and Arun Venkatraman) choose (and may modify in the future) for this repository.

