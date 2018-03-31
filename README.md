##  *Kidnapped Vehicle Project*

The goals / steps of this project are the following:
* Modify the code to estimate the vehicle position using particle filter

---
### Writeup / README
In this project simulated data is used to estimate Vehicle position using Particle Filter. Source code provided by "UDACITY CarND Kidnapped Vehicle Project" was used as base for this project. 

### Modifications
particle_filter.cpp was modified to implement Particle Filter. Init, Prediction , updateWeights , resample functions were modified to proces data and to implement Particle Filter. Modified code can be found [here] ("./Source")

### Code Flow
Following is brief description of the flow of the code.
- Initialize the Particles with initial position , Initial positions are based on GPS sensor output. 
- Perdict the position of the Particles by using the new data inputs.
- Convert the observations to map coordinates.
- Update Weight of each particle using Gaussian equation.
- Discard the particles with low weight and fill the particles that are near to the once with higher weight.

