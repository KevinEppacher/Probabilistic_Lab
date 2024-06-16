# Debbuging List:

## Problem 1:
### Description:
The beam_range_finder_model from the Sensor Model only returns 0 ( or at least small numbers )

### Solution: 
Increase z_rand Paramter to over 1 --> has to be verified

## Problem 2:
### Description:
The Particles are on the wrong location and do not move

### Checked Implementations:
1. The Particles weights are normalized

### Solved issus:
1. In the resampling process, are the weights also normalized and not only the particles
2. The particle weights are updaten with new weights ( for each particle the weight / particles quantity )

### Other issues:
1. Program is slow, due to visualization in all the nested loops --> visualize all particles, rays at once on each particle
2. For debugging purposes, print the sum of the errors of each measurement


### Solution: 
Yet no solution

## Problem 2:
### Description:
The particle_filter program takes to long

### Checked Implementations:
1.  Using isPoseInFreeCell does not affect 
With isPoseInFreeCell: 2.4 s
Without isPoseInFreeCell: 2.3 s

2. Sensor_Model.cpp
With Ray visualization: 2.4 s
Without Ray visualization: 2.6 s

3. Particel_Filter.cpp
Updating map in every loop with this-> map = map : 2.6 s
Not Updating map in every loop with this-> map = map : 1.8 s

4. 
With Docker: ...
Without Docker: ...


### Solved issus:


### Other issues:


### Solution: 
