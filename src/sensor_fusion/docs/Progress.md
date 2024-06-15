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


### Solution: 
