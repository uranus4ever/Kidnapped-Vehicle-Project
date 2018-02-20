# Kidnapped Vehicle with Particle Filter

## Overview

My robot has been kidnapped and transported to a new location. Luckily it has a map of this location, a noisy GPS estimate of its initial location, and lots of noisy sensor and control data.

In this project, particle filter will be given a map and some initial localization information (GPS).  

### Demo: Vehicle Localization with Particle Filter

![demo gif][gif]

**Note:**

 - Inputs:
    - one map contrains landmarks.
    - one initial location (GPS) in the very beginning with big uncertainty.
    - noisy landmark observation in each timestamp while vehicle is moving.
 - Outputs:
    - The blue circle is the real-time estimation of the vehicle's location and heading orientation from the particle filter.
 - Ground truth:
    - The blue car is the ground truth of the vehicle, including position and heading orientation.

## Code & Files

### 1. Dependencies & environment

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* uWebSockets: used for communication between the main code and the simulator.

### 2. How to run the code
1. Clone this repo.
2. Clean the project: $./clean.sh
3. Build the project: $./build.sh
4. Run the project: $./run.sh
5. Start the simulator, select the Kidnaped Vehicle, and click start.

## Backgound Knowledge

Particle Filter Workflow:

![flow][img]

[//]: # (Image References)
[gif]: ./extra/video.gif
[img]: ./extra/flow.png



