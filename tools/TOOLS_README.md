# CAWSR Tools


This directory contains various quality of life tools.


## Generate Replay Video

This simple script generates a replay video from a recorded scenario execution.


### Usage 

1. Have CARLA running in the background
2. Find the recording you want to generate a replay video for e.g., `results/recordings/RouteScenario.log`
3. Run the script:

```sh
python generate_replay_video.py -f results/recordings/RouteScenario.log
```