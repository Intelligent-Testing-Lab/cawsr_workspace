# CAWSR Tools


This directory contains various quality of life tools.

## Example scenario generator

This script builds the all the example scenarios in `scenarios/examples`. It uses PairWise sampling to cover possible combinations of weather, towns and scenario events. 

To run the generate script, change the docker compose service command to 
```bash
command: -c "source /ros_workspace/autoware_msgs/install/setup.bash && source /opt/ros/humble/setup.bash && python3 /autoware_scenario_runner/scenario_generator.py" 
```
Then, run the CAWSR and carla services
```bash
docker compose up carla cawsr
```

## Generate Replay Video

This simple script generates a replay video from a recorded scenario execution.


### Usage 

1. Have CARLA running in the background
2. Find the recording you want to generate a replay video for e.g., `results/recordings/RouteScenario.log`
3. Run the script:

```sh
python generate_replay_video.py -f results/recordings/RouteScenario.log
```