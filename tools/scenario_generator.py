import json
import os
import numpy as np
import gc
import random
import carla
from allpairspy import AllPairs
from collections import OrderedDict
import lanelet2
import lanelet2.io
import lanelet2.geometry
import lanelet2.core
from srunner.tools import route_manipulation
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from agents.navigation.global_route_planner import GlobalRoutePlanner

import time

HOST = '127.0.0.1'
PORT = 2000

class CARLAScenarioGenerator:
    def __init__(self, lanelet2_map_key: dict) -> None:
        """
        Initialize the CARLA Scenario Generator.
    
        Args:
            lanelet2_map_key (dict): Dict containing paths to lanelet2 files by town name.
            default_scenario_path (str): The file path to the default scenario JSON.
        """
        self.lanelet2 = lanelet2_map_key
        self.laneletmap_ = None
        self._map = None
        self._route_planner = None

        # weather options   
        self.weather_options = {
            "sunny": {
                "route_percentage": 100.0,
                "precipitation": 0.0,
                "cloudiness": 0.0,
                "precipitation_deposits": 0.0,
                "wetness": 0.0,
                "wind_intensity": 0.0,
                "sun_azimuth_angle": -1.0,
                "sun_altitude_angle": 90.0,
                "fog_density": 2.0,
            },
            "rainy": {
                "route_percentage": 0.0,
                "precipitation": 100.0,
                "cloudiness": 0.0,
                "precipitation_deposits": 100.0,
                "wetness": 100.0,
                "wind_intensity": 100.0,
                "sun_azimuth_angle": -1.0,
                "sun_altitude_angle": 90.0,
                "fog_density": 2.0,
            },
            "cloudy": {
                "route_percentage": 0.0,
                "precipitation": 0.0,
                "cloudiness": 100.0,
                "precipitation_deposits": 0.0,
                "wetness": 0.0,
                "wind_intensity": 0.0,
                "sun_azimuth_angle": -1.0,
                "sun_altitude_angle": 90.0,
                "fog_density": 2.0,
            },
        }
        
        # scenario events 
        self.event_options = [
            "FollowLeadingVehicle",
            "FollowLeadingVehicleWithObstacle",
            "VehicleTurningRight",
            "VehicleTurningLeft",
            "OppositeVehicleRunningRedLight",
            "StationaryObjectCrossing",
            "DynamicObjectCrossing",
            "NoSignalJunctionCrossing",
            "ControlLoss",
            "SignalizedJunctionRightTurn",
            "SignalizedJunctionLeftTurn",
            "ConstructionObstacle",
            "HardBreak",
            "Accident",
        ]

        # carla maps        
        self.map_options = [
            "Town01",
            "Town02",
            "Town03",
            "Town04",
            "Town05",
            "Town06",
            "Town07",
            "Town10HD",
        ]

    def _np_to_json(self, p1: np.ndarray) -> dict:
        """Convert numpy array to JSON 

        Args:
            p1 (np.ndarray): 3D point

        Returns:
            dict: JSON format compatible with CAWSR
        """
        return {"position": {"x": float(p1[0]), "y": float(p1[1]), "z": 0.0}}

    def _get_all_lanelet_points(self, town: str) -> np.ndarray:
        """Extract all centerline points from lanelet2 map

        Args:
            town (str): Lanelet2 map key

        Returns:
            np.ndarray: Array of 2D center line points 
        """
        lanelet2_osm = self.lanelet2[town]
        laneletmap_ = lanelet2.io.load(lanelet2_osm, lanelet2.io.Origin(0, 0))
        
        self.laneletmap_ = laneletmap_

        centerline_points = []
        for lanelet in list(self.laneletmap_.laneletLayer):
            for points in lanelet.centerline:
                centerline_points.append(np.asarray([points.x, points.y]))

        return np.asarray(centerline_points)

    def _not_same_lane_check(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        """Check if two points are NOT in the same lane.

        Args:
            p1 (np.ndarray): First point to compare
            p2 (np.ndarray): Second point to compare

        Returns:
            bool: True if points are not in the same lane, False otherwise
        """
        p1_point = lanelet2.core.BasicPoint2d(p1[0], p1[1])
        p2_point = lanelet2.core.BasicPoint2d(p2[0], p2[1])

        lanelets = [
            lanelet2.geometry.findNearest(self.laneletmap_.laneletLayer, p1_point, 1),
            lanelet2.geometry.findNearest(self.laneletmap_.laneletLayer, p2_point, 1),
        ]

        lanelet_ids = [
            {ll.id for dist, ll in lanelets[0]},
            {ll.id for dist, ll in lanelets[1]},
        ]
        common_lanes = lanelet_ids[0].intersection(lanelet_ids[1])
        return not common_lanes
    
    def _to_carla(self, point: np.ndarray) -> carla.Location:
        """Convert 3D point to carla.Location

        Args:
            point (np.ndarray): 3D point

        Returns:
            carla.Location: CARLA compatible position_
        """
        return carla.Location(point[0], point[1], 0.0)

    def _dist(self, p1: np.ndarray, p2: np.ndarray) -> np.floating:
        """ Euclidean distance between points

        Args:
            p1 (np.ndarray): First point
            p2 (np.ndarray): Second point

        Returns:
            np.floating: Euclidean distance
        """
        return np.linalg.norm(p1 - p2)

    def generate_valid_route(self, town, min_distance=50.0, max_attempts=100):
        """Generate a valid route with start and end waypoints."""
        all_points = self._get_all_lanelet_points(town)
        
        for _ in range(max_attempts):
            # Randomly select two points
            idx1, idx2 = random.sample(range(len(all_points)), 2)
            p1, p2 = all_points[idx1], all_points[idx2]
            
            # Check if points are far enough and not in the same lane
            if not self._dist(p1, p2) >= min_distance or not self._not_same_lane_check(p1, p2):
                continue
            
            if self.valid_route([p1, p2]):
                return [p1, p2]
        
    def valid_route(self, route) -> bool:
        carla_route = list(map(self._to_carla, route))

        gps_route, route = route_manipulation.interpolate_trajectory(carla_route, grp=self._route_planner)
        return not ((len(gps_route) == 1) and (len(route) == 1))

    def pick_trigger_point_on_route(self, waypoint_start, waypoint_end):
        """Pick a random point along the route for scenario trigger."""
        # Interpolate between start and end waypoints
        # Pick a point between 20% and 80% of the route
        t = random.uniform(0.2, 0.8)
        trigger_point = waypoint_start + t * (waypoint_end - waypoint_start)
        
        # Calculate yaw angle (direction from start to end)
        direction = waypoint_end - waypoint_start
        yaw = float(np.degrees(np.arctan2(direction[1], direction[0])))
        
        return {
            "x": float(trigger_point[0]),
            "y": float(trigger_point[1]),
            "z": 0.0,
            "yaw": yaw
        }

    def create_scenario_definition(self, town_map, weather_start, weather_end, 
                                   event_type, route_id, waypoints, trigger_point):
        """Create a complete scenario definition based on parameters."""
        # Create weather list with start and finish conditions
        weather_start_config = self.weather_options[weather_start].copy()
        weather_start_config["route_percentage"] = 0.0
        
        weather_end_config = self.weather_options[weather_end].copy()
        weather_end_config["route_percentage"] = 100.0
        
        # Build scenario structure
        scenario_def = {
            "routes": [
                {
                    "route": {
                        "id": route_id,
                        "weathers": [
                            {"weather": weather_start_config},
                            {"weather": weather_end_config}
                        ],
                        "waypoints": [
                            self._np_to_json(waypoints[0]),
                            self._np_to_json(waypoints[1])
                        ],
                        "scenarios": [
                            {
                                "scenario": {
                                    "name": f"{event_type}_{route_id}",
                                    "type": event_type,
                                    "trigger_point": trigger_point
                                }
                            }
                        ]
                    }
                }
            ],
            "scenarios": [
                {
                    "scenario": {
                        "town": town_map,
                        "ego_vehicle": {
                            "x": float(waypoints[0][0]),
                            "y": float(waypoints[0][1]),
                            "z": 0,
                            "yaw": float(np.degrees(np.arctan2(
                                waypoints[1][1] - waypoints[0][1],
                                waypoints[1][0] - waypoints[0][0]
                            ))),
                            "model": "vehicle.toyota.prius",
                            "name": "ego_vehicle"
                        }
                    }
                }
            ]
        }
        
        return scenario_def
    
    def initialize_carla_world(self, town):
        """Initialize CARLA client connection."""
        # only initialise new map if different from current
        if self._map != town or self._route_planner is None:
            
            del self._route_planner
            self._route_planner = None
            CarlaDataProvider.cleanup()

            # force garbage collection to free memory
            gc.collect()
            time.sleep(2)
            
            print("Loading CARLA world:", town)
            self.client.load_world(town)
            CarlaDataProvider.set_world(self.client.get_world())
            self._map = town
            
            print("Generating new planner for CARLA world:", town)
            self._route_planner = GlobalRoutePlanner(CarlaDataProvider.get_map(), 1.0)
        
    def initialise_carla_client(self, host='127.0.0.1', port=2000, timeout=20.0):
        """Initialise CARLA client connection."""
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        CarlaDataProvider.set_client(self.client)

        
    def generate_pairwise_scenarios(self, output_path="scenarios/examples/"):
        """Generate all pairwise test scenarios and save to file."""
        
        # Define parameters for pairwise testing
        parameters = OrderedDict({
            "map": sorted(self.map_options),
            "weather_at_start": list(self.weather_options.keys()),
            "weather_at_finish": list(self.weather_options.keys()),
            "event_at_last_waypoint": self.event_options,
        })
        
        time.sleep(10)  # wait for carla server to be ready
        
        # initialise carla
        self.initialise_carla_client(
            host=HOST,
            port=PORT
        )
        
        # Generate pairwise combinations
        all_scenarios = []
        route_id = 0
        
        # sort combinations by town to avoid reloading map too often
        all_pairs = sorted(AllPairs(parameters), key=lambda x: x[0])

        for combination in all_pairs:
            town_map = combination[0]
            weather_start = combination[1]
            weather_end = combination[2]
            event_type = combination[3]
            
            self.initialize_carla_world(town_map)
                        
            print(f"Generating scenario {route_id}: Map={town_map}, "
                  f"Weather={weather_start}->{weather_end}, Event={event_type}")
            
            # Generate valid route for this map
            waypoints = self.generate_valid_route(town_map)
            
            # Pick trigger point along the route
            trigger_point = self.pick_trigger_point_on_route(waypoints[0], waypoints[1])
            
            # Create scenario definition
            scenario = self.create_scenario_definition(
                town_map, weather_start, weather_end, 
                event_type, route_id, waypoints, trigger_point
            )
            
            all_scenarios.append(scenario)
            route_id += 1

            with open(os.path.join(output_path, f'{town_map}_{event_type}_{weather_start}.json'), 'w') as f:
                json.dump(scenario, f, indent=2)
            
            print(f"Saving scenario {route_id}: Map={town_map}, "
                  f"Weather={weather_start}->{weather_end}, Event={event_type}")
             
        print(f"\nGenerated {len(all_scenarios)} pairwise scenarios")
        print(f"Saved to {output_path}")
        
        return all_scenarios


if __name__ == "__main__":
    lanelet2_osms = {
            "Town01": "algorithms/resources/Town01.osm",
            "Town02": "algorithms/resources/Town02.osm",
            "Town03": "algorithms/resources/Town03.osm",
            "Town04": "algorithms/resources/Town04.osm",
            "Town05": "algorithms/resources/Town05.osm",
            "Town06": "algorithms/resources/Town06.osm",
            "Town07": "algorithms/resources/Town07.osm",
            "Town10HD": "algorithms/resources/Town10HD.osm",
    }
    
    generator = CARLAScenarioGenerator(
        lanelet2_map_key=lanelet2_osms,
    )
    
    scenarios = generator.generate_pairwise_scenarios()