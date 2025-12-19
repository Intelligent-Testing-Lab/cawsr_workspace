#!/usr/bin/env python


# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import queue
import re
import time
from pathlib import Path

import carla
import cv2
import numpy as np


def get_log_duration(client, log_file):
    info = client.show_recorder_file_info(log_file, False)
    # Look at for Duration: 12.34 s"
    match = re.search(r"Duration:\s+([0-9.]+)", info)
    if not match:
        raise RuntimeError("Duration time cannot be read!")
    return float(match.group(1))


def get_video_writer(
    frame_size: tuple[int, int], path: str | Path, fps: int
) -> cv2.VideoWriter:
    output_filename = Path(path).with_suffix(".mp4").resolve()
    output_filename.parent.mkdir(parents=True, exist_ok=True)
    print(f"Saving render to {output_filename}")
    codec = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(str(output_filename), codec, fps, frame_size)


def main(args: argparse.Namespace):
    print(f"Generating replay video...{args}")
    # Initialize CARLA client and world
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()

    log_file = Path(args.log_file).resolve()
    assert log_file.is_file(), f"Log file {log_file} does not exist."

    print(f"Replaying log file: {str(log_file)}")
    client.replay_file(str(log_file), 0, 0, 0)

    ego_vehicle_type = args.ego_vehicle_type

    MAX_WAIT_TIME = time.time() + 60  # seconds
    while True:
        actors = world.get_actors()
        ego_vehicle = next(
            (actor for actor in actors if actor.type_id == ego_vehicle_type), None
        )
        if ego_vehicle:
            print(f"Ego vehicle '{ego_vehicle_type}' found.")
            break
        print(f"Waiting for ego vehicle '{ego_vehicle_type}' to spawn...")
        if time.time() > MAX_WAIT_TIME:
            raise RuntimeError(
                f"Ego vehicle '{ego_vehicle_type}' not found in the replay within the time limit."
            )
        else:
            time.sleep(1)

    # Attach the camera to the vehicle
    print("Attaching camera to the ego vehicle...")
    camera = world.get_blueprint_library().find("sensor.camera.rgb")
    camera.set_attribute("image_size_x", str(args.width))
    camera.set_attribute("image_size_y", str(args.height))
    camera.set_attribute("fov", str(args.fov))

    camera_transform = carla.Transform(
        carla.Location(x=0, y=0, z=3), carla.Rotation(pitch=-15)
    )
    camera = world.spawn_actor(camera, camera_transform, attach_to=ego_vehicle)

    frame_q = queue.Queue(maxsize=1)  # save RGB frame

    def _safe_put(item):
        try:
            frame_q.put_nowait(item)
        except queue.Full:
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
            frame_q.put_nowait(item)

    def process_image(image):
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8)
        bgra = np.reshape(bgra, (image.height, image.width, 4))
        bgr = bgra[:, :, :3].copy()
        rgb = bgr[:, :, ::-1]
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _safe_put(rgb)

    video_writer = get_video_writer((args.width, args.height), args.out_path, args.fps)
    camera.listen(lambda img: process_image(img))

    duration = get_log_duration(client, str(log_file))
    t0_sim = world.get_snapshot().timestamp.elapsed_seconds
    duration += t0_sim
    prev_sim_time = t0_sim
    frame_interval = 1 / args.fps
    frame_count = 0
    try:
        while True:
            snapshot = world.get_snapshot()
            sim_time = snapshot.timestamp.elapsed_seconds

            if sim_time >= duration:
                print("Replay finished")
                break

            if sim_time - prev_sim_time < frame_interval:
                continue

            # Get frame and write to video
            try:
                rgb = frame_q.get_nowait()
                video_writer.write(rgb)
                frame_count += 1
                print(
                    f"Saving frame: {frame_count}. Sim seconds to go {(duration - sim_time):.2f}s"
                )
            except queue.Empty:
                continue

            prev_sim_time = sim_time
    except KeyboardInterrupt:
        print("Exit...")
    except Exception as e:
        print(e)
    finally:
        print("Saving the video file...")
        if camera is not None:
            camera.stop()
            camera.destroy()
        ego_vehicle.destroy()
        video_writer.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Capture frames from a CARLA replay and create a mp4 video.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="CARLA's IP of the host server (default: 127.0.0.1)",
    )
    parser.add_argument(
        "-p",
        "--port",
        default=2000,
        type=int,
        help="CARLA's TCP port to listen to (default: 2000)",
    )
    parser.add_argument(
        "-f",
        "--log-file",
        required=True,
        help="Path to the CARLA replay log file.",
    )
    parser.add_argument(
        "-o",
        "--out-path",
        type=str,
        default="output_video.mp4",
        help="Path to save the output video file (default: output_video.mp4).",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1920,
        help="Video resolution width (default: 1920).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1080,
        help="Video resolution height (default: 1080).",
    )
    parser.add_argument(
        "--fov",
        type=int,
        default=90,
        help="Camera field of view (default: 90).",
    )
    parser.add_argument(
        "--ego-vehicle-type",
        type=str,
        default="vehicle.lincoln.mkz_2017",
        help="Type of the ego vehicle to attach the camera to (default: vehicle.lincoln.mkz_2017).",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="Frames per second to capture from the replay (default: 10).",
    )

    args = parser.parse_args()
    main(args)
