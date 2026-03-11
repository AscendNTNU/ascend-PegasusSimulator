#!/usr/bin/env python

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})
# simulation_app = SimulationApp({"headless": True})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.sensors import Magnetometer, IMU, Barometer, GPS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])


        # Spawn 5 vehicles with the PX4 control backend in the simulation, separated by 1.0 m along the x-axis
        for i in range(1):
            self.vehicle_factory(i, gap_x_axis=1.0)
        

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def vehicle_factory(self, vehicle_id: int, gap_x_axis: float):
        """Auxiliar method to create multiple multirotor vehicles

        Args:
            vehicle_id (_type_): _description_
        """

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()

        namespace = "drone_sim"
        
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig({
            # "vehicle_id": vehicle_id,
            "vehicle_id": vehicle_id,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe # CHANGE this line to 'iris' if using PX4 version bellow v1.14
        })
        #config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config), ROS2Backend(vehicle_id=vehicle_id, config={
            "namespace": namespace,
            "pub_sensors": True,
            "pub_imu": True,
            "pub_gps": True,
            "pub_graphical_sensors": True,
            "pub_state": True,
            "pub_tf": False,
            "sub_control": True,
            })
        ]
        #self.camera = MonocularCamera("camera", config={"update_rate": 60.0})
        #self.lidar = Lidar("Lidar")
        #config_multirotor.graphical_sensors = [self.camera, self.lidar]
        
        # Sensors
        front_camera_prim_path = "body/front_camera"
        down_camera_prim_path = "body/down_camera"
        lidar_prim_path = "body/lidar"
        front_camera_config = {
            #"position": [0.1, 0.0, 0.07],
            ##"orientation": Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()
            "depth": True,
            "update_rate": 60.0
        }
        down_camera_config = {
            #"position": [0.1, 0.0, 0.07],
            #"orientation": Rotation.from_euler("XYZ", [0.0, 90.0, 0.0], degrees=True).as_quat()
            "orientation": [0.0, 90.0, 0.0],
            "update_rate": 60.0
            #"depth": True,
            #"update_rate": 60.0
        }
        lidar_config = {
            "sensor_configuration": "Example_Rotary_2D",
            "update_rate": 60.0
        }

        config_multirotor.sensors = [
            Magnetometer(),
            IMU(),
            Barometer(),
            GPS(),
        ]

        config_multirotor.graphical_sensors = [
            MonocularCamera(front_camera_prim_path, front_camera_config),
            MonocularCamera(down_camera_prim_path, down_camera_config),
            Lidar(lidar_prim_path, lidar_config)
        ]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            vehicle_id,
            [gap_x_axis * vehicle_id, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor
            )

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
