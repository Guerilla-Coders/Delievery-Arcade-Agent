#!/usr/bin/env python3

import rospy
from rospy.exceptions import ROSException
import socketio
from da_pkg.config_fetcher.config_fetcher import fetch_config
from da_pkg.config_fetcher.network_config import NetworkConfig
from da_pkg.physics_processing import map_number
from da_pkg.consts import Limits
from da_pkg.agent import DeliveryArcadeAgent
from da_pkg.datatypes.commands import Movement

sio = socketio.Client()


if __name__ == "__main__":
    network_config = NetworkConfig(fetch_config("server"))
    print(f"Control server URI: http://{network_config.ip}:{network_config.ports.control}")

    # DeliveryArcadeAgent instance generated
    Robot = DeliveryArcadeAgent()
    rospy.loginfo("Created DeliveryArcadeAgent()")


    @sio.on('connect', namespace='/agent')
    def on_connect():
        rospy.loginfo("Connection established to control server")


    @sio.event
    def connect_error(data):
        rospy.loginfo(f"The connection failed! {data}")


    @sio.on('command', namespace='/agent')
    def receive_command(data):
        command = Movement(data)
        Robot.target_linear_vel = map_number(command.throttle, -32, 32,
                                             -Limits.WAFFLE_MAX_LIN_VEL, Limits.WAFFLE_MAX_LIN_VEL)
        Robot.target_angular_vel = map_number(command.steer, -32, 32,
                                              Limits.WAFFLE_MAX_ANG_VEL, -Limits.WAFFLE_MAX_ANG_VEL)
        rospy.loginfo(f'Received command from server. data: {command}')


    @sio.event
    def disconnect():
        rospy.loginfo('Disconnected from control server')


    def run_robot_forever():
        while True:
            Robot.run()


    try:
        sio.connect(f'http://{network_config.ip}:{network_config.ports.control}', namespaces=["/agent"])
        task = sio.start_background_task(run_robot_forever)
        print(f"Robot status {task.is_alive()}")
        task.join()

    except ROSException as error:
        print(f"{error}")

    finally:
        Robot.terminator()
        print("Warning : Teleoperation Shutdown!\n")
        Robot.do_publishing()
