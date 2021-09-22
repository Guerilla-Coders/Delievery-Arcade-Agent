#!/usr/bin/env python3

import rospy
from rospy.exceptions import ROSException
import socketio
from da_pkg.config_fetcher.config_fetcher import fetch_config
from da_pkg.config_fetcher.network_config import NetworkConfig
from da_pkg.agent import DeliveryArcadeAgent
from da_pkg.datatypes.commands import Movement, SoundEffect, LidAction

sio = socketio.Client()

if __name__ == "__main__":
    network_config = NetworkConfig(fetch_config("server"))
    print(f"Control server URI: http://{network_config.ip}:{network_config.ports.control}")

    # DeliveryArcadeAgent instance generated
    Robot = DeliveryArcadeAgent()

    rospy.loginfo("Created DeliveryArcadeAgent() and its State object")


    @sio.on('connect', namespace='/agent')
    def on_connect():
        rospy.loginfo("Connection established to control server")


    @sio.event
    def connect_error(data):
        rospy.loginfo(f"The connection failed! {data}")


    @sio.on('command', namespace='/agent')
    def receive_command(data):
        rospy.loginfo(f'Received command from server. data: {str(data)[:20]}')
        if 'movement' in data:
            rospy.loginfo(f'Command is "Movement" {str(data["movement"])}')
            command = Movement(data)
            Robot.movement_publisher.set_movement(command)
        elif 'sound_effect' in data:
            rospy.loginfo(f'Command is "SoundEffect" {str(data["sound_effect"])}')
            command = SoundEffect(data)
            pass
        elif 'lid_action' in data:
            rospy.loginfo(f'Command is "LidAction" {str(data["lid_action"])}')
            command = LidAction(data)
            pass


    @sio.event
    def disconnect():
        rospy.loginfo('Disconnected from control server')


    def run_robot_forever():
        while True:
            output = Robot.run()
            [sio.emit('info', dict(data), namespace='/agent') for data in output if data is not None]


    try:
        sio.connect(f'http://{network_config.ip}:{network_config.ports.control}', namespaces=["/agent"])
        task = sio.start_background_task(run_robot_forever)
        print(f"Robot status {task.is_alive()}")
        task.join()

    except ROSException as error:
        print(f"{error}")

    finally:
        Robot.terminate()
        print("Warning : Teleoperation Shutdown!\n")
