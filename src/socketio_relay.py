#!/usr/bin/env python3

import rospy
from rospy.exceptions import ROSException
import socketio
from da_pkg.config_fetcher.config_fetcher import fetch_config
from da_pkg.config_fetcher.network_config import NetworkConfig
from da_pkg.agent import DeliveryArcadeAgent, FREQUENT, OFTEN, RARELY
from da_pkg.datatypes.commands import Movement
from da_pkg.datatypes.sound_effects import SoundEffect

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
        command = Movement(data)
        Robot.movement_publisher.set_movement(command)
        rospy.loginfo(f'Received command from server. data: {command}')

    @sio.on('sound_effects', namespace='/agent')
    def receive_sound_effects(data):
        sound_effects = SoundEffect(data)
        Robot.sound_effect_publisher.set_soundeffect(sound_effects)
        rospy.loginfo(f'Received sound_effects from server. data : {sound_effects}')



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
