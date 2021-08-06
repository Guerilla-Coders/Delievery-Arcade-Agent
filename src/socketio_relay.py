#!/usr/bin/env python3

import rospy
import sys
import os
import select
import tty
import termios
import socketio
from da_pkg.config_fetcher.config_fetcher import fetch_config
from da_pkg.config_fetcher.network_config import NetworkConfig
from da_pkg.agent import DeliveryArcadeAgent

sio = socketio.Client()


def get_key():
    """키보드 좌판 입력을 받아오는 함수"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key_input = sys.stdin.read(1)
    else:
        key_input = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key_input


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    node_dir = os.path.dirname(__file__)

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
    def receive_command(data) -> None:
        rospy.loginfo(f'Received command from server. data: {data}')


    @sio.event
    def disconnect():
        rospy.loginfo('Disconnected from control server')


    try:
        # while True:
        #     # get keyboard input
        #     key = get_key()
        #     if key == 'w':
        #         Robot.add_front_velocity()
        #     elif key == 'x':
        #         Robot.add_back_velocity()
        #     elif key == 'a':
        #         Robot.add_CCW_velocity()
        #     elif key == 'd':
        #         Robot.add_CW_velocity()
        #     elif key == ' ' or key == 's':
        #         Robot.stop()
        #     elif key == '\x03':
        #         break
        #
        #     # set final control linear and angular velocities
        #     Robot.run()
        sio.connect(f'http://{network_config.ip}:{network_config.ports.control}', namespaces=["/agent"])
        task = sio.start_background_task(Robot.run)

    except BaseException as error:
        print(f"Exception {error}")

    finally:
        Robot.terminator()
        print("Warning : Teleoperation Shutdown!\n")
        Robot.do_publishing()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
