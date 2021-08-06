import requests
import json
import os

CONFIG_DIST_SERVER_URI = "http://49.50.175.88:6000/config/"
HOME_PATH = os.environ['HOME']
BASHRC_PATH = f"{HOME_PATH}/.bashrc"
BASHRC_BAK_PATH = f"{BASHRC_PATH}-bak"
TARGET_PHRASE = "ROS_MASTER_URI"

if __name__ == "__main__":
    response = requests.get(f"{CONFIG_DIST_SERVER_URI}server")
    config = json.loads(response.text)
    print(f"Fetched config file for server from {CONFIG_DIST_SERVER_URI}")

    server_ip = config["server"]["ip"]
    print(f"Target ROS master URI is {server_ip}")

    with open(BASHRC_PATH, 'r') as file:
        bashrc_data = file.readlines()
    print(f"Read {len(bashrc_data)} lines from {BASHRC_PATH}")

    with open(BASHRC_BAK_PATH, 'w') as file:
        file.writelines(bashrc_data)
    print(f"Backed up ~/.bashrc in {BASHRC_BAK_PATH}")

    for i, line in enumerate(bashrc_data):
        if TARGET_PHRASE in line:
            new_line = f"{TARGET_PHRASE}=http://{server_ip}:11311/"
            print(f"Found {line[:-1]} in line number {i}. Replacing to {new_line}")
            bashrc_data[i] = new_line + "\n"

    with open(BASHRC_PATH, 'w') as file:
        file.writelines(bashrc_data)
    print(f"Wrote new ~/.bashrc in {BASHRC_PATH}")
    print(f"Make sure to: source ~/.bashrc")
