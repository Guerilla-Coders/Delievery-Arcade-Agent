from dataclasses import dataclass


@dataclass
class LeftRightPorts:
    L: int
    R: int


@dataclass
class VideoPorts:
    post: LeftRightPorts
    broadcast: LeftRightPorts


@dataclass
class Ports:
    video: VideoPorts
    control: int


class NetworkConfig:
    ip: str
    secret_key: str
    ports: Ports

    def __init__(self, config: dict):
        self.config = config

        self.ip = self.config["server"]["ip"]
        self.secret_key = self.config["server"]["secret_key"]

        video_post_l = self.config["server"]["port"]["video"]["post"]["L"]
        video_post_r = self.config["server"]["port"]["video"]["post"]["R"]
        video_post_port = LeftRightPorts(video_post_l, video_post_r)

        video_cast_l = self.config["server"]["port"]["video"]["broadcast"]["L"]
        video_cast_r = self.config["server"]["port"]["video"]["broadcast"]["R"]
        video_cast_port = LeftRightPorts(video_cast_l, video_cast_r)

        video_ports = VideoPorts(video_post_port, video_cast_port)
        control_port = self.config["server"]["port"]["control"]

        self.ports = Ports(video_ports, control_port)
