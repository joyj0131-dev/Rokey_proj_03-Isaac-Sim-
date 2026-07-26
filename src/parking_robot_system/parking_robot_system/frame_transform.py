"""USD(XZ,+Y상방) ↔ ROS map(XY) 좌표/yaw 변환. 규약: x_map=x_usd, y_map=-z_usd."""


def usd_to_map(x_usd, z_usd):
    return (x_usd, -z_usd)


def map_to_usd(x_map, y_map):
    return (x_map, -y_map)


def usd_yaw_to_map_deg(yaw_usd_deg):
    # y축 반사는 회전 방향을 뒤집는다 → yaw 부호 반전.
    return -yaw_usd_deg


def map_to_usd_yaw_deg(yaw_map_deg):
    return -yaw_map_deg
