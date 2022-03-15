import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from numpy import array, floor
from numpy.linalg import norm

#  ________________________________________________________________________________


def gridValue(map_data, Xp):
    # Create a method to wait on map...

    # rclpy.node.get_logger('node name').info('Res' + str(map_data) + ' Res  ' + str(map_data.info.resolution))
    resolution = round(map_data.info.resolution, 2)
    Xstartx = map_data.info.origin.position.x
    Xstarty = map_data.info.origin.position.y

    width = map_data.info.width
    Data = map_data.data
    #  returns grid value at "Xp" location
    #  map data:  100 occupied      -1 unknown       0 free
    # If data is bad sometimes get a division by zero error
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution))

    # rclpy.node.get_logger('node name').info('Index ' + str(index))
    if int(index) < len(Data):
        # rclpy.node.get_logger('node name').info('I am a wall ' + str(Data[int(index)]))
        return Data[int(index)]
    return 100


#  ________________________________________________________________________________


def index_of_point(map_data, Xp):
    """
    index_of_point
    """
    resolution = map_data.info.resolution
    Xstartx = map_data.info.origin.position.x
    Xstarty = map_data.info.origin.position.y
    width = map_data.info.width
    Data = map_data.data
    index = int((floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution)))
    return index


def point_of_index(map_data, i):
    """
    point_of_index
    """
    y = map_data.info.origin.position.y + (i / map_data.info.width) * map_data.info.resolution  # + 0.006
    x = (i - (floor((y - map_data.info.origin.position.y) / map_data.info.resolution) * map_data.info.width)) * map_data.info.resolution + map_data.info.origin.position.x

    if y > 0:
        y = y - 0.006
    else:
        y = y - 0.002
    """
    map_data.info.origin.position.x + (i-(i/map_data.info.width)*(map_data.info.width))*map_data.info.resolution
    """
    return array([x, y])


#  ________________________________________________________________________________


def informationGain(map_data, point, r):
    """
    informationGain
    """
    infoGain = 0
    index = index_of_point(map_data, point)
    r_region = int(r / map_data.info.resolution)
    init_index = index - r_region * (map_data.info.width + 1)

    points = point_of_index(map_data, index)

    # rclpy.node.get_logger('node name').info('Point Index then Point from index' + str(point) + "  " + str(index) + "   " + str(points))
    for n in range(0, 2 * r_region + 1):
        start = n * map_data.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / map_data.info.width) + 2) * map_data.info.width
        for i in range(start, end + 1):
            if 0 <= i < limit and i < len(map_data.data):
                # rclpy.node.get_logger('node name').info('Point ' + str(norm(array(point)-point_of_index(map_data, i))))
                if (map_data.data[i] == -1 and norm(array(point) - point_of_index(map_data, i)) <= r):
                    infoGain += 1


# rclpy.node.get_logger('node name').info('Info gain ' + str(infoGain) + 'Real gain ' + str(infoGain*(map_data.info.resolution**2)))
    return infoGain * (map_data.info.resolution ** 2)
