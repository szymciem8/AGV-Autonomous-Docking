import math
import rosbag
import rospy


def get_angle(error):
    D = 195

    return math.asin(error/math.sqrt(D**2+error**2))

def get_distance(l1, l2):
    X0 = 150   #X0 = 150
    Y0 = 87    #Y0 = 85

    angle = get_angle(l2-l1)
    d = l1 * math.cos(angle)

    d_p = (X0**2 + Y0**2)**(0.5)

    beta = angle + math.atan2(X0, Y0) - math.pi/2

    x = math.sin(beta) * d_p

    distance =  d + x

    return distance 

def get_distance_from_wall(l1, l2):
    # In mm
    X0 = 150   #X0 = 150
    Y0 = 87    #Y0 = 85

    angle = get_angle(l2-l1)
    d = l1 * math.cos(angle)

    return d + math.cos(angle) * X0 + math.sin(angle) * Y0


topics = rospy.get_published_topics()
tp = []
for topic in topics:
    tp.append(topic[0])



bag = rosbag.Bag('test.bag', 'w')
for topic, msg, t in bag.read_messages(topics=tp):
    print(t)

bag.close()