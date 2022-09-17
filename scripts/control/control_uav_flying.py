import rospy
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from mavros_msgs.msg import State, PositionTarget
import sys, select, os
import tty, termios
from std_msgs.msg import String


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]

    if multirotor_num == 18:
        formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
    elif multirotor_num == 9:
        formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
    elif multirotor_num == 6:
        formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    elif multirotor_num == 1:
        formation_configs = ['stop controlling']

    rospy.init_node(multirotor_type + '_uav_flying_control')

    cmd = String()
    target_pose = Pose()

    multi_cmd_pose_pub = [None] * multirotor_num
    multi_cmd_pub = [None] * multirotor_num
    for i in range(multirotor_num):
        multi_cmd_pose_pub[i] = rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + "/cmd_pose_flu", Pose, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/' + multirotor_type + '_' + str(i) + '/cmd', String, queue_size=1)

    cmd_vel_mask = True
    while True:
        key = getKey()
        if key == 'r':
            cmd = 'AUTO.RTL'
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            cmd_vel_mask = True
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            cmd = ''
            # print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            cmd_vel_mask = False
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            cmd_vel_mask = True
            print('Landing')
        elif key in ['k', 's']:
            cmd_vel_mask = False
            cmd = 'HOVER'
            print('Hover')
        else:
            for i in range(10):
                if key == str(i):
                    cmd = formation_configs[i]
                    print(cmd)
                    cmd_vel_mask = True
            if key == '\x03':
                break

        target_pose.position.x = 4.5
        target_pose.position.y = 1.0
        target_pose.position.z = 1.0

        for i in range(multirotor_num):
            if not cmd_vel_mask:
                print('publish setpoint now')
                if control_type == 'vel':
                    multi_cmd_pose_pub[i].publish(target_pose)
                else:
                    multi_cmd_pose_pub[i].publish(target_pose)
            multi_cmd_pub[i].publish(cmd)

        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
