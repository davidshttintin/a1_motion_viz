#!/usr/bin/env python
import rospy
import pickle
import numpy as np
import tf
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

def talker(results):
    print results
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rr_foot_pub = rospy.Publisher('RR_foot', TransformStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(int(results['total_grid']/results['time_span']))
    index = 0
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["" for _ in range(18)]
        joint_state.position = [.0 for _ in range(18)]
        joint_state.name[0] = "base_x"
        joint_state.position[0] = results['base_poses'][0, index]
        joint_state.name[1] = "base_y"
        joint_state.position[1] = results['base_poses'][1, index]
        joint_state.name[2] = "base_z"
        joint_state.position[2] = results['base_poses'][2, index]
        joint_state.name[3] = "base_roll"
        joint_state.position[3] = results['base_poses'][3, index]
        joint_state.name[4] = "base_pitch"
        joint_state.position[4] = results['base_poses'][4, index]
        joint_state.name[5] = "base_yaw"
        joint_state.position[5] = results['base_poses'][5, index]

        joint_state.name[6] = "FR_hip_joint"
        joint_state.position[6] = results['FR_states'][0, index]
        joint_state.name[7] = "FR_thigh_joint"
        joint_state.position[7] = results['FR_states'][1, index]
        joint_state.name[8] = "FR_calf_joint"
        joint_state.position[8] = results['FR_states'][2, index]

        joint_state.name[9] = "FL_hip_joint"
        joint_state.position[9] = results['FL_states'][0, index]
        joint_state.name[10] = "FL_thigh_joint"
        joint_state.position[10] = results['FL_states'][1, index]
        joint_state.name[11] = "FL_calf_joint"
        joint_state.position[11] = results['FL_states'][2, index]

        joint_state.name[12] = "RR_hip_joint"
        joint_state.position[12] = results['RR_states'][0, index]
        joint_state.name[13] = "RR_thigh_joint"
        joint_state.position[13] = results['RR_states'][1, index]
        joint_state.name[14] = "RR_calf_joint"
        joint_state.position[14] = results['RR_states'][2, index]

        joint_state.name[15] = "RL_hip_joint"
        joint_state.position[15] = results['RL_states'][0, index]
        joint_state.name[16] = "RL_thigh_joint"
        joint_state.position[16] = results['RL_states'][1, index]
        joint_state.name[17] = "RL_calf_joint"
        joint_state.position[17] = results['RL_states'][2, index]

        
        
        js_pub.publish(joint_state)

        rate.sleep()
        index += 1
        if index == results['total_grid']:
            index = 0

if __name__ == '__main__':
    f = open('/home/daly/guidedog_ws/src/digit-opti/scripts/solved_debug_12s.p', 'r')
    results = pickle.load(f)
    f.close()
    try:
        # result = {}
        # result['total_grid'] = 1
        # result['time_span'] = 1
        # result['base_poses'] = np.array([[num] for num in [5.33124525e-03, 1.40476550e-04, 2.96701968e-01, 2.21779981e-02, -2.95332678e-03, -4.35811163e-01]])
        # result['FR_states'] = np.array([[num] for num in [0.34513559, 0.71611916, -1.5943171]])
        # result['FL_states'] = np.array([[num] for num in [0.10806939, 0.87501563, -1.2839156]])
        # result['RR_states'] = np.array([[num] for num in [-0.16803905, 0.48412645, -1.32322079]])
        # result['RL_states'] = np.array([[num] for num in [-0.39999989, 0.85956598, -1.46872909]])
        talker(results)
    except rospy.ROSInterruptException:
        pass
