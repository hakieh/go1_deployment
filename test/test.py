import torch
import rospy
import geometry_msgs.msg as gem
import gazebo_msgs.msg as gam
import unitree_legged_msgs.msg as unm
import sensor_msgs.msg as sem
import torch.nn as nn

class Actor(nn.Module):
    def __init__(self):
        super(Actor, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(45,128),
            nn.ELU(),
            nn.Linear(128,128),
            nn.ELU(),
            nn.Linear(128,128),
            nn.ELU(),
            nn.Linear(128,12),
        )
    
    def forward(self, x):
        return self.actor(x) 
    

def euler_from_quaternion(quat_angle):
        x = quat_angle[:,0]; y = quat_angle[:,1]; z = quat_angle[:,2]; w = quat_angle[:,3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = torch.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = torch.clip(t2, -1, 1)
        pitch_y = torch.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = torch.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

ang=gem.Vector3()
ori=gem.Quaternion()
def do_ang(msg_ang):
    global ang,ori
    ang=msg_ang.twist[2].angular
    ori=msg_ang.pose[2].orientation

dof_map= [2, 0, 1, 5, 3, 4, 8, 6, 7, 11, 9, 10]
j_pos=list()
j_vec=list()
def do_joint(msg_joi):
    global j_pos,j_vec
    j_pos=list(msg_joi.position[dof_map[i]] for i in range(0,12))
    j_vec=list(msg_joi.velocity[dof_map[i]] for i in range(0,12))


if __name__ == "__main__":
    
    sub_ang=rospy.Subscriber("/gazebo/model_states",gam.ModelStates,do_ang,queue_size=1)
    rospy.init_node("test")
    mydiv='cpu'
    velocity_commands=torch.tensor([1,0,0],device=mydiv).unsqueeze(0)

    sub_joint=rospy.Subscriber("/go1_gazebo/joint_states",sem.JointState,do_joint,queue_size=1)

    puber=list()
    puber.append(rospy.Publisher("/go1_gazebo/FL_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FL_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FL_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_calf_controller/command",unm.MotorCmd,queue_size=1))

    rate=rospy.Rate(1)
    rate.sleep()
    actions=torch.tensor([j_pos],device=mydiv)
    
    model = Actor()
    weights=torch.load("./source/standalone/workflows/rsl_rl/weight_1/model_3500.pt")
    model.load_state_dict(weights['model_state_dict'],strict=False)

    while(not rospy.is_shutdown()):
        
        base_ang_vel=torch.tensor([ang.x,ang.y,ang.z],device=mydiv).unsqueeze(0)
        quaternion=torch.tensor([ori.x,ori.y,ori.z,ori.w],device=mydiv).unsqueeze(0)
        roll, pitch, yaw = euler_from_quaternion(quaternion) 
        projected_gravity = torch.stack((roll, pitch, yaw), dim=1)
        joint_pos=torch.tensor([j_pos],device=mydiv)
        joint_vec=torch.tensor([j_vec],device=mydiv)
        
        obs=torch.cat([base_ang_vel,projected_gravity,velocity_commands,joint_pos,joint_vec,actions],dim=-1)
        
        output=model(obs)

        leg_msg=unm.LowCmd()
        for i in range(12):
            leg_msg.motorCmd[i].mode=10
            leg_msg.motorCmd[i].q=output[i]
            leg_msg.motorCmd[i].dq=0.1
            leg_msg.motorCmd[i].dq=0
            leg_msg.motorCmd[i].Kp=80
            leg_msg.motorCmd[i].Kd=2
            leg_msg.motorCmd[i].tau=0
            puber[i].publish(leg_msg.motorCmd[i])
        actions=output[:]
        rate.sleep()