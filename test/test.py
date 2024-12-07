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

def quat_rotate_inverse(q):
    shape=q.shape
    v=torch.tensor([0,0,-1],dtype=torch.float,device=mydiv).unsqueeze(0)
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

ang=gem.Vector3()
ori=gem.Quaternion()
def do_imu(msg_ang):
    global ang,ori
    ang=msg_ang.angular_velocity
    ori=msg_ang.orientation

dof_map= [1, 4, 7, 10,
         2, 5, 8, 11   ,
          0, 3, 6, 9
          ]
j_pos=list()
j_vec=list()
def do_joint(msg_joi):
    global j_pos,j_vec
    j_pos=list(msg_joi.position[dof_map[i]] for i in range(0,12))
    j_vec=list(msg_joi.velocity[dof_map[i]] for i in range(0,12))
    #print(list(msg_joi.name[dof_map[i]] for i in range(0,12)))



if __name__ == "__main__":
    
    sub_imu=rospy.Subscriber("/trunk_imu",sem.Imu,do_imu,queue_size=1)
    rospy.init_node("test")
    mydiv='cuda'
    velocity_commands=torch.tensor([0.7,0,0],device=mydiv).unsqueeze(0)

    sub_joint=rospy.Subscriber("/go1_gazebo/joint_states",sem.JointState,do_joint,queue_size=1)

    puber=list()
    puber.append(rospy.Publisher("/go1_gazebo/FL_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FL_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FL_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/FR_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RL_thigh_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_calf_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_hip_controller/command",unm.MotorCmd,queue_size=1))
    puber.append(rospy.Publisher("/go1_gazebo/RR_thigh_controller/command",unm.MotorCmd,queue_size=1))
    
    model = Actor()
    weights=torch.load("./model_3500.pt")
    model.load_state_dict(weights['model_state_dict'],strict=False)
    model.eval()
    model = model.to(mydiv)
    
    rate0=rospy.Rate(1000)
    targetpos=[-1.3,0.0,0.67,-1.3,0.0,0.67,-1.3,0.0,0.67,-1.3,0.0,0.67]
    kkp=[180,300,180,180,300,180,180,300,180,180,300,180]
    kkd=[8,15,8,8,15,8,8,15,8,8,15,8]
    rate0.sleep()
    startpos=j_pos[:]
    duaration=1000
    leg_msg=unm.LowCmd()
    for i in range(12):
        leg_msg.motorCmd[i].mode = 10
        leg_msg.motorCmd[i].Kp = 180
        leg_msg.motorCmd[i].Kd = 8
    percent=float(0)
    for i in range(3000):
        percent+=float(1/duaration)
        if percent>1:
            percent=1
        for j in range(12):
            leg_msg.motorCmd[j].q=(1 - percent)*startpos[j] + percent*targetpos[dof_map[j]]
            puber[dof_map[j]].publish(leg_msg.motorCmd[j])
        rate0.sleep()

    rate=rospy.Rate(50)
    actions=torch.tensor([0,0,0,0,0,0,0,0,0,0,0,0],device=mydiv).unsqueeze(0)
    rqt_plus=[-1.5,0.1,0.8,-1.5,-0.1,0.8,-1.5,0.1,1,-1.5,-0.1,1]
    model_plus=torch.tensor(list(rqt_plus[dof_map[i]] for i in range(0,12)),device=mydiv).unsqueeze(0)
    for i in range(12):
        leg_msg.motorCmd[i].mode = 10
        leg_msg.motorCmd[i].Kp = 40
        leg_msg.motorCmd[i].Kd = 0.5
    ssss=1
    while(not rospy.is_shutdown()):
        
        base_ang_vel=torch.tensor([ang.x,ang.y,ang.z],device=mydiv).unsqueeze(0)
        quaternion=torch.tensor([ori.x,ori.y,ori.z,ori.w],device=mydiv).unsqueeze(0)
        projected_gravity = quat_rotate_inverse(quaternion)
        joint_pos=torch.tensor(j_pos,device=mydiv).unsqueeze(0)-model_plus
        joint_vec=torch.tensor(j_vec,device=mydiv).unsqueeze(0)
        obs=torch.cat([base_ang_vel,projected_gravity,velocity_commands,joint_pos,joint_vec,actions],dim=-1)
        actions=model(obs)
        output=actions[:]*0.25+model_plus
        print(output)
        for i in range(12):
            leg_msg.motorCmd[i].mode=10
            leg_msg.motorCmd[i].q=output[0][i]
            leg_msg.motorCmd[i].dq=0
            leg_msg.motorCmd[i].tau=0
            puber[dof_map[i]].publish(leg_msg.motorCmd[i])
        #actions=output[:]
        rate.sleep()

'''
FLhip->FRhip->...->thigh->calf
dof_map= [1, 4, 7, 10,
         2, 5, 8, 11   ,
          0, 3, 6, 9
          ]

FLhip->FLthigh->FLcalf->...->FR
dof_map=[1,2,0,4,5,3,7,8,6,10,11,9]
'''