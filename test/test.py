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



if __name__ == "__main__":
    
    sub_ang=rospy.Subscriber("/gazebo/model_states",gam.ModelStates,do_ang,queue_size=1)
    rospy.init_node("test")
    mydiv='cuda'
    velocity_commands=torch.tensor([0.1,0,0],device=mydiv).unsqueeze(0)

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
    rate0.sleep()
    startpos=j_pos[:]
    duaration=1000
    leg_msg=unm.LowCmd()
    for i in range(12):
        leg_msg.motorCmd[i].mode = 10;
        leg_msg.motorCmd[i].Kp = 180;
        leg_msg.motorCmd[i].Kd = 8;
    percent=float(0)
    for i in range(5000):
        percent+=float(1/duaration)
        if percent>1:
            percent=1
        for j in range(12):
            leg_msg.motorCmd[j].q=(1 - percent)*startpos[j] + percent*targetpos[dof_map[j]]
            puber[dof_map[j]].publish(leg_msg.motorCmd[j])
        rate0.sleep()

    rate=rospy.Rate(50)
    actions=torch.tensor(j_pos,device=mydiv).unsqueeze(0)
    while(not rospy.is_shutdown()):
        
        base_ang_vel=torch.tensor([ang.x,ang.y,ang.z],device=mydiv).unsqueeze(0)
        quaternion=torch.tensor([ori.x,ori.y,ori.z,ori.w],device=mydiv).unsqueeze(0)
        roll, pitch, yaw = euler_from_quaternion(quaternion) 
        projected_gravity = torch.stack((roll, pitch, yaw), dim=1)
        joint_pos=torch.tensor(j_pos,device=mydiv).unsqueeze(0)
        joint_vec=torch.tensor(j_vec,device=mydiv).unsqueeze(0)
        obs=torch.cat([base_ang_vel,projected_gravity,velocity_commands,joint_pos,joint_vec,actions],dim=-1)
        
        output=model(obs)
        print(output)
        
        for i in range(12):
            leg_msg.motorCmd[i].mode=10
            leg_msg.motorCmd[i].q=output[0][i]
            leg_msg.motorCmd[i].dq=0
            leg_msg.motorCmd[i].Kp=20
            leg_msg.motorCmd[i].Kd=0.5
            leg_msg.motorCmd[i].tau=0
            puber[dof_map[i]].publish(leg_msg.motorCmd[i])
        actions=output[:]
        rate.sleep()