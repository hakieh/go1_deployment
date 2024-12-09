import torch
import rospy
import torch.nn as nn

#以下是一些订阅gazebo话题会用到的数据类型
import geometry_msgs.msg as gem    #gazebo位姿信息
import unitree_legged_msgs.msg as unm   #控制狗子12个关节电机
import sensor_msgs.msg as sem   #订阅imu信息

#以下是用来加载强化学习模型用到的网络结构，没有特殊功能
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

#以下是根据四元数推算projected_gravity的函数，从go1_real.py上面扒的
def quat_rotate_inverse(q):    #q是狗子的四元数
    shape=q.shape
    v=torch.tensor([0,0,-1],dtype=torch.float,device=mydiv).unsqueeze(0)    #v是重力之类的，默认为(0,0,-1)
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

#以下是订阅imu话题的反馈函数
ang=gem.Vector3()   #角速度
ori=gem.Quaternion()    #四元数
def do_imu(msg_ang):
    global ang,ori
    ang=msg_ang.angular_velocity
    ori=msg_ang.orientation

#dof_map为12个关节电机的gazebo顺序到模型顺序的映射
#具体来说gazebo[dof_map[i]] = model[i]
dof_map= [1, 4, 7, 10,
         2, 5, 8, 11   ,
          0, 3, 6, 9
          ]

#以下是订阅12个关节位置以及速度话题的反馈函数
j_pos=list()    #关节位置(gazebo顺序)
j_vec=list()    #关节速度(gazebo顺序)
def do_joint(msg_joi):
    global j_pos,j_vec
    j_pos=list(msg_joi.position[dof_map[i]] for i in range(0,12))   #关节位置(model顺序)
    j_vec=list(msg_joi.velocity[dof_map[i]] for i in range(0,12))   #关节速度(model顺序)
    #print(list(msg_joi.name[dof_map[i]] for i in range(0,12)))     #测试语句，仅输出关节顺序测试dof_map有效性


#主体函数
if __name__ == "__main__":
    rospy.init_node("test")     #初始化节点名称
    mydiv='cuda'    #设置模型在GPU上运行
    velocity_commands=torch.tensor([0.7,0,0],device=mydiv).unsqueeze(0)     #这是obs中的一个元素，控制狗子运行的目标速度

    sub_imu=rospy.Subscriber("/trunk_imu",sem.Imu,do_imu,queue_size=1)      #订阅imu话题
    sub_joint=rospy.Subscriber("/go1_gazebo/joint_states",sem.JointState,do_joint,queue_size=1)     #订阅关节位置以及速度的话题

    puber=list()    #用于发布12个关节的指令，顺序为gazebo顺序
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
    
    model = Actor()     #实例化网络结构
    weights=torch.load("./model_3500.pt")   #加载训练好的模型
    model.load_state_dict(weights['model_state_dict'],strict=False)     #调用模型权重
    model.eval()    #转化为评估模式(只调用模型，不训练模型)
    model = model.to(mydiv)     #将模型储存在GPU上
    
    #以下用于在运动开始前让狗子从趴下变成站着
    rate0=rospy.Rate(1000)    #站立命令发布频率为1000hz
    targetpos=[-1.3,0.0,0.67,-1.3,0.0,0.67,-1.3,0.0,0.67,-1.3,0.0,0.67]     #当狗子站立时各个关节的位置，即狗子关节运动的目标(gazebo顺序)
    kkp=[180,300,180,180,300,180,180,300,180,180,300,180]   #kp为关节刚性，kd为阻尼系数，具体可以查阅电机控制相关资料
    kkd=[8,15,8,8,15,8,8,15,8,8,15,8]
    rate0.sleep()   #休息一点时间让程序接收到话题的消息，不然可能会报错
    startpos=j_pos[:]   #狗子初始关节位置
    duaration=1000
    leg_msg=unm.LowCmd()    #储存将要发布的12个电机指令，将来由puber数组发布话题
    for i in range(12):     #初始化
        leg_msg.motorCmd[i].mode = 10
        leg_msg.motorCmd[i].Kp = 180
        leg_msg.motorCmd[i].Kd = 8
    percent=float(0)
    for i in range(3000):   #给狗子站起来的时间大约是3s
        percent+=float(1/duaration)     #判断狗子站立进度，狗子并不是一下直接站立，而是一个阶段一个阶段发布指令
        if percent>1:
            percent=1
        for j in range(12):
            leg_msg.motorCmd[j].q=(1 - percent)*startpos[j] + percent*targetpos[dof_map[j]]     #计算当前时刻狗子应该运行到哪个位置
            puber[dof_map[j]].publish(leg_msg.motorCmd[j])
        rate0.sleep()   #程序休眠，给关节运行的时间

    rate=rospy.Rate(50)
    actions=torch.tensor([0,0,0,0,0,0,0,0,0,0,0,0],device=mydiv).unsqueeze(0)   #obs元素之一，本来应记录上一时刻模型输出，循环开始前初始化为0
    rqt_plus=[-1.5,0.1,0.8,-1.5,-0.1,0.8,-1.5,0.1,1,-1.5,-0.1,1]    #给狗子关节的偏置，gazebo顺序
    model_plus=torch.tensor(list(rqt_plus[dof_map[i]] for i in range(0,12)),device=mydiv).unsqueeze(0)      #同样是偏置，model顺序
    for i in range(12): #运行的参数与站立的参数不同
        leg_msg.motorCmd[i].mode = 10
        leg_msg.motorCmd[i].Kp = 40
        leg_msg.motorCmd[i].Kd = 0.5

    while(not rospy.is_shutdown()):     #程序在人为退出之前会一直运行
        
        base_ang_vel=torch.tensor([ang.x,ang.y,ang.z],device=mydiv).unsqueeze(0)    #obs元素之一，狗子角速度
        quaternion=torch.tensor([ori.x,ori.y,ori.z,ori.w],device=mydiv).unsqueeze(0)    #狗子的四元数
        projected_gravity = quat_rotate_inverse(quaternion)     #obs元素之一
        joint_pos=torch.tensor(j_pos,device=mydiv).unsqueeze(0)-model_plus     #obs元素之一
        joint_vec=torch.tensor(j_vec,device=mydiv).unsqueeze(0)     #obs元素之一
        obs=torch.cat([base_ang_vel,projected_gravity,velocity_commands,joint_pos,joint_vec,actions],dim=-1)    #将所有元素整合为一个obs
        actions=model(obs)  #actions记录模型输出，成为下一时刻模型输入之一
        output=actions[:]*0.25+model_plus   #乘0.25是因为模型输出大了四倍，为了更好的微分
        #print(output)   #只是为了测试
        for i in range(12): #发布命令
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