from lib import rapinterface as ri, dynamic_window_approach as dw
import matplotlib.pyplot as plt
import numpy as np
import math

sensor_pos = [(0.10638160980,-0.1381410850,0.06811071560),(0.15552160980,-0.1202410850,0.06811871560),(0.19054160980,-0.0785010850,0.06811471560),(0.20916160980,-0.0272730850,0.06810771560),(0.20916160980,0.0272739150,0.06810371560),(0.19058160980,0.0831339150,0.06810771560),(0.15540160980,0.1250489150,0.06811171560),(0.10638160980,0.1381989150,0.06811071560),(-0.11031839020,0.1381989150,0.06817271560),(-0.15956839020,0.1202689150,0.06816971560),(-0.19458839020,0.0784989150,0.06816771560),(-0.21320839020,0.0272729150,0.06816671560),(-0.21320839020,-0.0272760850,0.06816671560),(-0.19457839020,-0.0785010850,0.06816771560),(-0.15956839020,-0.1202410850,0.06816971560),(-0.11031839020,-0.1381510850,0.06817271560)]
sensor_ori = [270,310, 330, 350, 10, 30, 50, 90, 90, 130, 150, 170, 190, 210, 230, 270]
param = {}
config = dw.Config()

def rap_motion(u):
    wr = (u[0] + u[1]*config.c)/config.r
    wl = (u[0] - u[1]*config.c)/config.r

    ri.setvelocity(param['motorRight'], wr)
    ri.setvelocity(param['motorLeft'], wl)
    #ri.setvelocity(param['motorRight'], 0)
    #ri.setvelocity(param['motorLeft'], 0)

    ri.sleep(config.dt)
    position = ri.getposition(param['robot'],param['base'])
    orientation = ri.getorientation(param['robot'],param['base'])
    x = [0]*5
    x[0],x[1],x[2] = position[0],position[1], orientation[2]
    x[3], x[4] = u

    return x        
                

def init():
    param['target'] = ri.getobject("Target")
    param['base'] = ri.getobject("base")
    param['motorLeft'] = ri.getobject("Pioneer_p3dx_leftMotor")
    param['motorRight'] = ri.getobject("Pioneer_p3dx_rightMotor")
    param['robot'] = ri.getobject("Pioneer_p3dx")
    param['sonic'] = ri.getcollection("sonic")
    
    for i in range(16):
        sensor_ori[i] = sensor_ori[i]*math.pi/180
    
    position = ri.getpositionb(param['robot'],param['base'])
    orientation = ri.getorientationb(param['robot'],param['base'])
    goal = ri.getpositionb(param['target'],param['base'])

    robot_type = dw.RobotType.circle

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([position[0], position[1], orientation[2], 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([goal[0], goal[1]])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    config.goal = goal
    param['trajectory'] = np.array(x)
    param['x'] = x

    config.max_speed = 0.75  # [m/s]
    config.min_speed = -0.30  # [m/s]
    config.max_yaw_rate = 100.0 * math.pi / 180.0  # [rad/s]
    config.max_accel = 0.3  # [m/ss]
    config.max_delta_yaw_rate = 100.0 * math.pi / 180.0  # [rad/ss]
    config.v_resolution = 0.01  # [m/s]
    config.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
    config.dt = 0.1  # [s] Time tick for motion prediction
    config.predict_time = 0.5  # [s]
    config.to_goal_cost_gain = 0.15
    config.speed_cost_gain = 1.0
    config.obstacle_cost_gain = 1.0
    config.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
    # config.robot_type = dw.RobotType.circle
    # # Also used to check if goal is reached in both types
    config.robot_radius = 0.30  # [m] for collision check

    config.robot_type = dw.RobotType.rectangle
    # Also used to check if goal is reached in both types
    config.robot_width = 0.38  # [m] for collision check
    config.robot_length = 0.52  # [m] for collision check

    config.c = 0.19 # [m] distancia do centro às rodas
    config.r = 0.185# [m] raio da roda

ob = np.array([[0,0]])

def loop():
    x = param['x']
    u, predicted_trajectory = dw.dwa_control(x, config, config.goal, ob)
    #x = dw.motion(x,u,config.dt)
    x = rap_motion(u) # simulate robot
    if(x[0] == 0):
        return
    param['x'] = x
    param['trajectory'] = np.vstack((param['trajectory'], x))  # store state history

    if dw.show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(config.goal[0], config.goal[1], "xb")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        dw.plot_robot(x[0], x[1], x[2], config)
        dw.plot_arrow(x[0], x[1], x[2])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

    # check reaching goal
    dist_to_goal = math.hypot(x[0] - config.goal[0], x[1] - config.goal[1])
    if dist_to_goal <= config.robot_radius:
        print("Goal!!")
        rap_motion([0,0])
        return True
def finnaly():
    print("Done")
    if dw.show_animation:
        plt.plot(param['trajectory'][:, 0], param['trajectory'][:, 1], "-r")
        plt.pause(0.001)

    plt.show()

ri.start(init,loop,finnaly,30)