import time
import mujoco
import mujoco.viewer
import numpy as np

m = mujoco.MjModel.from_xml_path('robots/summit_xl_description/summit_xls.xml')
d = mujoco.MjData(m)

mobile_kv = 200.0

mobile_dot = np.zeros(4)
target_vel_list = np.array([[1,1,1,1],
                            [1.5,-1.5,-1.5,1.5],
                            [-1,-1,-1,-1],
                            [-1.5,1.5,1.5,-1.5]])
command = np.zeros(4)
t=0
delay = 1000
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    
    # Contact display
    # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1

    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)
        
        # get current wheel velocity
        mobile_dot[0] = d.qvel[19] # R1_front_left_wheel_rolling_joint
        mobile_dot[1] = d.qvel[6]  # R1_front_right_wheel_rolling_joint
        mobile_dot[2] = d.qvel[45] # R1_back_left_wheel_rolling_joint
        mobile_dot[3] = d.qvel[32] # R1_back_right_wheel_rolling_joint

        if t < delay:
            target_vel = np.zeros(4)
        elif t < delay+4000:
            target_vel = target_vel_list[0,:]
        elif t < delay+8000:
            target_vel = target_vel_list[1,:]
        elif t < delay+12000:
            target_vel = target_vel_list[2,:]
        elif t < delay+16000:
            target_vel = target_vel_list[3,:]
        else:
            target_vel = np.zeros(4)

        command = (target_vel - mobile_dot) * mobile_kv
        

        d.ctrl[0] = command[1]
        d.ctrl[1] = command[0]
        d.ctrl[2] = command[3]
        d.ctrl[3] = command[2]

        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
        if t % 100 == 0:
            print("current velocity:")
            print(mobile_dot)
            print("control force:")
            print(command)
            print("\n\n")
        t+=1
