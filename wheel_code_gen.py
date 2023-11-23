from math import pi, sin, cos 
import numpy as np
import argparse

roller_ratio = 0.08 / 0.127 # radius ratio of roller wrt wheel

def main(args):

    total_desc = ''

    x = args.pos[0]
    y = args.pos[1]
    z = args.pos[2]

    r = args.size[0]
    h = args.size[1]

    cylinder = f"""<body name="{args.link_name}_wheel_intermediate_link" pos="{str(x)} {str(y)} {str(z)}">
    <inertial pos="0 0 0" quat="0.707107 0 0 0.707107" mass="6.5" diaginertia="0.0524193 0.0303095 0.0303095" />
    <joint name="R1_{args.link_name}_wheel_rolling_joint" pos="0 0 0" axis="0 1 0" />
    <geom size="{str(r)} {str(h)}" quat="0.707107 0.707107 0 0" type="cylinder" rgba="0.2 0.2 0.2 0.5" contype="1" conaffinity="0"/>
    """
    total_desc += cylinder + '\n'

    step = (2*pi) / args.n_roller
    roller_r = r * roller_ratio

    for i in range(args.n_roller):
        body_name = args.link_name + '_roller_' + str(i)
        joint_name = args.link_name + '_slipping_' + str(i)

        pin_1 = np.array([(r - roller_r)*cos(step*i), -h/2, (r - roller_r)*sin(step*i)])

        if args.type == 0:
            if i == args.n_roller-1:
                pin_2 = np.array([(r - roller_r)*cos(step*0), h/2, (r - roller_r)*sin(step*0)])
            else:
                pin_2 = np.array([(r - roller_r)*cos(step*(i+1)), h/2, (r - roller_r)*sin(step*(i+1))])
        else:
            if i == 0:
                pin_2 = np.array([(r - roller_r)*cos(step*(args.n_roller-1)), h/2, (r - roller_r)*sin(step*(args.n_roller-1))])
            else:
                pin_2 = np.array([(r - roller_r)*cos(step*(i-1)), h/2, (r - roller_r)*sin(step*(i-1))])


        axis = pin_2 - pin_1
        pos = pin_1 + axis/2

        wheel = f"""
    <body name="{body_name}_link" pos="{str(pos[0])} {str(pos[1])} {str(pos[2])}">
        <joint name="{joint_name}_joint" type="hinge" pos="0 0 0" axis="{str(axis[0])} {str(axis[1])} {str(axis[2])}" damping="0.1" limited="false" actuatorfrclimited="false"/>
        <inertial pos="0 0 0" quat="0.711549 0.711549 0 0 " mass="0.001" diaginertia="0.00001 0.00001 0.00001" />
        <geom size="{str(r)}" quat="1 0 0 0" type="sphere" rgba="0.2 0.2 0.2 1" contype="1" conaffinity="0"/>
    </body>"""

        total_desc += wheel + '\n' 
    total_desc += '</body>' + '\n'

    open('wheel.xml','w').write(total_desc)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--link_name", type=str, default="mecanum", help="link name of a wheel")
    parser.add_argument("--n_roller", type=int, default=12, help="number of rollers of wheel")
    parser.add_argument("--pos", type=float, nargs=3, default=[0, 0, 0], help="position of wheel wrt body frame")
    parser.add_argument("--mass", type=float, default=6.5, help="mass of a wheel")
    parser.add_argument("--diag_inertia", type=float, nargs=3, default=[0.0524193, 0.0303095, 0.0303095], help="diagonal inertia of a wheel [ixx, iyy, izz]")
    parser.add_argument("--size", type=float, nargs=2, default=[0.120, 0.0435], help="size of a wheel [radius, height]")
    parser.add_argument("--type", type=int, default=1, help="type of a wheel (1 or 2)")

    
    args = parser.parse_args()
    main(args)