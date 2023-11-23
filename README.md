# mujoco_mecanum
- Mecamum wheel model (visual and collision) for Mujoco.
- Python3 based [macro](wheel_code_gen.py)
- Example code about SUMMIT-XL STEEL

# Requirement
- [mujoco_py](https://github.com/openai/mujoco-py)
- numpy (`pip install numpy`)
- argparse (`pip install argparse`)

# Usage
In [macro](wheel_code_gen.py) code, there are some parameters that can be customized according to user settings.
  1. link_name: link name of wheel
  2. n_roller: number of rollers of wheel
  3. pos: position of wheel wrt body frame
  4. mass: mass of a wheel
  5. diag_inertia: diagonal inertia of a wheel [ixx, iyy, izz]
  6. size: size of a wheel [radius, height]
  7. type: type of a wheel (1 or 2)
  8. --link_name", type=str, default="mecanum", help="link name of a wheel")

```bash
python3 wheel_code_gen.py --link_name mecanum 
```

After running this code, you can get `wheel.xml`. Using this, you can implement collision model ofmecanum wheel.


# Example
SUMMIT XL STEEL model was used to implement mecanum wheel.

In this [code](robots/summit_xl_description/assets/summit_xls.urdf.xml), you can see how to use.

And you can simmulate this model.
```
python3 summit_test.py
```


# Notice
For using this model, you can not use velocity actuator like below.
```
<actuator>
  <velocity name="mecanum_wheel_joint" .../>
</actuator>
```
Rather than using velocity, use force based actuator.
```
<actuator>
  <motor name="mecanum_wheel_joint" .../>
</actuator>
```
And in controller code, you can control wheel by PID controller like this [code](summit_test).

# License
`mujoco mecanum`is releasd under MIT license. Please see the [LICENSE](LICENSE) file for more information.
