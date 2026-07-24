import socket
import time
import pybullet as p
import pybullet_data
import math


# UDP Connection
QNX_Ip = "192.168.126.135"
port = 5000
UPDATE_HZ = 20
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
drone_id = p.loadURDF("sphere_1cm.urdf", [0,0,1],globalScaling=10)
#p.loadURDF("cube.urdf", [3, 0, 0.5], globalScaling=5)

# Add this BEFORE the while loop
obstacle_positions = [[3, 2, 0.0], [4, -3, 0.0], [-2, 4, 0.0]]
for pos in obstacle_positions:
    p.loadURDF("cube.urdf", pos, globalScaling=5, useFixedBase = True)

print(f"Streaming PyBullet telemetry to {QNX_Ip}:{port}")

try:
    while True:
        pos, ori = p.getBasePositionAndOrientation(drone_id)
        euler = p.getEulerFromQuaternion(ori)
        yaw = euler[2]

        #automata
        t = time.time()
        new_x = 3 * math.sin(t*0.5)
        new_y = 3 * math.cos(t*0.5)
        new_yaw = t * 0.5

        ori2 = p.getQuaternionFromEuler([0, 0, new_yaw])
        p.resetBasePositionAndOrientation(drone_id, [new_x, new_y, 1], ori)
        pos = [new_x, new_y, 1.0]
        yaw = new_yaw

        # Lidar
        ray_end = [
            pos[0] + 10 * math.cos(yaw),
            pos[1] + 10 * math.sin(yaw),
            pos[2]
        ]
        ray_result = p.rayTest(pos, ray_end)

        # Draw a line from pos to ray_end. Red if no hit, Green if it hits something.
        hit = ray_result[0][0]
        p.addUserDebugLine(pos, ray_end, [1, 0, 0] if hit == -1 else [0, 1, 0], lifeTime=0.1)

        hit_dist = ray_result[0][2] * 10.0

        # Data packet
        msg = f"{hit_dist:.2f}, {pos[0]:.2f}, {pos[1]:.2f}"

        #Transmission
        sock.sendto(msg.encode('utf-8'), (QNX_Ip,port))

        p.stepSimulation()
        time.sleep(1.0/UPDATE_HZ)
except KeyboardInterrupt:
    p.disconnect()
    print("\nSimulation Stopped.")
