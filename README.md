# snake_visualization
visualizes a 3-segment continuum robot by receiving configuration through udp or GUI input. Uses VTK 6.1, Qt 5.5.1, Eigen 3.2.1 <br />

The code should serve as a model to visualize a multi-bakbone continuum robot. Especially it is tailored for visualizing in real-time by recieving the configuration through UDP. A MATLAB file is provided (/send_udp_matlab/udp_send.slx) to test this feature. <br />

Important note: I used MATLAB Coder to convert kinematic functions written in MATLAB to C functions. This made my life easier
for my application of interest but came at the cost of challenges in case you want to change snake parameters such as segment lengths, radius, etc bacause some parameters such as number of segments disks are hardcoded at this point. <br />

![Alt text](image/snake_snapshot.PNG?raw=true "Snake visualization GUI")
