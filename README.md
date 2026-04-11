For my senior design SOAR project sponsored by Lockheed Martin, I was tasked with creating and implementing two
human detection models. Usings thermal and rgb human detection datasets found online, such as POP and FLIR to name
a few, two detection models were created. The thermal model has a recall of 72% and precision of 93% and will use 
a FLIR thermal camera for data collect. The rgb model has a recall of 80% and precision of 87% and will use a EMeet
webcam for data collect. Both models will run on a NVIDIA Jetson Orin Nano at an average rate of 12 fps for thermal
and 15 fps for rgb. The weights for both models can be found within camera driver's weights folder. Additionally, 
I have been tasked with obstacle avoidance. The Orin Nano collects and packages the data received by the RPLiDAR 
C1 in the mini fence format and sent to the flight control via MAVLink. 

After the detection models have proccessed each frame, the resulting frame will be sent back to ground station via
WiFi. For proper streaming, ensure the ground station and Orin Nano are connected to the same Wifi Network. The 
command below can be used to connect the Orin to a new Wifi Network:

sudo nmcli device wifi connect "NETWORK_NAME" password "YOUR_PASSWORD"


Also ensure the Orin is streaming to the correct IP address within the Wifi Network. Within fusion visualizer
holds the IP address to which the stream is sent. Short-cut:

code sensor2_ws/src/sensor_fusion/sensor_fusion/fusion_visualizer.py
or 
vim sensor2_ws/src/sensor_fusion/sensor_fusion/fusion_visualizer.py

and adjust self.laptop_ip to the correct IP address and ensure the correct port number below (self.port)


If the IP address or port number was changed, return to sensor2_ws directory (cd ~/sensor2_ws) and do the following
commands:

colcon build --packages-select sensor_fusion
source install/setup.bash


Now depending on your application, you have the following functions at your disposal. 
1) ros2 launch sensor_fusion sensor_fusion.launch.py use_camera:=true camera_type:=thermal 
2) ros2 launch sensor_fusion sensor_fusion.launch.py use_camera:=true  (default is rgb)
3) ros2 launch sensor_fusion sensor_fusion.launch.py use_camera:=false (only obstacle avoidace LiDAR only)
