[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/w3FoW0fO)
# EE260C_LAB1: Perception

Please refer to the instructions [here](https://docs.google.com/document/d/1BvQ9ztEvxDwsHv-RWEy2EOA7kdAonzdkbJIuQSB1nJI/edit?usp=sharing)

## Lab1 Doc Note
### Part2
1. Edit configuration to get needed sensors.
2. Read the [sensor documentation](https://www.google.com/url?q=https://www.google.com/url?q%3Dhttps://carla.readthedocs.io/en/latest/ref_sensors/%26amp;sa%3DD%26amp;source%3Deditors%26amp;ust%3D1729062475723795%26amp;usg%3DAOvVaw0lc21IGrJaOXsrzBuhuISd&sa=D&source=docs&ust=1729062475759264&usg=AOvVaw1bT1Okf9SXRL644xrRPyrC)
3. Setup one sensor first, and visualize those sensor in pygame window, add more sensors until the pipline works.
4. use `utils/transform.py` to transform points to another coordinate system.
#### Tasks
In this part,
- [ ] document your sensor setup, and give a decent reasoning behind it.
- [ ] Take a screenshot of the visualization of each and every sensor you use.
- [ ] Pick one sensor to visualize the groundtruth bounding boxes of the traffic participants.

### Part3
1. Select and Integrate a pre-trained model into detector.py.
2. use docker if necessary to solve dependency mismatches.
3. Firstly use online example data to test pre-trained model.
4. Understand input/output data form (tensor shape) of the model.
Manipulate the sensor data obtained from sensor_data argument, change it to be consistent with the model input format
Run inference using the model with the manipulated input
Visualize the output using the script written in Lab 0 and Part 2
Convert the output to the required format to get the correct AP value