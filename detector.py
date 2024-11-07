from pprint import pprint
import os
import numpy as np
from copy import deepcopy
from utils.inferencer import inferencer
from mmdet3d.structures.bbox_3d import LiDARInstance3DBoxes

class Detector:
    def __init__(self):
        # Add your initialization logic here
        self.inferencer = inferencer
        # result = inferencer(**call_args)

    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the detector. The location is defined with respect to the actor center
        -- x axis is longitudinal (forward-backward)
        -- y axis is lateral (left and right)
        -- z axis is vertical
        Unit is in meters

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]

        """
        sensors = [
            # a camera that looks at mid front
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
            # {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            #  'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            # {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            #  'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.0, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'range': 50,
             'rotation_frequency': 20, 'channels': 64,
             'upper_fov': 4, 'lower_fov': -20, 'points_per_second': 2304000,
             'id': 'LIDAR'},

            # {'type': 'sensor.other.gnss', 'x': 0.7,
            #     'y': -0.4, 'z': 1.60, 'id': 'GPS'}
        ]

        return sensors

    def detect(self, sensor_data):
        """
        Add your detection logic here
            Input: sensor_data, a dictionary containing all sensor data. Key: sensor id. Value: tuple of frame id and data. For example
                'Right' : (frame_id, numpy.ndarray)
                    The RGBA image, shape (H, W, 4)
                'Left' : (frame_id, numpy.ndarray)
                    The RGBA image, shape (H, W, 4)
                'LIDAR' : (frame_id, numpy.ndarray)
                    The lidar data, shape (N, 4)
            Output: a dictionary of detected objects in global coordinates
                det_boxes : numpy.ndarray
                    The detection bounding box, shape (N, 8, 3) or (N, 4, 2).
                det_class : numpy.ndarray
                    The object class for each predicted bounding box, shape (N, 1) corresponding to the above bounding box. 
                    0 for vehicle, 1 for pedestrian, 2 for cyclist.
                det_score : numpy.ndarray
                    The confidence score for each predicted bounding box, shape (N, 1) corresponding to the above bounding box.
        """
        input_data = sensor_data['LIDAR'][1]
        input_data[:,1]= -input_data[:,1]
        result = self.inferencer({'points': input_data})
        det_boxes = LiDARInstance3DBoxes(result['predictions'][0]['bboxes_3d']).corners
        det_boxes = det_boxes.cpu().numpy().reshape(-1, 8, 3)

        if len(det_boxes) == 0:
            return {
                'det_boxes': np.array([]),
                'det_class': np.array([]),
                'det_score': np.array([])
            }
        else:
            det_boxes[:,:,1] = -det_boxes[:,:,1]
            return {
                'det_boxes': det_boxes,
                'det_class': np.array(result['predictions'][0]['labels_3d']),
                'det_score': np.array(result['predictions'][0]['scores_3d'])
            }