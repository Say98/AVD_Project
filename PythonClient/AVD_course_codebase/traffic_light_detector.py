import os
import json
import cv2
import numpy as np
from enum import Enum
from traffic_light_detection_module.yolo import YOLO
from traffic_light_detection_module.postprocessing import draw_boxes

###############################################################################
# DECLARATION OF TRAFFIC LIGHT STATES
###############################################################################
class TrafficLightState(Enum):
    GO = 0
    STOP = 1
    NO_TL = -1

###############################################################################
# DECLARATION OF MAIN CLASS FOR THE DETECTOR
###############################################################################
class TrafficLightDetector(YOLO):
    """
    This class allows to use the Traffic-Light-Detection module in order
    to detect traffic lights in Carla.
    """
    # Minimum thresholds to refuse false positives
    MAX_TH1 = 0.20
    MAX_TH2 = 0.35

    # Minimum number of frames in which output has to be stable in order to change the tl state
    MIN_STOP_FRAMES = 5     # Min frames to choice STOP signal 
    MIN_GO_FRAMES = 10      # Min frames to choice GO signal
    MIN_NOTL_FRAMES = 10    # Min frames to choice NO_TL signal
    MIN_INDECISION_FRAMES = 30  # Min frames to understand that the state predicted is not reliable 

    def __init__(self):
        # recover detector configuration, changing the default path to h5 model
        with open('traffic_light_detection_module/config.json', 'r') as config_file:
            self.config = json.load(config_file)
            self.config['model']['saved_model_name']=os.path.join(os.path.realpath(os.path.dirname(__file__)),"traffic_light_detection_module","checkpoints","traffic-light-detection.h5")

        super().__init__(self.config) # initialize supeclass
        self.labels = self.config['model']['classes'] # labels predictable by the detector
        self.model.load_weights(os.path.join(os.path.realpath(os.path.dirname(__file__)),
                                            'traffic_light_detection_module',
                                            'checkpoints', self.config['model']['saved_model_name'])) # recover best weights
        self._state_counter = 0 # counter for checking detector output stability
        self._indecision_counter = 0 # counter for checking detector output instability
        self._state = None      # last stable traffic light state
        self._new_state = None # current traffic light state

        self._current_boxes = [] # best boxes detected in the current frame, one for image -> format: [box1, box2]
        self._current_box = None # box associated to the current chosen detection -> format: {camera_name: box}

    def get_state(self):
        return self._state

    def preprocess_image(self, image, image_h, image_w):
        """this function prepares the image for being inferred by the detector
        Args:
            image: image to be inferred
            image_h: desidered height of the image
            image_w: desidered width of the image
        Returns:
            image: preprocessed image
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (image_h, image_w))
        image = image/255
        image = np.expand_dims(image, 0)

        return image

    def predict_image(self, image):
        """Predicts the bounding boxes of traffic lights from a cv2 image, properly preprocessed
        Args:
            image: cv2 image to be inferred
        Returns:
            boxes: list of BoundBox objects found by the detector in the given image
        """
        image = self.preprocess_image(image, self.image_h, self.image_w)
        boxes = super().predict_image(image)
        return boxes

    def draw_boxes(self, image, boxes):
        """Draws the detected traffic lights' boxes on a cv2 image
        Args:
            image: originale image used for the inferation
            boxes: list of BoundBox objects found by the detector in the given image
        Returns:
            image: originale image with the boxes on it
        """
        return draw_boxes(image, boxes, self.labels)
         
    def light_state(self, boxes):
        """Filters all the detected boxes, discarding that with a score less than or equal to MIN_TH 
        and selecting the one with greater area that means the one nearest to ego vehicle.
        Args:
            boxes (List[BoundBox]): list of Bounding boxes predicted by the detector on a certain image
        
        Returns:
            [tlstate, score]: the predicted label with the relative score
        """

        # if no boxes are found, we haven't a traffic light
        if len(boxes) == 0: 
            self._current_boxes.append([])
            return TrafficLightState.NO_TL, 0

        # else process the box found

        box = boxes[0]
        self._current_boxes.append(box) # save current best detection on one of the two RGB captured images

        # return GO or STOP state with relative accuracy
        if box.get_label() == TrafficLightState.GO.value: return TrafficLightState.GO, box.get_score()

        return TrafficLightState.STOP, box.get_score()

    def update_state(self, boxes1, boxes2):
        """this function uses an heiristic in order to decide the current state of the traffic light
        Args:
            boxes1 (List[BoundBox]): list of Bounding boxes predicted by the detector on the image captured by the front camera
            boxes2 (List[BoundBox]): list of Bounding boxes predicted by the detector on the image captured by the lateral camera
        Returns:
            current state of the traffic light
        """

        # recover the best prediction for each image
        light_state1, light_accuracy1 = self.light_state(boxes1)
        light_state2, light_accuracy2 = self.light_state(boxes2)

        # HEURISTIC: if the prediction on the lateral camera has a confidence greater than MAX_TH2, 
        # use it as current traffic light state; otherwise check if the prediction on the other image
        # has a confidence greater than MAX_TH1, in this case use it as the state otherwise set state 
        # to NO_TL because detector accuracy is too low.
        if round(light_accuracy2, 2) > self.MAX_TH2:
            current_state = light_state2
            current_box = {"CameraDEPTH_TL": self._current_boxes[1]}
        else:
            if round(light_accuracy1, 2) > self.MAX_TH1:
                current_state = light_state1
                current_box = {"CameraDEPTH_FRONT": self._current_boxes[0]}
            else:
                current_state = TrafficLightState.NO_TL
                current_box = None

        self._current_boxes.clear()

        # first initialization
        if self._state is None:
            self._state = TrafficLightState.NO_TL
            self._new_state = TrafficLightState.NO_TL

        # if current state is different from the previous one, we are in a indecision state
        if self._new_state != current_state:
            self._indecision_counter += 1
            self._state_counter = 0
            self._new_state = current_state
        else: # update stable state counter
            self._state_counter += 1
            self._new_state = current_state

        # if the state has changed MIN_INDECISION_FRAMES times, set to NO_TL because we can't decide...
        if self._indecision_counter >= self.MIN_INDECISION_FRAMES:
            print(f"Cannot decide TL state. Defaulting to NO_TL...")
            self._state = TrafficLightState.NO_TL
            self._indecision_counter = 0
        # ... otherwise set the right state
        elif ((self._new_state == TrafficLightState.NO_TL and self._state_counter >= self.MIN_NOTL_FRAMES) or
              (self._new_state == TrafficLightState.GO and self._state_counter >= self.MIN_GO_FRAMES) or
              (self._new_state == TrafficLightState.STOP and self._state_counter >= self.MIN_STOP_FRAMES)):
            self._state = self._new_state
            self._state_counter = 0
            self._indecision_counter = 0
            self._current_box = current_box

        return self.get_state()
