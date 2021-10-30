#!python3
# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 8/5/20
# Description:
# This program manages the AI returning results back from opencv and tflite

class AI:
    def __init__(self, model_dir, show_images=False):
        self.detector = obj_det.Detector(model_dir, True) #1. model dir, 2. show images
        self.results = None

    def process_image(self, target):
        self.results = self.detector.process_image(target)

    def print_results(self):
        print(self.results)

    def get_target_data(self):
        # Lateral Distance MM, Distance MM, OffCenterX, OffCenterY, Foundtarget
        return self.results[0], self.results[1], self.results[2], self.results[3], self.results[4]

    def terminate(self):
        self.detector.finalize()
        return True
