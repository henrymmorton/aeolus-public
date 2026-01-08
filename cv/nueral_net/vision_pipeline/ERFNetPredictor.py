import os
import numpy as np
import torch
import torch_tensorrt
from torchvision.transforms import Resize, ToTensor
from PIL import Image
from core.cv.nueral_net.model.ERFNet import ERFNet
from core.constants import IMAGE_WIDTH, IMAGE_HEIGHT, NUM_CLASSES
import logging

logger =  logging.getLogger(__name__)

class ERFNetPredictor:

    def __init__(self, model_path):
        # The required input size for the model
        self.target_width = IMAGE_WIDTH
        self.target_height = IMAGE_HEIGHT

        # The desired true/false threshold for inference
        self.thresh = 0.85

        # Clear the cuda cache
        torch.cuda.empty_cache()

        # Initialize the ERFNet model
        model = torch.jit.load(model_path)

        # Move it to cuda
        model.to("cuda")
        logger.debug("ERFNET PREDICTOR: Loaded model into CUDA")

        # Puts the model in evaluation mode
        model.eval()

        # Save the model as part of the class
        self.model = model

    def resize(self, image):
        """
        Resizes an image to be compatible with the ERFNet instance

        param image: A image represented as a numpy array with 3 channels
        """
        image_width = image.shape[:2][1]
        image_height = image.shape[:2][0]
            
        logger.warning("ERFNET PREDICTOR: Resizing image")
        logger.warning(f"ERFNET PREDICTOR: Old image size {image.shape}")
        width_percent =  self.target_width / float(image_width)
        height_percent = self.target_height / float(image_height)
        new_width = int(image_width * max(width_percent, height_percent))
        new_height = int(image_height * max(width_percent, height_percent))
        logger.warning(f"ERFNET PREDICTOR: New image size: {new_width}, {new_height}")

        image = Image.fromarray(image)

        # Resize the image while preserving the center
        image = Resize((new_width, new_height), Image.BILINEAR)(image)
        
        # Crop the image to the target size
        left = (new_width - self.target_width) / 2
        top = (new_height - self.target_height) / 2
        right = (new_width + self.target_width) / 2
        bottom = (new_height + self.target_height) / 2
        image = image.crop((left, top, right, bottom))
        logger.warning(f"ERFNET PREDICTOR: cropped image size is {image.size}")

        image = np.array(image)
        
        return image
    
    def threshold_transform(self, x):
        """
        Transforms a prediction from an array of probabilities to booleans (0 or 255) based on a probability threshold

        param x: A numpy array of probabilities representing a prediction
        """
        thresholded = x > self.thresh

        return thresholded

    def predict_lane_lines(self, image):
        """
        Predicts lane lines in a image

        param image: A image represented as a numpy array with 3 channels
        """
        logger.info("ERFNET PREDICTOR: Starting prediction pipeline")

        image_width = image.shape[:2][1]
        image_height = image.shape[:2][0]
        if (image_width, image_height) != (self.target_width, self.target_height):
            image = self.resize(image)

        # Normalize to 1, permute to correct shape, convert to torch tensor, and move to cuda
        image_tensor = torch.from_numpy(image).float().permute(2, 0, 1).div(255).cuda(non_blocking=True).half()
        image_tensor = image_tensor[None, :]

        # Do inference
        with torch.inference_mode():
            outputs = self.model(image_tensor)
            softmax = torch.nn.Softmax(dim=1)
            outputs = self.threshold_transform(softmax(outputs))
        logger.debug("ERFNET PREDICTOR: Completed prediction")

        # Save as a numpy array
        outputs_cpu = outputs.cpu()
        outputs_cpu = outputs_cpu.numpy()[0]
        logger.info("ERFNET PREDICTOR: Finished prediction pipeline")

        return outputs_cpu

    def predict_lane_lines_from_stored_image(self, image_path):
        """
        Predicts lane lines from a stored image

        param image_path: The path to the desired image
        """
        if(not os.path.exists(image_path)):
            logger.error("Error: image could not be loaded")
            
        image = Image.open(image_path)

        self.predict_lane_lines(image)