from transformers import DPTImageProcessor, DPTForDepthEstimation
import torch
import numpy as np
from PIL import Image


class DPTModel:
    """
    Dense Prediction Transformer (DPT) model trained on 1.4 million images for monocular depth estimation.
    It was introduced in the paper Vision Transformers for Dense Prediction by Ranftl et al. (2021) and
    first released in this repository. DPT uses the Vision Transformer (ViT) as backbone and adds a neck +
    head on top for monocular depth estimation.
    """

    def __init__(self):
        self._processor = DPTImageProcessor.from_pretrained("Intel/dpt-large")
        self._model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")

    """
    @param image - has shape (n, m, 3)
    """

    def predict(self, image: np.ndarray) -> np.ndarray:
        if len(image.shape) != 3 or image.shape[2] != 3:
            raise ValueError(
                f"The shape of the array is not valid. Expected shape: (n,m,{3}). Actual shape: {image.shape}")

        image = Image.fromarray(np.uint8(image))
        # prepare image for the model
        inputs = self._processor(images=image, return_tensors="pt")

        with torch.no_grad():
            outputs = self._model(**inputs)
            predicted_depth = outputs.predicted_depth

        # interpolate to original size
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=image.size[::-1],
            mode="bicubic",
            align_corners=False,
        )

        # postprocess
        output = prediction.squeeze().cpu().numpy()
        formatted = (output * 255 / np.max(output)).astype("uint8")
        return formatted
