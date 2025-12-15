import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor

class SAMSegmenter:
    def __init__(self, checkpoint_path, model_type="vit_h", device=None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        sam = sam_model_registry[model_type](checkpoint=checkpoint_path)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)

    def segment_from_point(self, rgb_np: np.ndarray, point_xy):
        # rgb_np: HxWx3 RGB uint8
        self.predictor.set_image(rgb_np)
        input_point = np.array([point_xy])
        input_label = np.array([1])
        masks, scores, _ = self.predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=True,
        )
        best = int(np.argmax(scores))
        return masks[best], float(scores[best])

