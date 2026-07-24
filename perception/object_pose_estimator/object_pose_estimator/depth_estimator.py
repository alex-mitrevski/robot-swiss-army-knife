import numpy as np
from PIL import Image
from transformers import pipeline as hf_pipeline


class DepthEstimator:
    def __init__(
        self,
        model_id: str = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
        device: str = "cpu",
    ):
        self._pipe = hf_pipeline(
            task="depth-estimation",
            model=model_id,
            device=device,
        )

    def estimate(self, image: np.ndarray) -> np.ndarray:
        """Return metric depth map (H, W) float32 in metres."""
        pil = Image.fromarray(image)
        result = self._pipe(pil)
        depth = result["predicted_depth"].squeeze().numpy().astype(np.float32)
        return depth
