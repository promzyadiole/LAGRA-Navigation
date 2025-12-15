import torch
from transformers import CLIPProcessor, CLIPModel

class CLIPVerifier:
    def __init__(self, model_name="openai/clip-vit-base-patch32", device=None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = CLIPProcessor.from_pretrained(model_name)
        self.model = CLIPModel.from_pretrained(model_name).to(self.device)

    @torch.no_grad()
    def score(self, pil_image, text: str) -> float:
        inputs = self.processor(text=[text], images=pil_image, return_tensors="pt", padding=True).to(self.device)
        outputs = self.model(**inputs)
        # logits_per_image: shape [1,1]
        return outputs.logits_per_image[0, 0].item()
