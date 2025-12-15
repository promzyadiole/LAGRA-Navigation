from transformers import Blip2Processor, Blip2ForConditionalGeneration
import torch
from PIL import Image

class BLIP2Captioner:
    def __init__(self, model_name="Salesforce/blip2-flan-t5-xl", device=None):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = Blip2Processor.from_pretrained(model_name)
        self.model = Blip2ForConditionalGeneration.from_pretrained(
            model_name,
            torch_dtype=torch.float16 if self.device == "cuda" else torch.float32
        ).to(self.device)

    @torch.no_grad()
    def caption(self, pil_image: Image.Image, prompt="Describe the scene briefly."):
        inputs = self.processor(images=pil_image, text=prompt, return_tensors="pt").to(self.device)
        out = self.model.generate(**inputs, max_new_tokens=40)
        return self.processor.decode(out[0], skip_special_tokens=True).strip()
