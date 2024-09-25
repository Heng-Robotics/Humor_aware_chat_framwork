# coding:utf-8
import os
import torch
from lavis.models import load_model_and_preprocess

class CaptionConfig:
   _instance = None

   def __new__(cls, *args, **kwargs):
      if not cls._instance:
         cls._instance = super(CaptionConfig, cls).__new__(cls, *args, **kwargs)
      return cls._instance
   
   def __init__(self):
      
      if not hasattr(self, 'initialized'):
         self.initialized = True
         os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
         # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
         self.device = torch.device( "cpu")


         self.model, self.vis_processors, _ = load_model_and_preprocess(
            name="blip_caption", 
            model_type="base_coco", 
            is_eval=True, device=self.device)

Caption_config = CaptionConfig()
