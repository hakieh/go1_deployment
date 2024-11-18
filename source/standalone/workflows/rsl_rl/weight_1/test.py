import torch
import torch.nn as nn

class Actor(nn.Module):
    def __init__(self):
        super(Actor, self).__init__()
        self.actor_mlp = nn.Sequential(
            nn.Linear(45,128),
            nn.ELU(),
            nn.Linear(128,128),
            nn.ELU(),
            nn.Linear(128,128),
            nn.ELU(),
            nn.Linear(128,12),
        )
    
    def forward(self, x):
        return self.actor_mlp(x) 
    

mlp_model = Actor()

print("Actor MLP:",mlp_model.actor_mlp)


