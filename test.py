import torch
import torch.nn as nn

data_path = r'.\model_3500.pt'
weight = torch.load(data_path)


class Critic(nn.Module):
    def __init__(self):
        super().__init__()
        self.critic_layers = []
        self.critic_layers.append(nn.Linear(45, 128, bias=True))
        self.critic_layers.append(nn.ELU(alpha=1.0))
        for l in range(3):
            if l == 2:
                self.critic_layers.append(nn.Linear(128, 1, bias=True))
            else:
                self.critic_layers.append(nn.Linear(128, 128, bias=True))
                self.critic_layers.append(nn.ELU(alpha=1.0))
        self.critic = nn.Sequential(*self.critic_layers)

    def forward(self,x):
        x = self.critic(x)
        return x
    def p(self):
        print('Critic MLP:')
        print(self.critic)
class Actor(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor_layers = []
        self.actor_layers.append(nn.Linear(in_features=45,out_features=128,bias=True))
        self.actor_layers.append(nn.ELU(alpha=1.0))
        self.actor_layers.append(nn.Linear(in_features=128, out_features=128, bias=True))
        self.actor_layers.append(nn.ELU(alpha=1.0))
        self.actor_layers.append(nn.Linear(in_features=128, out_features=128, bias=True))
        self.actor_layers.append(nn.ELU(alpha=1.0))
        self.actor_layers.append((nn.Linear(in_features=128,out_features=12,bias=True)))
        self.actor = nn.Sequential(*self.actor_layers)
    def p(self):
        print('Actor MLP:')
        print(self.actor)
actor = Actor()
actor.load_state_dict(weight,strict=False)
hi3kirby = Critic()
hi3kirby.load_state_dict(weight,strict=False)
print(hi3kirby)
print(actor)



