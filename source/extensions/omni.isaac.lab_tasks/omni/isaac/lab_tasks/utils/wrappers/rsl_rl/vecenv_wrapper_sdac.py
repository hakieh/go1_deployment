# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Wrapper to configure a :class:`ManagerBasedRLEnv` or :class:`DirectRlEnv` instance to RSL-RL vectorized environment.

The following example shows how to wrap an environment for RSL-RL:

.. code-block:: python

    from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper

    env = RslRlVecEnvWrapper(env)

"""


import gymnasium as gym
import torch

from rsl_rl.env import VecEnv

from omni.isaac.lab.envs import DirectRLEnv, ManagerBasedRLEnv

class MemoryTracker:
    def __init__(self):
        self.initial_memory = torch.cuda.memory_allocated()
        self.obs = None
        self.rew = None
        self.done = None
        self.c = None
        self.truncated = None
        self.terminated = None

    def checkpoint(self):
        current_memory = torch.cuda.memory_allocated()
        delta_memory = current_memory - self.initial_memory
        self.initial_memory = current_memory
        # return delta_memory / (1024**2)
        return current_memory / (1024**2)

class SDACVecEnvWrapper(VecEnv):
    """Wraps around Isaac Lab environment for RSL-RL library

    To use asymmetric actor-critic, the environment instance must have the attributes :attr:`num_privileged_obs` (int).
    This is used by the learning agent to allocate buffers in the trajectory memory. Additionally, the returned
    observations should have the key "critic" which corresponds to the privileged observations. Since this is
    optional for some environments, the wrapper checks if these attributes exist. If they don't then the wrapper
    defaults to zero as number of privileged observations.

    .. caution::

        This class must be the last wrapper in the wrapper chain. This is because the wrapper does not follow
        the :class:`gym.Wrapper` interface. Any subsequent wrappers will need to be modified to work with this
        wrapper.

    Reference:
        https://github.com/leggedrobotics/rsl_rl/blob/master/rsl_rl/env/vec_env.py
    """

    def __init__(self, env: ManagerBasedRLEnv | DirectRLEnv):
        """Initializes the wrapper.

        Note:
            The wrapper calls :meth:`reset` at the start since the RSL-RL runner does not call reset.

        Args:
            env: The environment to wrap around.

        Raises:
            ValueError: When the environment is not an instance of :class:`ManagerBasedRLEnv` or :class:`DirectRLEnv`.
        """
        # check that input is valid
        if not isinstance(env.unwrapped, ManagerBasedRLEnv) and not isinstance(env.unwrapped, DirectRLEnv):
            raise ValueError(
                "The environment must be inherited from ManagerBasedRLEnv or DirectRLEnv. Environment type:"
                f" {type(env)}"
            )
        # initialize the wrapper
        self.env = env
        # store information required by wrapper
        self.num_envs = self.unwrapped.num_envs
        self.device = self.unwrapped.device
        self.max_episode_length = self.unwrapped.max_episode_length
        self.state_dim = self.unwrapped.observation_manager.group_obs_dim["policy"][0]
        self.action_dim = 12
        self.action_atoms = 51
        self.path = '.'
        self.name = 'go1'
        if hasattr(self.unwrapped, "action_manager"):
            self.num_actions = self.unwrapped.action_manager.total_action_dim
        else:
            self.num_actions = gym.spaces.flatdim(self.unwrapped.single_action_space)
        if hasattr(self.unwrapped, "observation_manager"):
            self.num_obs = self.unwrapped.observation_manager.group_obs_dim["policy"][0]
        else:
            self.num_obs = gym.spaces.flatdim(self.unwrapped.single_observation_space["policy"])
        # -- privileged observations
        if (
            hasattr(self.unwrapped, "observation_manager")
            and "critic" in self.unwrapped.observation_manager.group_obs_dim
        ):
            self.num_privileged_obs = self.unwrapped.observation_manager.group_obs_dim["critic"][0]
        elif hasattr(self.unwrapped, "num_states") and "critic" in self.unwrapped.single_observation_space:
            self.num_privileged_obs = gym.spaces.flatdim(self.unwrapped.single_observation_space["critic"])
        else:
            self.num_privileged_obs = 0
        # reset at the start since the RSL-RL runner does not call reset
        self.env.reset()
        self.tracker = MemoryTracker()
    def __str__(self):
        """Returns the wrapper name and the :attr:`env` representation string."""
        return f"<{type(self).__name__}{self.env}>"

    def __repr__(self):
        """Returns the string representation of the wrapper."""
        return str(self)

    """
    Properties -- Gym.Wrapper
    """

    @property
    def cfg(self) -> object:
        """Returns the configuration class instance of the environment."""
        return self.unwrapped.cfg

    @property
    def render_mode(self) -> str | None:
        """Returns the :attr:`Env` :attr:`render_mode`."""
        return self.env.render_mode

    @property
    def observation_space(self) -> gym.Space:
        """Returns the :attr:`Env` :attr:`observation_space`."""
        return self.env.observation_space

    @property
    def action_space(self) -> gym.Space:
        """Returns the :attr:`Env` :attr:`action_space`."""
        return self.env.action_space

    @classmethod
    def class_name(cls) -> str:
        """Returns the class name of the wrapper."""
        return cls.__name__

    @property
    def unwrapped(self) -> ManagerBasedRLEnv | DirectRLEnv:
        """Returns the base environment of the wrapper.

        This will be the bare :class:`gymnasium.Env` environment, underneath all layers of wrappers.
        """
        return self.env.unwrapped

    """
    Properties
    """

    def get_observations(self) -> tuple[torch.Tensor, dict]:
        """Returns the current observations of the environment."""
        if hasattr(self.unwrapped, "observation_manager"):
            obs_dict = self.unwrapped.observation_manager.compute()
        else:
            obs_dict = self.unwrapped._get_observations()
        return obs_dict["policy"], {"observations": obs_dict}

    @property
    def episode_length_buf(self) -> torch.Tensor:
        """The episode length buffer."""
        return self.unwrapped.episode_length_buf

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor):
        """Set the episode length buffer.

        Note:
            This is needed to perform random initialization of episode lengths in RSL-RL.
        """
        self.unwrapped.episode_length_buf = value

    """
    Operations - MDP
    """

    def seed(self, seed: int = -1) -> int:  # noqa: D102
        return self.unwrapped.seed(seed)

    def reset(self) -> tuple[torch.Tensor, dict]:  # noqa: D102
        # reset the environment
        obs_dict, _ = self.env.reset()
        print("====================")
        # return observations
        return obs_dict["policy"] #, {"observations": obs_dict}

    def step(self, actions: torch.Tensor) :
        # record step information
        # actions = torch.clip(actions,-5,5)
        print(f"Memory used for x: {self.tracker.checkpoint():.2f} MB","step start")
        obs_dict, self.rew, self.terminated, self.truncated, extras = self.env.step(self.to_foat(actions))
        
        # extras = None
        # print(obs_dict.device())
        # print(f"Memory used for x: {self.tracker.checkpoint():.2f} MB")
        # compute dones for compatibility with RSL-RL
        self.dones = (self.terminated | self.truncated).to(dtype=torch.long)
        # move extra observations to the extras dict
        self.obs = obs_dict["policy"]
        del obs_dict
        del extras
        # extras["observations"] = obs_dict
        # move time out information to the extras dict
        # this is only needed for infinite horizon tasks
        # if not self.unwrapped.cfg.is_finite_horizon:
        #     extras["time_outs"] = truncated

        # return the step information
        print(f"Memory used for x: {self.tracker.checkpoint():.2f} MB","start end")
        return self.obs.detach(), self.rew.detach(), self.dones.detach(), None

    def close(self):  # noqa: D102
        return self.env.close()
    
    def to_foat(self,actions):
        res = actions/(self.action_atoms-1)*6-3
        return res

