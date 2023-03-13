import time
import copy
from argparse import ArgumentParser

import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from torch.distributions import constraints
from torch.distributions.transforms import Transform
from torch.distributions import Normal, TransformedDistribution

from torch.nn.functional import softplus


import math 

from collections import deque, namedtuple
from numpy.random import default_rng


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

Transition = namedtuple('Transition', ('state', 'action', 'reward', 'nextstate', 'real_done'))



class MeanStdevFilter():
    def __init__(self, shape, clip=3.0):
        self.eps = 1e-4
        self.shape = shape
        self.clip = clip
        self._count = 0
        self._running_sum = np.zeros(shape)
        self._running_sum_sq = np.zeros(shape) + self.eps
        self.mean = np.zeros(shape)
        self.stdev = np.ones(shape) * self.eps

    def update(self, x):
        if len(x.shape) == 1:
            x = x.reshape(1,-1)
        self._running_sum += np.sum(x, axis=0)
        self._running_sum_sq += np.sum(np.square(x), axis=0)
        # assume 2D data
        self._count += x.shape[0]
        self.mean = self._running_sum / self._count
        self.stdev = np.sqrt(
            np.maximum(
                self._running_sum_sq / self._count - self.mean**2,
                 self.eps
                 ))
    
    def __call__(self, x):
        return np.clip(((x - self.mean) / self.stdev), -self.clip, self.clip)

    def invert(self, x):
        return (x * self.stdev) + self.mean

class ReplayPool:

    def __init__(self, action_dim, state_dim, capacity=1e6):
        self.capacity = int(capacity)
        self._action_dim = action_dim
        self._state_dim = state_dim
        self._pointer = 0
        self._size = 0
        self._init_memory()
        self._rng = default_rng()

    def _init_memory(self):
        self._memory = {
            'state': np.zeros((self.capacity, self._state_dim), dtype='float32'),
            'action': np.zeros((self.capacity, self._action_dim), dtype='float32'),
            'reward': np.zeros((self.capacity), dtype='float32'),
            'nextstate': np.zeros((self.capacity, self._state_dim), dtype='float32'),
            'real_done': np.zeros((self.capacity), dtype='bool')
        }

    def push(self, transition: Transition):

        # Handle 1-D Data
        num_samples = transition.state.shape[0] if len(transition.state.shape) > 1 else 1
        idx = np.arange(self._pointer, self._pointer + num_samples) % self.capacity

        for key, value in transition._asdict().items():
            self._memory[key][idx] = value

        self._pointer = (self._pointer + num_samples) % self.capacity
        self._size = min(self._size + num_samples, self.capacity)

    def _return_from_idx(self, idx):
        sample = {k: tuple(v[idx]) for k,v in self._memory.items()}
        return Transition(**sample)

    def sample(self, batch_size: int, unique: bool = True):
        idx = np.random.randint(0, self._size, batch_size) if not unique else self._rng.choice(self._size, size=batch_size, replace=False)
        return self._return_from_idx(idx)

    def sample_all(self):
        return self._return_from_idx(np.arange(0, self._size))

    def __len__(self):
        return self._size

    def clear_pool(self):
        self._init_memory()

    def initialise(self, old_pool):
        # Not Tested
        old_memory = old_pool.sample_all()
        for key in self._memory:
            self._memory[key] = np.append(self._memory[key], old_memory[key], 0)


# Taken from: https://github.com/pytorch/pytorch/pull/19785/files
# The composition of affine + sigmoid + affine transforms is unstable numerically
# tanh transform is (2 * sigmoid(2x) - 1)
# Old Code Below:
# transforms = [AffineTransform(loc=0, scale=2), SigmoidTransform(), AffineTransform(loc=-1, scale=2)]
class TanhTransform(Transform):
    r"""
    Transform via the mapping :math:`y = \tanh(x)`.
    It is equivalent to
    ```
    ComposeTransform([AffineTransform(0., 2.), SigmoidTransform(), AffineTransform(-1., 2.)])
    ```
    However this might not be numerically stable, thus it is recommended to use `TanhTransform`
    instead.
    Note that one should use `cache_size=1` when it comes to `NaN/Inf` values.
    """
    domain = constraints.real
    codomain = constraints.interval(-1.0, 1.0)
    bijective = True
    sign = +1

    @staticmethod
    def atanh(x):
        return 0.5 * (x.log1p() - (-x).log1p())

    def __eq__(self, other):
        return isinstance(other, TanhTransform)

    def _call(self, x):
        return x.tanh()

    def _inverse(self, y):
        # We do not clamp to the boundary here as it may degrade the performance of certain algorithms.
        # one should use `cache_size=1` instead
        return self.atanh(y)

    def log_abs_det_jacobian(self, x, y):
        # We use a formula that is more numerically stable, see details in the following link
        # https://github.com/tensorflow/probability/blob/master/tensorflow_probability/python/bijectors/tanh.py#L69-L80
        return 2. * (math.log(2.) - x - softplus(-2. * x))


class MLPNetwork(nn.Module):
    
    def __init__(self, input_dim, output_dim, hidden_size=256):
        super(MLPNetwork, self).__init__()
        self.network = nn.Sequential(
                        nn.Linear(input_dim, hidden_size),
                        nn.ReLU(),
                        nn.Linear(hidden_size, hidden_size),
                        nn.ReLU(),
                        nn.Linear(hidden_size, hidden_size),
                        nn.ReLU(),
                        nn.Linear(hidden_size, output_dim),
                        )
    
    def forward(self, x):
        return self.network(x)


class Policy(nn.Module):

    def __init__(self, state_dim, action_dim, hidden_size=256):
        super(Policy, self).__init__()
        self.action_dim = action_dim
        self.network = MLPNetwork(state_dim, action_dim * 2, hidden_size)

    def forward(self, x, get_logprob=False):
        mu_logstd = self.network(x)
        mu, logstd = mu_logstd.chunk(2, dim=1)
        logstd = torch.clamp(logstd, -20, 2)
        std = logstd.exp()
        dist = Normal(mu, std)
        transforms = [TanhTransform(cache_size=1)]
        dist = TransformedDistribution(dist, transforms)
        action = dist.rsample()
        if get_logprob:
            logprob = dist.log_prob(action).sum(axis=-1, keepdim=True)
        else:
            logprob = None
        mean = torch.tanh(mu)
        return action, logprob, mean


class DoubleQFunc(nn.Module):
    
    def __init__(self, state_dim, action_dim, hidden_size=256):
        super(DoubleQFunc, self).__init__()
        self.network1 = MLPNetwork(state_dim + action_dim, 1, hidden_size)
        self.network2 = MLPNetwork(state_dim + action_dim, 1, hidden_size)

    def forward(self, state, action):
        x = torch.cat((state, action), dim=1)
        return self.network1(x), self.network2(x)


def evaluate_agent(env, agent, state_filter, n_starts=1):
    reward_sum = 0
    for _ in range(n_starts):
        done = False
        truncated = False
        state, _ = env.reset()

        while (not done and not truncated ):

            action = agent.get_action( state, state_filter=state_filter, deterministic=True )
            

            nextstate, reward, done, truncated,  _ = env.step( [ action ] )
            reward_sum += reward
            state = nextstate

    return reward_sum / n_starts


class SAC_Agent:

    def __init__(self, seed, state_dim, action_dim, lr=3e-4, gamma=0.99, tau=5e-3, batchsize=256, hidden_size=256, update_interval=1, buffer_size=int(1e6), target_entropy=None):
        self.gamma = gamma
        self.tau = tau
        self.target_entropy = target_entropy if target_entropy else -action_dim
        self.batchsize = batchsize
        self.update_interval = update_interval

        torch.manual_seed(seed)

        # aka critic
        self.q_funcs = DoubleQFunc(state_dim, action_dim, hidden_size=hidden_size).to(device)
        self.target_q_funcs = copy.deepcopy(self.q_funcs)
        self.target_q_funcs.eval()
        for p in self.target_q_funcs.parameters():
            p.requires_grad = False

        # aka actor
        self.policy = Policy(state_dim, action_dim, hidden_size=hidden_size).to(device)

        # aka temperature
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)

        self.q_optimizer = torch.optim.Adam(self.q_funcs.parameters(), lr=lr)
        self.policy_optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr)
        self.temp_optimizer = torch.optim.Adam([self.log_alpha], lr=lr)

        self.replay_pool = ReplayPool(action_dim=action_dim, state_dim=state_dim, capacity=int(1e6))
    
    def get_action(self, state, state_filter=None, deterministic=False):
        if state_filter:
            state = state_filter(state)
        with torch.no_grad():
            action, _, mean = self.policy(torch.Tensor(state).view(1,-1).to(device))
        if deterministic:
            return mean.squeeze().cpu().numpy()
        return np.atleast_1d(action.squeeze().cpu().numpy())

    def update_target(self):
        """moving average update of target networks"""
        with torch.no_grad():
            for target_q_param, q_param in zip(self.target_q_funcs.parameters(), self.q_funcs.parameters()):
                target_q_param.data.copy_(self.tau * q_param.data + (1.0 - self.tau) * target_q_param.data)

    def update_q_functions(self, state_batch, action_batch, reward_batch, nextstate_batch, done_batch):
        with torch.no_grad():
            nextaction_batch, logprobs_batch, _ = self.policy(nextstate_batch, get_logprob=True)
            q_t1, q_t2 = self.target_q_funcs(nextstate_batch, nextaction_batch)
            # take min to mitigate positive bias in q-function training
            q_target = torch.min(q_t1, q_t2)
            value_target = reward_batch + (1.0 - done_batch) * self.gamma * (q_target - self.alpha * logprobs_batch)
        q_1, q_2 = self.q_funcs(state_batch, action_batch)
        loss_1 = F.mse_loss(q_1, value_target)
        loss_2 = F.mse_loss(q_2, value_target)
        return loss_1, loss_2

    def update_policy_and_temp(self, state_batch):
        action_batch, logprobs_batch, _ = self.policy(state_batch, get_logprob=True)
        q_b1, q_b2 = self.q_funcs(state_batch, action_batch)
        qval_batch = torch.min(q_b1, q_b2)
        policy_loss = (self.alpha * logprobs_batch - qval_batch).mean()
        temp_loss = -self.alpha * (logprobs_batch.detach() + self.target_entropy).mean()
        return policy_loss, temp_loss

    def optimize(self, n_updates, state_filter=None):
        q1_loss, q2_loss, pi_loss, a_loss = 0, 0, 0, 0
        for i in range(n_updates):
            samples = self.replay_pool.sample(self.batchsize)

            if state_filter:
                state_batch = torch.FloatTensor(state_filter(samples.state)).to(device)
                nextstate_batch = torch.FloatTensor(state_filter(samples.nextstate)).to(device)
            else:
                state_batch = torch.FloatTensor(samples.state).to(device)
                nextstate_batch = torch.FloatTensor(samples.nextstate).to(device)
            action_batch = torch.FloatTensor(samples.action).to(device)
            reward_batch = torch.FloatTensor(samples.reward).to(device).unsqueeze(1)
            done_batch = torch.FloatTensor(samples.real_done).to(device).unsqueeze(1)
            
            # update q-funcs
            q1_loss_step, q2_loss_step = self.update_q_functions(state_batch, action_batch, reward_batch, nextstate_batch, done_batch)
            q_loss_step = q1_loss_step + q2_loss_step
            self.q_optimizer.zero_grad()
            q_loss_step.backward()
            self.q_optimizer.step()

            # update policy and temperature parameter
            for p in self.q_funcs.parameters():
                p.requires_grad = False
            pi_loss_step, a_loss_step = self.update_policy_and_temp(state_batch)
            self.policy_optimizer.zero_grad()
            pi_loss_step.backward()
            self.policy_optimizer.step()
            self.temp_optimizer.zero_grad()
            a_loss_step.backward()
            self.temp_optimizer.step()
            for p in self.q_funcs.parameters():
                p.requires_grad = True

            q1_loss += q1_loss_step.detach().item()
            q2_loss += q2_loss_step.detach().item()
            pi_loss += pi_loss_step.detach().item()
            a_loss += a_loss_step.detach().item()
            if i % self.update_interval == 0:
                self.update_target()
        return q1_loss, q2_loss, pi_loss, a_loss

    @property
    def alpha(self):
        return self.log_alpha.exp()
    
if __name__ == "__main__":
        

    # Call the Pendulum Example
    env = gym.make( 'Pendulum-v1', g = 9.81 )

    # Set the random number seeds and get the Initial State
    env.reset( seed = round( time.time( ) ) )

    # Set the random number seeds for others
    env.action_space.seed( round( time.time( ) ) )
    torch.manual_seed(     round( time.time( ) ) )
    np.random.seed(        round( time.time( ) ) )

    # Get the dimension of states and actions, and also the 
    # [WARNING] This is for environments where we assume the mean of action is 0. 
    n_state    = env.observation_space.shape[ 0 ] 
    n_action   = env.action_space.shape[ 0 ]
    max_action = float( env.action_space.high  )

    # Define the agent, noise and replay buffers
    
    agent = SAC_Agent( time.time( ), n_state, n_action )

    parser = ArgumentParser()
    parser.add_argument('--env', type=str, default='HalfCheetah-v2')
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--use_obs_filter', dest='obs_filter', action='store_true')
    parser.add_argument('--update_every_n_steps', type=int, default=1)
    parser.add_argument('--n_random_actions', type=int, default=10000)
    parser.add_argument('--n_collect_steps', type=int, default=1000)
    parser.add_argument('--n_evals', type=int, default=1)
    parser.add_argument('--experiment_name', type=str, default='')
    parser.add_argument('--make_gif', dest='make_gif', action='store_true')
    parser.add_argument('--save_model', dest='save_model', action='store_true')
    parser.add_argument('--total_steps', type=int, default=int(1e7))
    parser.set_defaults(obs_filter=False)
    parser.set_defaults(save_model=False)

    args = parser.parse_args()
    params = vars(args)

    update_timestep = params['update_every_n_steps']
    seed = params['seed']
    log_interval = 1000
    gif_interval = 500000
    n_random_actions = params['n_random_actions']
    n_evals = params['n_evals']
    n_collect_steps = params['n_collect_steps']
    use_statefilter = params['obs_filter']
    save_model = params['save_model']
    total_steps = params['total_steps']


    avg_length = 0
    time_step = 0
    cumulative_timestep = 0
    cumulative_log_timestep = 0
    n_updates = 0
    i_episode = 0
    log_episode = 0
    samples_number = 0
    episode_rewards = []
    episode_steps = []

    max_steps = env.spec.max_episode_steps

    if use_statefilter:
        state_filter = MeanStdevFilter(env.env.observation_space.shape[0])
    else:
        state_filter = None


    while samples_number < total_steps:
        time_step = 0
        episode_reward = 0
        i_episode += 1
        log_episode += 1
        state, _ = env.reset()
        if state_filter:
            state_filter.update(state)
        done = False

        truncated = False

        while (not done and not truncated ):
            cumulative_log_timestep += 1
            cumulative_timestep += 1
            time_step += 1
            samples_number += 1
            if samples_number < n_random_actions:
                action = env.action_space.sample()
            else:
                action = agent.get_action(state, state_filter=state_filter)
            nextstate, reward, done, truncated, _ = env.step(action)
            # if we hit the time-limit, it's not a 'real' done; we don't want to assign low value to those states
            
            real_done = False if time_step == max_steps else done
            agent.replay_pool.push(Transition(state, action, reward, nextstate, real_done))
            state = nextstate
            if state_filter:
                state_filter.update(state)
            episode_reward += reward
            # update if it's time
            if cumulative_timestep % update_timestep == 0 and cumulative_timestep > n_collect_steps:
                q1_loss, q2_loss, pi_loss, a_loss = agent.optimize(update_timestep, state_filter=state_filter)
                n_updates += 1
            # logging
            if cumulative_timestep % log_interval == 0 and cumulative_timestep > n_collect_steps:

                avg_length = np.mean(episode_steps)
                running_reward = np.mean(episode_rewards)
                
                eval_reward = evaluate_agent(env, agent, state_filter, n_starts=n_evals)

                print('Episode {} \t Samples {} \t Avg length: {} \t Test reward: {} \t Train reward: {} \t Number of Policy Updates: {}'.format(i_episode, samples_number, avg_length, eval_reward, running_reward, n_updates))
                episode_steps = []
                episode_rewards = []


        episode_steps.append(time_step)
        episode_rewards.append(episode_reward)