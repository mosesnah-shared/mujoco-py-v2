import sys
import copy
import time

import torch
import torch.autograd
import torch.nn            as nn
import torch.nn.functional as F 
import numpy               as np
import gymnasium as gym
import moviepy.editor      as mpy

# Check whether GPU computation (i.e., CUDA) is available.
device = torch.device( "cuda" if torch.cuda.is_available( ) else "cpu" )

sys.path += [ "../MLmodules" ]

from MLutils import OUNoise, ReplayBuffer, Actor, Critic

class TD3( object ):

    def __init__( self, n_state, n_action, max_action, gamma = 0.95, tau = 0.05, policy_noise = 0.2, noise_clip = 0.5, policy_freq = 2 ):

        self.actor            = Actor( n_state, n_action, max_action = max_action ).to( device )
        self.actor_target     = copy.deepcopy( self.actor )
        self.actor_optimizer  = torch.optim.Adam( self.actor.parameters( ) , lr = 1e-4 )

        self.critic1          = Critic( n_state, n_action ).to( device )
        self.critic2          = Critic( n_state, n_action ).to( device )

        self.critic1_target   = copy.deepcopy( self.critic1 )
        self.critic2_target   = copy.deepcopy( self.critic2 )

        self.critic1_optimizer = torch.optim.Adam( self.critic1.parameters( ), lr = 1e-3 )
        self.critic2_optimizer = torch.optim.Adam( self.critic2.parameters( ), lr = 1e-3 )

        self.max_action = max_action

        self.gamma = gamma
        self.tau   = tau 

        self.policy_noise = policy_noise
        self.noise_clip   = noise_clip
        self.policy_freq  = policy_freq
        
        self.total_it = 0 

    def get_action( self, state ):

        # Conduct the a = mu(s), where mu is a "deterministic function"
        # Unsqueeze makes an 1 x n_s array of state. 
        state  = torch.from_numpy( state ).float( ).unsqueeze( 0 ).to( device )

        # Returns an 1 x n_a array of state
        # forward method can be omitted
        action = self.actor( state )

        # Change action from Torch to Numpy.
        # Since n_a is 1 for this case, action is simply an 1x1 array.
        # Hence, flattening the data. 
        action = action.cpu( ).data.numpy( ).flatten( )

        return action
    
    def update( self, replay_buffer, batch_size: int = 256 ):

        self.total_it += 1
        state, action, reward, next_state, is_done = replay_buffer.sample( batch_size )

        with torch.no_grad( ):
            noise       = ( torch.randn_like( action ) * self.policy_noise  ).clamp( -self.noise_clip, self.noise_clip )
            next_action = ( self.actor_target( next_state ) + noise ).clamp( -self.max_action, self.max_action )

            target_Q1, target_Q2 = self.critic1_target( next_state, next_action ), self.critic2_target( next_state, next_action )
            target_Q = torch.min( target_Q1, target_Q2 )
            target_Q = reward + (  ( 1. - is_done ) * self.gamma * target_Q ).detach( )

		# Get Current Q estimates
        current_Q1, current_Q2 = self.critic1( state, action ), self.critic2( state, action )

		# Compute critic loss
        critic_loss = F.mse_loss( current_Q1, target_Q ) + F.mse_loss( current_Q2, target_Q )

		# Optimize the critic
        self.critic1_optimizer.zero_grad()
        self.critic2_optimizer.zero_grad()

        critic_loss.backward()

        self.critic1_optimizer.step()
        self.critic2_optimizer.step()

		# Delayed policy updates
        if self.total_it % self.policy_freq == 0:

			# Compute actor losse
            actor_loss = -self.critic1( state, self.actor( state ) ).mean( )
			
			# Optimize the actor 
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

			# Update the frozen target models
            for target_param, param in zip( self.critic1_target.parameters( ), self.critic1.parameters( ) ):
                target_param.data.copy_( self.tau * param.data + ( 1 - self.tau ) * target_param.data )

            for target_param, param in zip( self.critic2_target.parameters( ), self.critic2.parameters( ) ):
                target_param.data.copy_( self.tau * param.data + ( 1 - self.tau ) * target_param.data )                
                
            for target_param, param in zip( self.actor_target.parameters( ) ,  self.actor.parameters( ) ):
                target_param.data.copy_( self.tau * param.data + ( 1 - self.tau ) * target_param.data )


    def save( self, filename ):
        
        torch.save( self.critic1.state_dict( )           , filename + "_critic1"              )
        torch.save( self.critic1_optimizer.state_dict( ) , filename + "_critic1_optimizer"    )

        torch.save( self.critic2.state_dict( )           , filename + "_critic2"              )
        torch.save( self.critic2_optimizer.state_dict( ) , filename + "_critic2_optimizer"    )        
        
        torch.save( self.actor.state_dict( )            , filename + "_actor"               )
        torch.save( self.actor_optimizer.state_dict( )  , filename + "_actor_optimizer"     )


    def load( self, filename ):

        # Load Critic
        self.critic.load_state_dict(            torch.load( filename + "_critic"           )  )
        self.critic_optimizer.load_state_dict(  torch.load( filename + "_critic_optimizer" )  )
        self.critic_target = copy.deepcopy( self.critic )

        # Load Actor
        self.actor.load_state_dict(             torch.load( filename + "_actor"            )  )
        self.actor_optimizer.load_state_dict(   torch.load( filename + "_actor_optimizer"  )  )
        self.actor_target = copy.deepcopy( self.actor )
        
