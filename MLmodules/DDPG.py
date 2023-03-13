import sys
import copy
import torch
import numpy 	 		   as np
import gymnasium 		   as gym
import torch.nn  		   as nn
import torch.nn.functional as F

sys.path += [ "../MLmodules" ]

from MLutils import Actor, Critic

# Code from:
# [REF] https://github.com/mosesnah-shared/machine-learning-tutorial/blob/main/notebooks/DDPG.ipynb
device = torch.device( "cuda" if torch.cuda.is_available( ) else "cpu" )

class DDPG( object ):

    def __init__( self, n_state, n_action, max_action = 1., gamma = 0.99, tau = 0.005 ):

        # Actor Network , its target (copy) Network, and the ADAM optimizer.
        self.actor             = Actor( n_state, n_action, max_action = max_action ).to( device )
        self.actor_target      = copy.deepcopy( self.actor )
        self.actor_optimizer   = torch.optim.Adam(  self.actor.parameters( ), lr = 1e-3 )

        # Critic Network, its target (copy) Network, and the ADAM optimizer.
        self.critic            = Critic( n_state, n_action ).to( device )
        self.critic_target     = copy.deepcopy( self.critic )
        self.critic_optimizer  = torch.optim.Adam(  self.critic.parameters( ), lr = 1e-3 )

        # The discount factor gamma and the soft-update gain tau
        self.gamma = gamma
        self.tau   = tau

        # The maximum action.
        self.max_action = max_action

    
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
    
    def update( self, replay_buffer, batch_size = 256 ):
        """
            Mini-batch update. 
        """
        # Randomly sample batch_size numbers of S A R S.
        states, actions, rewards, next_states, is_done = replay_buffer.sample( batch_size )

        # ====================================================== #
        # ================ Critic Optimizer Part =============== #
        # ====================================================== #
        # Keep in mind that the original paper first optimizes the Critic network

        # Critic loss 
        Qprime  = self.critic_target( next_states, self.actor_target( next_states ) )
        Qprime  = rewards + ( ( 1. - is_done ) * self.gamma * Qprime ).detach( )
        Q       = self.critic( states,actions )

        critic_loss  = F.mse_loss( Q, Qprime )

        # Update Critic network
        self.critic_optimizer.zero_grad( )
        critic_loss.backward( ) 
        self.critic_optimizer.step( )

        # ====================================================== #
        # ================ Actor Optimizer Part ================ #
        # ====================================================== #
        # Actor loss, it is simply the mean of the Q function 
        # The Q function value Q( s, a ) is actually Q( s, mu( s ) ), hence the Q function is described as the parameters of mu (actor).
        # Since a is a continuous function, we can compute its gradient.   
        actor_loss = - self.critic( states, self.actor( states ) ).mean( )
        
        # Update (Optimize) Actor network
        self.actor_optimizer.zero_grad( )
        actor_loss.backward( )
        self.actor_optimizer.step( )


        # The "soft" update of target networks
        for target_param, param in zip( self.actor_target.parameters( ), self.actor.parameters( ) ):
            target_param.data.copy_( param.data * self.tau + target_param.data * ( 1.0 - self.tau ) )
       
        for target_param, param in zip( self.critic_target.parameters( ), self.critic.parameters( ) ):
            target_param.data.copy_( param.data * self.tau + target_param.data * ( 1.0 - self.tau ) )


    def save( self, filename ):
        
        torch.save( self.critic.state_dict( )           , filename + "_critic"              )
        torch.save( self.critic_optimizer.state_dict( ) , filename + "_critic_optimizer"    )
        
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
        