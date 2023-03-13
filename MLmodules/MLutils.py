import torch
import torch.nn            as nn
import torch.nn.functional as F 
import numpy               as np

class Actor( nn.Module ):
    def __init__( self, n_state: int, n_action: int, n_hidden: int = 256, max_action: float = 1.0 ):

        # Class inheritance. 
        super( Actor, self ).__init__( )

        # Save the maximum action value 
        assert max_action >= 0
        self.max_action = max_action

        # First Layer, changes array  with size N x ( n_state  ) to N x ( n_hidden )
        self.l1 = nn.Linear(  n_state, n_hidden )

        # Second Layer, changes array with size N x ( n_hidden ) to N x ( n_hidden )
        self.l2 = nn.Linear( n_hidden, n_hidden )

        # Third Layer, changes array  with size N x ( n_hidden ) to N x ( n_action )
        self.l3 = nn.Linear( n_hidden, n_action )
        
    def forward( self, state ):
        
        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l1( state ) )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l2( x ) )

        # Applying to tanh, which ranges the value from -1 to +1
        x = torch.tanh( self.l3( x ) ) 

        # Since the x value is from -1 to +1, we change the range to -max_action to +max_action.
        return x * self.max_action
    

class Critic( nn.Module ):
    """
        Learning the Q(s,a) function, which is the "Quality" function. Hence, input is a concatenation of state, action and the output is a scalar. 
    """
    def __init__( self, n_state, n_action, n_hidden = 256 ):

        # Class inheritance. 
        super( Critic, self ).__init__()

        self.l1 = nn.Linear( n_state + n_action, n_hidden )
        self.l2 = nn.Linear(           n_hidden, n_hidden )
        self.l3 = nn.Linear(           n_hidden,        1 )

    
    def forward( self, state, action ):

        # Concatenation of state and action vector.
        # Note that the critic network update from batch of data 
        # If batch size is N, then 
        # state = N x n_s , action = N x n_a
        # x = N x (n_s + n_a )
        x = torch.cat( [ state, action ], dim = 1 )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l1( x ) )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l2( x ) )

        # A simple Ax + b combination 
        x = self.l3( x )

        # The output is a N x 1 array. 
        return x
        

class OUNoise( object ):
    
    def __init__( self, action_space, mu = 0.0, theta = 0.15, max_sigma = 0.3, min_sigma = 0.3, decay_period = 1e5 ):
        self.mu           = mu
        self.theta        = theta
        self.sigma        = max_sigma
        self.max_sigma    = max_sigma
        self.min_sigma    = min_sigma
        self.decay_period = decay_period
        self.action_dim   = action_space.shape[ 0 ]
        self.low          = action_space.low
        self.high         = action_space.high
        self.reset( )
        
    def reset( self ):
        # Caution! The state here is not the "pendulum"'s state, but the "noise" itself. 
        self.state = np.ones( self.action_dim ) * self.mu
        
    def step( self, t = 0 ):
        
        # Call the current noise value. 
        x  = self.state

        # randn returns a sample from the standard (i.e., normal) distribution
        dx = self.theta * ( self.mu - x ) + self.sigma * np.random.randn( self.action_dim )

        # For our case, we simply set the max_sigma and min_sigma the same, hence the sigma value is constant for us
        self.sigma = self.max_sigma - ( self.max_sigma - self.min_sigma ) * min( 1.0, t / self.decay_period )

        # Time-increment. x_{n+1} = x_{n} + dx
        self.state = x + dx

        return self.state
    
    def add_noise2action( self, action, t = 0 ): 
        
        # Calculate the noise with respect to the given time. 
        ou_noise   = self.step( t )

        # Adding ou noise onto the action and then clipping it.
        return np.clip( action + ou_noise, self.low, self.high )
    
class ReplayBuffer( object ):

    def __init__( self, n_state, n_action, max_size = 100000 ):

        # Save the dimension of state, dimension of action and the maximum size of the replay buffer
        self.n_state  = n_state
        self.n_action = n_action
        self.max_size = max_size

        # Defining the current size of the replay buffer, just to make the sampling easy. 
        self.current_size = 0

        # Defining the Index Pointer (ptr) of the replaybuffer. 
        # This is required for "adding" the experiences on the replaybuffer. 
        self.idx_ptr      = 0

        # Device
        self.device = torch.device( "cuda" if torch.cuda.is_available( ) else "cpu" )

        # Defining the 2D arrays of the ReplayBuffer
        # 2D array definition is necessary to forward the Neural Network
        self.states      = np.zeros( ( max_size, n_state   ) )
        self.actions     = np.zeros( ( max_size, n_action  ) )
        self.rewards     = np.zeros( ( max_size, 1         ) )
        self.next_states = np.zeros( ( max_size, n_state   ) )
        self.is_done     = np.zeros( ( max_size, 1         ) )


    def add( self, state, action, reward, next_state, is_done ):
        """
            Adding a state-action-reward-next_state pair into the ReplayBuffer. 
        """

        self.states[      self.idx_ptr ] = state
        self.actions[     self.idx_ptr ] = action
        self.rewards[     self.idx_ptr ] = reward
        self.next_states[ self.idx_ptr ] = next_state
        self.is_done[     self.idx_ptr ] = is_done

        # Update our index pointer. Note that the "oldest" experiences are overwritten.
        self.idx_ptr = ( self.idx_ptr + 1 ) % self.max_size

        # Update the current size of the replay buffer
        self.current_size = min( self.current_size + 1, self.max_size )

    def sample( self, n_batch_size ):
        """
            Collect "n_batch_size" samples from the replay buffer and return it as a batch.
        """
        idx = np.random.randint( 0, self.current_size, size = n_batch_size )

        # Returning the 2D numpy array as a 2D torch array.

        return ( 
            torch.FloatTensor(      self.states[ idx ]  ).to( self.device ) ,
            torch.FloatTensor(     self.actions[ idx ]  ).to( self.device ) , 
            torch.FloatTensor(     self.rewards[ idx ]  ).to( self.device ) ,
            torch.FloatTensor( self.next_states[ idx ]  ).to( self.device ) ,
            torch.FloatTensor(     self.is_done[ idx ]  ).to( self.device )   
        )


    def reset( self ):
        """
            Reset all the replay buffers to zeros
        """
        self.states      = np.zeros( ( self.max_size, self.n_state   ) )
        self.actions     = np.zeros( ( self.max_size, self.n_action  ) )
        self.rewards     = np.zeros( ( self.max_size, 1              ) )
        self.next_states = np.zeros( ( self.max_size, self.n_state   ) )
        self.is_done     = np.zeros( ( self.max_size, 1              ) )