import sys
import time
import torch
import numpy 	 		as np
import gymnasium 		as gym
import moviepy.editor  	as mpy

sys.path += [ "../MLmodules" ]
from DDPG    import DDPG
from MLutils import OUNoise, ReplayBuffer

def train( n_epi = 400 ):

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
	agent         = DDPG( n_state, n_action, max_action )
	OUnoise       = OUNoise( env.action_space )
	replay_buffer = ReplayBuffer( n_state, n_action )

	# The number of "batch" that will be sampled from the replay buffer will be "batch_size" 
	n_batch_size  = 256

	# Saving these values to plot the performance at the end.
	frames        = [ ]
	whole_rewards = [ ]

	# For the pendulum model the best reward is 0, hence saving a -infinity value. 
	best_model_val = -np.inf
	rewards       = [ ]
	avg_rewards   = [ ]

	for episode in range( n_epi ):
		
		# Initialize the OU noise 
		OUnoise.reset( )

		# Reset the Environment
		state, _ = env.reset(  )	

		# Initialize the episode's reward
		episode_reward = 0
		
		# For pendulum v1 gym, a single simulation is maximum 500-steps long. 
		# [REF] https://gymnasium.farama.org/environments/classic_control/cart_pole/
		for step in range( env._max_episode_steps ):

			# Get the action value from the deterministic policy Actor network.
			action = agent.get_action( state )

			if is_save_video : 
				frames.append( env.render( ) )
			else:
				env.render( )

			# Apply the OU noise on this action
			action = OUnoise.add_noise2action( action, step )

			# Run a single step of simulation
			new_state, reward, done, truncated, _ = env.step( action )  

			done = True if done or truncated else False

			# Add this to our replay buffer, note that push simply generates the tuple and add 
			replay_buffer.add( state, action, reward, new_state, done )
			
			# Once the agent memory is full, then update the policy via replay buffer.
			if replay_buffer.current_size > n_batch_size: agent.update( replay_buffer, batch_size = n_batch_size )        
			
			# Update the state and reward value 
			state = new_state
			episode_reward += reward

			if done or truncated:
				break

		if best_model_val <= episode_reward:
			best_model_val = episode_reward 

			# If this policy has a good result, save it 
			agent.save( "../models/DDPG/DDPG" ) 

		# Once a single simulation is done, append the values that will be plotted later
		rewards.append( episode_reward )
		avg_rewards.append( np.mean( rewards[ -10 : ] ) )
		sys.stdout.write("episode: {}, reward: {}, average_reward: {} \n".format( episode, np.round( episode_reward, decimals = 2 ), avg_rewards[ -1 ] ) ) 

	whole_rewards.append(  rewards  )
	env.close( )

def replay( is_save_video ):

	# Call the Pendulum Example
	env = gym.make( 'Pendulum-v1', g = 9.81, render_mode = "rgb_array" if is_save_video else "human" )

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

	model_path = "../MLmodels/DDPG/DDPG"
	agent      = DDPG( n_state, n_action, max_action )
	agent.load( model_path )

	# Reset the Environment
	state, _ = env.reset(  )	

	frames = []
	
	for _ in range( 500 ):

		if is_save_video: 
			frames.append( env.render( )  ) 

		else:
			env.render( )

		# Get the choice of action and the pi( a_t | s_t ) for the gradient calculation
		action = agent.get_action( state )
		new_state, _, done, truncated, _ = env.step( action )

		# If the trail encounters the terminal state
		if done or truncated: 
			break
		
		state = new_state

	# Save Video
	env.close( )

	if is_save_video:
		clip = mpy.ImageSequenceClip( frames, fps = 30 )
		clip.write_gif( "../videos/DDPG.gif" )

if __name__ == "__main__":

	
	is_train_model = True
	is_load_model = False
	is_save_video = True

	if is_train_model and not is_load_model: 
		train(  )
	
	elif is_load_model and not is_train_model:
		replay( is_save_video )

	else:
		pass
