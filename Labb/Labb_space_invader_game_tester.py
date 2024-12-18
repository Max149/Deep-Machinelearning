import keras
from keras import layers
import gymnasium as gym
import ale_py
from gymnasium.wrappers.frame_stack import FrameStack
from gymnasium.wrappers.atari_preprocessing import AtariPreprocessing
 
gym.register_envs(ale_py)

model_file = "Labb/space_invaders_qmodel_109990.keras"
agent = keras.models.load_model(model_file)

env = gym.make("SpaceInvadersNoFrameskip-v4", render_mode="human")
env = AtariPreprocessing(env)
env = FrameStack(env, 4)

state, _ = env.reset()
done = False
while not done:
# first convert to a tensor for compute efficiency
    state_tensor = keras.ops.convert_to_tensor(state)
# shape of state is 4, 84, 84, but we need 84, 84, 4
    state_tensor = keras.ops.transpose(state_tensor, [1, 2, 0])
# Add batch dimension
    state_tensor = keras.ops.expand_dims(state_tensor, 0)
# ’predict’ method is for large batches, call as function instead
    action_probs = agent(state_tensor, training=False)
# Take ’best’ action
    action = keras.ops.argmax(action_probs[0]).numpy()

    state, reward, done, _, _ = env.step(action)
# AQ-learningmodelthatissuitableforthisproblemisthefollowing(fromtheDeepMindpaper):
    num_actions = 6 # see atari gym documentation for SpaceInvaders
 
    def create_q_model():
        return keras.Sequential(
 [
            layers.Input(shape=(84, 84, 4)),
            layers.Conv2D(32, kernel_size=8, strides=4, activation="relu"),
            layers.Conv2D(64, kernel_size=4, strides=2, activation="relu"),
            layers.Conv2D(64, kernel_size=3, strides=1, activation="relu"),
            layers.Flatten(),
            layers.Dense(512, activation="relu"),
            layers.Dense(num_actions, activation="linear")
 ]
)