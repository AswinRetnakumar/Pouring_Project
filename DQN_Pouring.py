import gym
import numpy as np
import random
from keras.models import Sequential
from keras.layers import Dense, Dropout, Conv2D, MaxPooling2D, Flatten
from keras.optimizers import Adam
from kukaPouring import kukaPouring
from collections import deque
import time
import pickle

class DQN:
    def __init__(self, env):
        self.env     = env
        self.memory  = deque(maxlen=10000)
        
        self.gamma = 0.85
        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995
        self.learning_rate = 0.009
        self.tau = .125

        self.model        = self.create_model()
        self.target_model = self.create_model()

    def create_model(self):
        model   = Sequential()
        state_shape  = self.env.observation_space.shape
        print("State shape:", state_shape)
        model.add(Conv2D(64, (5, 5), padding ='same', activation = 'relu',input_shape = state_shape))
        model.add(Conv2D(64, (5, 3), padding ='same', activation = 'relu'))
        #model.add(MaxPooling2D(pool_size = (2,2), strides = None,padding = 'valid',data_format = None)) #pooling to reduce position dependence of features, downsampling done
        #model.add(Dropout(0.25))
        model.add(Conv2D(32, (3,3), padding = 'same', activation='relu'))
        #model.add(Conv2D(32, (3,3), padding = 'same', activation='relu'))
        #model.add(Conv2D(32, (3,3), padding = 'same', activation='relu'))
        '''
        model.add(Conv2D(64, (4,2), padding = 'same', activation='relu'))
        
        model.add(MaxPooling2D(pool_size = (2,2), strides = None,padding = 'valid',data_format = None))
        '''
        model.add(Flatten())
        model.add(Dense(128, activation="relu"))
        model.add(Dense(self.env.action_space.n))
        model.compile(loss="mean_squared_error",
            optimizer=Adam(lr=self.learning_rate))
        return model

    def act(self, state):
        self.epsilon *= self.epsilon_decay
        self.epsilon = max(self.epsilon_min, self.epsilon)
        if np.random.random() < self.epsilon:
            return self.env.action_space.sample()
        
        return np.argmax(self.model.predict(state)[0])

    def remember(self, state, action, reward, new_state, done):
        self.memory.append([state, action, reward, new_state, done])

    def replay(self):
        batch_size = 16
        if len(self.memory) < batch_size: 
            return

        samples = random.sample(self.memory, batch_size)
        for sample in samples:
            state, action, reward, new_state, done = sample
            target = self.target_model.predict(state)
            if done:
                target[0][action] = reward
            else:
                Q_future = max(self.target_model.predict(new_state)[0])
                target[0][action] = reward + Q_future * self.gamma
            self.model.fit(state, target, epochs=1, verbose=0)

    def target_train(self):
        weights = self.model.get_weights()
        target_weights = self.target_model.get_weights()
        for i in range(len(target_weights)):
            target_weights[i] = weights[i] * self.tau + target_weights[i] * (1 - self.tau)
        self.target_model.set_weights(target_weights)

    def save_model(self, fn):
        self.model.save_weights(fn)

def main():

    record = False
    trials  = 1
    
    if record:
        
        env = kukaPouring()
        trial_len = env.max_steps
        dqn_agent = DQN(env=env)

        for trial in range(trials):
            replay_file = open('good_action', 'wb')
            cur_state = env.reset()
            time.sleep(0.5)
            accum_rwd = 0
            for step in range(trial_len):
                action = int(input("Enter action [0:8]"))
                if action > 8 or action==None:
                    action = 0			
                new_state, reward, done, _ = env.step(action)
                accum_rwd += reward
                # reward = reward if not done else -20
            
                dqn_agent.remember(cur_state, action, reward, new_state, done)
                
                cur_state = new_state
                if done:
                    break
            print("Reward for iter "+ str(trial)+":"+str(accum_rwd))   
	    for _ in range(100):
                for i in range(14):
                    dqn_agent.memory.append(dqn_agent.memory[i])
        pickle.dump(dqn_agent.memory ,replay_file)
        print ("Current Length: ", len(dqn_agent.memory))
        replay_file.close() 


    env     = kukaPouring()
    time.sleep(0.5)
    gamma   = 0.9
    epsilon = .95
    
    trials  = 100
    trial_len = env.max_steps

    # updateTargetNetwork = 1000
    dqn_agent = DQN(env=env)
    steps = []
    infile = open('good_action','rb')
    dqn_agent.memory = pickle.load(infile)
    infile.close()
    print ("Current Length: ", len(dqn_agent.memory))


    for trial in range(trials):
        replay_file = open('experience_env', 'wb') 
        cur_state = env.reset()
        time.sleep(0.5)
        accum_rwd = 0
        for step in range(trial_len):
            action = dqn_agent.act(cur_state)
            new_state, reward, done, _ = env.step(action)
            print(str(step)+":"+str(reward))
            accum_rwd += reward
            # reward = reward if not done else -20
        
            dqn_agent.remember(cur_state, action, reward, new_state, done)
            
            
            cur_state = new_state
            if done:
                break
        print("Step: "+ str(step))
        print("Reward for iter "+ str(trial)+":"+str(accum_rwd)) 
	print("Length of mem "+str(len(dqn_agent.memory))) 
	if trial % 2 == 0:
                print("Training.....")
                dqn_agent.replay()       # internally iterates default (prediction) model
                dqn_agent.target_train() # iterates target model

 
        pickle.dump(dqn_agent.memory ,replay_file)  
        if trial % 25 == 0:
            dqn_agent.save_model("trial-{}.model".format(trial))
        replay_file.close()
            


if __name__ == "__main__":
    main()
