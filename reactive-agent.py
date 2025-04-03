
import gymnasium as gym
import numpy as np
import pygame

ENABLE_WIND = False
WIND_POWER = 15.0
TURBULENCE_POWER = 0.0
GRAVITY = -10.00
RENDER_MODE = 'human'
#RENDER_MODE = None #seleccione esta opção para não visualizar o ambiente (testes mais rápidos)
EPISODES = 1000

env = gym.make("LunarLander-v3", render_mode =RENDER_MODE, 
    continuous=True, gravity=GRAVITY, 
    enable_wind=ENABLE_WIND, wind_power=WIND_POWER, 
    turbulence_power=TURBULENCE_POWER)


def check_successful_landing(observation):
    x = observation[0]
    vy = observation[3]
    theta = observation[4]
    contact_left = observation[6]
    contact_right = observation[7]

    legs_touching = contact_left == 1 and contact_right == 1

    on_landing_pad = abs(x) <= 0.2

    stable_velocity = vy > -0.2
    stable_orientation = abs(theta) < np.deg2rad(20)
    stable = stable_velocity and stable_orientation
 
    if legs_touching and on_landing_pad and stable:
        print("✅ Aterragem bem sucedida!")
        return True

    print("⚠️ Aterragem falhada!")        
    return False
        
def simulate(steps=1000,seed=None, policy = None):    
    observ, _ = env.reset(seed=seed)
    for step in range(steps):
        action = policy(observ)

        observ, _, term, trunc, _ = env.step(action)

        if term or trunc:
            break

    success = check_successful_landing(observ)
    return step, success


#Perceptions
def perceptions(observation):
    x, y, vx, vy, theta, vtheta, contact_left, contact_right = observation
    return x, y, vx, vy, theta, vtheta, contact_left, contact_right

#Actions
motor_up = np.array([1, 0])
motor_right = np.array([0, -1])
motor_left = np.array([0, 1])
stable = np.array([0, 0])

def reactive_agent(observation):
    action = np.array([0, 0])
    x, y, vx, vy, theta, vtheta, contact_left, contact_right = observation
    
    LimiarXHigh = 0.25
    LimiarX = 0.2
    LimiarXLow = 0.15
    
    LimiarY = 0.7
    LimiarYLow = 0.5
    
    LimiarAngleHigh = 11
    LimiarAngle = 2
    LimiarAngleLow = 0.5
    
    LimiarVyWind = 0.15
    LimiarVy = 0.35
    
    LimiarVx = 0.6
    
    if ENABLE_WIND == True:
        if contact_left and contact_right and abs(x) < LimiarX:
            action = stable
        
        elif -LimiarX < x < LimiarX and vtheta < 0 and abs(theta) <  LimiarAngleLow:
            action = motor_right
        
        elif -LimiarX < x < LimiarX and vtheta > 0 and abs(theta) <  LimiarAngleLow:
            action = motor_left
        
        elif -LimiarX < x < LimiarX:
            action = motor_up
        
        elif x > LimiarX and vtheta < 0 and abs(theta) < LimiarAngleLow and y < LimiarY:
            action = motor_up
        
        elif x > LimiarX and vtheta < 0 and abs(theta) < LimiarAngleLow:
            action = motor_right
        
        elif x < -LimiarX and vtheta > 0 and abs(theta) < LimiarAngleLow and y < LimiarY:
            action = motor_up
        
        elif x < -LimiarX and vtheta > 0 and abs(theta) < LimiarAngleLow:
            action = motor_left
        
        elif vy < -LimiarVyWind:
            action = motor_up
        
        elif theta < 0:
            action = motor_right
        
        elif theta > 0:
            action = motor_left
        
        return action
    
    if contact_left and contact_right and abs(x) < LimiarX:
        action = stable
        
    elif vy < -LimiarVy:
        action = motor_up

    elif x < -LimiarXHigh and y < LimiarYLow and vtheta > 0 and abs(theta) > np.deg2rad(LimiarAngle):
        action = motor_up + motor_left
        
    elif x > LimiarXHigh and y < LimiarYLow and vtheta < 0 and abs(theta) > np.deg2rad(LimiarAngle):
        action = motor_up + motor_right
    
    elif theta > np.deg2rad(LimiarAngleHigh):
        action = motor_left
    
    elif theta < np.deg2rad(-LimiarAngleHigh):
        action = motor_right
        
    elif x > LimiarXLow:
        action = motor_right
    elif x < -LimiarXLow:
        action = motor_left
        
    elif vx < -LimiarVx:
        action = motor_left
    
    elif vx > LimiarVx:
        action = motor_right
    
    elif x < LimiarXHigh and vx < 0 and vtheta < 0 and theta > np.deg2rad(-LimiarAngleLow):
        action = motor_left

    elif x > -LimiarXHigh and vx > 0 and vtheta > 0 and theta < np.deg2rad(LimiarAngleLow):
        action = motor_right
    
    elif vtheta < 0:
        action = motor_right
    elif vtheta > 0:
        action = motor_left
        
    return action
    
def keyboard_agent(observation):
    action = [0,0] 
    keys = pygame.key.get_pressed()
    
    print('observação:',observation)

    if keys[pygame.K_UP]:  
        action =+ np.array([1,0])
    if keys[pygame.K_LEFT]:  
        action =+ np.array( [0,-1])
    if keys[pygame.K_RIGHT]: 
        action =+ np.array([0,1])

    return action
    

success = 0.0
steps = 0.0
for i in range(EPISODES):
    st, su = simulate(steps=1000000, policy=reactive_agent)

    if su:
        steps += st
    success += su
    
    if su>0:
        print('Média de passos das aterragens bem sucedidas:', steps/success*100)
    print('Taxa de sucesso:', success/(i+1)*100)