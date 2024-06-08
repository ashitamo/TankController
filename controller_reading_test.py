import pygame
import pygame.event

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(joystick.get_name())
print(joystick.get_numaxes())
print(joystick.get_numbuttons())
# print(joystick.get_instance_id())

while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            print(event.axis, event.value)

        if event.type == pygame.JOYBUTTONDOWN:
            print(event.button)
        
        if event.type == pygame.JOYHATMOTION:
            print(event.value)
        

