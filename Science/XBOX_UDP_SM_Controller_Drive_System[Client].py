import pygame
import socket
import time

AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 3
AXIS_R2 = 4
AXIS_L2 = 5
# Labels for DS4 controller buttons
# # Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_SQUARE = 2
BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 3
GEARUP = 5
GEARDOWN = 4
BUTTON_L2 = 7
BUTTON_R2 = 8
BUTTON_SHARE = 8
BUTTON_OPTIONS = 6

BUTTON_LEFT_STICK = 10
BUTTON_RIGHT_STICK = 11

UP_ARROW = 11
DOWN_ARROW = 12
LEFT_ARROW = 13
RIGHT_ARROW = 14
BUTTON_PS = 5
BUTTON_PAD = 15
GEARUP_toggle = True
GEARDOWN_toggle = True
GD = 0
GU = 0
Gear = 0
kv = 0
resetValue = 0
driveMode=0
# Debouncing time in milliseconds
debounce_time = 200

# Initial Count

Gear = 0

# Timestamp of the last button press
last_press_time = 0


class TextPrint:

    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (192, 192, 192))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 20

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 20


def map(value, fromLow, fromHigh, toLow, toHigh):
    # Calculate the scaled value
    scaled_value = (value - fromLow) * (toHigh - toLow) / \
        (fromHigh - fromLow) + toLow
    # Return the scaled value
    return round(scaled_value)


pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("SM & Drive Controls")
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)
print("Joystick Successfully sected")
controller.init()
output_string = " M{Gear}X{left_joystick_x}Y{left_joystick_y}S{kv}R{resetValue}D{driveMode}E"
# Set up the socket
# HOST = '192.168.137.250'  # The host IP address
HOST = "10.0.0.11"
# HOST = "127.0.0.1"
PORT = 5005      # The port to listen on
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    addr = (HOST, PORT)
    # s.connect(addr)
    # s, addr1 = s.accept()
    text_print = TextPrint()
    pygame.event.pump()
    font = pygame.font.Font(None, 36)
    while True:
        screen.fill((0, 0, 0))
        text_print.reset()
        text_print.tprint(screen, f"1 - Pour All Chemical")
        # text_print.indent()
        text_print.tprint(screen, f"2 - Vacumm")
        # text_print.indent()
        text_print.tprint(screen, f"3 - Servo Rotate")
        # text_print.indent()
        text_print.tprint(screen, f"4 - Servo Direction Toggle")
        # text_print.indent()
        text_print.tprint(screen, f"5 - Auger Down With Rotation")
        # text_print.indent()
        text_print.tprint(screen, f"6 - Auger UP")
        # text_print.indent()
        text_print.tprint(screen, f"7 - Auger Rotation For Deposition")
        # text_print.indent()
        text_print.tprint(screen, f"8 - Sensor Suite Up")
        # text_print.indent()
        text_print.tprint(screen, f"9 - Sensor Suite Down")
        # text_print.indent()
        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
        # Limit to 30 frames per second.

        print('connected by', addr)
        # Set up a timer and interval to send data
        timer = 0
        interval = 10  # Send data every 0.1 seconds
        running = True
        pygame.key.get_focused()

        while running:
            # Check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_1:
                        print("Key 1 has been pressed")
                        kv = 1
                    elif event.key == pygame.K_2:
                        print("Key 2 has been pressed")
                        kv = 2
                    elif event.key == pygame.K_3:
                        print("Key 3 has been pressed")
                        kv = 3
                    elif event.key == pygame.K_4:
                        print("Key 4 has been pressed")
                        kv = 4
                    elif event.key == pygame.K_5:
                        print("Key 5 has been pressed")
                        kv = 5
                    elif event.key == pygame.K_6:
                        print("Key 6 has been pressed")
                        kv = 6
                    elif event.key == pygame.K_7:
                        print("Key 7 has been pressed")
                        kv = 7
                    elif event.key == pygame.K_8:
                        print("Key 8 has been pressed")
                        kv = 8
                    elif event.key == pygame.K_9:
                        print("Key 9 has been pressed")
                        kv = 9
                    if event.key == pygame.K_SPACE:
                        print("Key SPACE has been pressed")
                        if driveMode == 0:
                            driveMode=1
                        else:
                            driveMode=0
                elif event.type == pygame.JOYHATMOTION:
                    if event.value[0] == -1:
                        print("D_PAD_LEFT has been pressed")
                        resetValue = 1
                    elif event.value[0] == 0:
                        resetValue = 0
                elif event.type == pygame.KEYUP:
                    # if event.key == pygame.K_1 or event.key == pygame.K_2:
                    kv = 0
                    Reset = 0
            # Update the timer and send data
            timer += 1 / 60  # 1/60 seconds have passed
            if timer >= interval:
                # The interval has passed, reset the timer and send data
                timer = 0
                R1 = controller.get_button(GEARUP)
                L1 = controller.get_button(GEARDOWN)
                if R1:
                    # Check for debouncing
                    if pygame.time.get_ticks() - last_press_time > debounce_time:
                        if Gear < 9:
                            Gear += 1
                            last_press_time = pygame.time.get_ticks()
                if L1:
                    # Check for debouncing
                    if pygame.time.get_ticks() - last_press_time > debounce_time:
                        if Gear > 0:
                            Gear -= 1
                            last_press_time = pygame.time.get_ticks()

                pygame.event.pump()
                left_joystick_0 = controller.get_axis(AXIS_LEFT_STICK_X)
                left_joystick_x_0 = int(
                    map(left_joystick_0, -1, 1, 1023, -1023))
                left_joystick_x = str(left_joystick_x_0)
                left_joystick_y_1 = (map(controller.get_axis(
                    AXIS_LEFT_STICK_Y), -1, 1, -1023, 1023))
                left_joystick_y = str(left_joystick_y_1)
                data = output_string.format(
                    Gear=Gear, left_joystick_x=left_joystick_x, left_joystick_y=left_joystick_y, kv=kv, resetValue=resetValue, driveMode=driveMode, E=0)
                print(data)
                # text_surface = font.render({data}, True, (255, 255, 255))
                # screen.blit(text_surface, (100, 100))

                # pygame.display.update()
                s.sendto(data.encode(), (addr))

# Quit pygame
pygame.quit()