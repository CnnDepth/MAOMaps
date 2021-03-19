import habitat
from habitat.sims.habitat_simulator.actions import HabitatSimActions
import keyboard

class KeyboardAgent(habitat.Agent):
    def __init__(self):
        self.speed = 0.
        self.twist = 1.

    def reset(self):
        pass

    def get_actions_from_keyboard(self):
        keyboard_commands = [HabitatSimActions.MOVE_FORWARD] * int(self.speed)
        if keyboard.is_pressed('left'):
            keyboard_commands += [HabitatSimActions.TURN_LEFT] * max(int(self.twist), 1)
        if keyboard.is_pressed('right'):
            keyboard_commands += [HabitatSimActions.TURN_RIGHT] * max(int(self.twist), 1)
        if keyboard.is_pressed('up'):
            self.speed += 0.1
        if keyboard.is_pressed('down'):
            self.speed = max(self.speed - 0.2, 0)
        if keyboard.is_pressed('s'):
            self.speed = 0
        if keyboard.is_pressed('e'):
            self.twist += 0.2
        if keyboard.is_pressed('d'):
            self.twist = max(self.twist - 0.2, 0)
        return keyboard_commands

    def act(self, observations, env):
        # receive command from keyboard and move
        actions = self.get_actions_from_keyboard()
        if len(actions) > 0:
            for action in actions[:-1]:
                env.step(action)
        if len(actions) > 0:
            return actions[-1]
        else:
            return HabitatSimActions.STOP