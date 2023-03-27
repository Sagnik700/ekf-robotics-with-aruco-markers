from tkinter.tix import Tree
import ev3_dc as ev3

class Claw:
    
    def __init__(self, ev3_obj):
        self.motor = ev3.Motor(ev3.PORT_B, ev3_obj=ev3_obj)
        self.is_down = False

    def grab(self):
        """Moves claw down to grab a block

        Returns:
            bool: success
        """
        if not self.is_down:
            self.motor.start_move_by(-140, speed=20)        
            self.is_down = True
            return True
        return False

    def release(self):
        """Moves claw up to release a block

        Returns:
            bool: success
        """
        if self.is_down:
            self.motor.start_move_by(140, speed=20)
            self.is_down = False
            return True
        return False
