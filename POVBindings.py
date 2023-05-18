import commands2.button

class POVBindings:

    def POVUpBinding(getPOV: commands2.button.CommandJoystick.getPOV):

        if getPOV == 0:
            return True
        else:
            return False

    def POVDownBinding(getPOV: commands2.button.CommandJoystick.getPOV):

        if getPOV == 180:
            return True
        else:
            return False