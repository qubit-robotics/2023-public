import rev


class Qubit_CANSparkMax(rev.CANSparkMax):
    """
    Seems like rev doesn't provide a way to set velocity values manually through code, this is a lazy solution.
    """

    simVelocity = 0

    def setSimVelocity(self, velocity):
        self.simVelocity = velocity

    def getSimVelocity(self):
        return self.simVelocity
