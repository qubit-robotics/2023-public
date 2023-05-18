import wpilib
from photonvision import PhotonCamera
import wpimath.controller
from wpilib import SmartDashboard
import commands2
class Tape(commands2.PIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                6,
                0,
                0,
                0
            ),
            0
        )
        self.cam = PhotonCamera("camera")
    def enable(self):
        pass
    def disable(self):
        pass
    def _useOutput(self, output: float, setpoint: float) -> None:
        print("tape pıd output voltage", output)
        SmartDashboard.putNumber("tapePIDoutput", output)
        return output
    def _getTapeOUTPUT(self) -> float:
        targets = self.cam.getLatestResult()  # Son PhotonVision sonuçlarını al

        if targets is not None and len(targets) > 0:
            closestTarget = min(targets, key=lambda target: abs(target.x))  # En yakın hedefi bul
            output = self._useOutput(closestTarget.x)  # PID kontrolcüsünü kullanarak çıktı hesapla
            return output  # Sürüş motoruna çıktıyı uygula
        else:
            return 0  # Hedef bulunamadığında motoru durdur

