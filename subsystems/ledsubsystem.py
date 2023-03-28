import wpilib
import commands2

class LedSubsystem(commands2.SubsystemBase):

    kLEDBuffer = 48

    def __init__(self) -> None:
        self.led = wpilib.AddressableLED(3)

        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(self.kLEDBuffer)]

        self.rainbowFirstPixelHue = 0

        # Default to a length of 60, start empty output
        # Length is expensive to set, so only set it once, then just update data
        self.led.setLength(self.kLEDBuffer)

        # Set the data
        self.led.setData(self.ledData)
        self.led.start()

        super().__init__()
    
    def setRainbow(self) -> None:

        self.rainbow()

        self.led.setData(self.ledData)

    def rainbow(self):
        # For every pixel
        for i in range(self.kLEDBuffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / self.kLEDBuffer)) % 180

            # Set the value
            self.ledData[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 3

        # Check bounds
        self.rainbowFirstPixelHue %= 180