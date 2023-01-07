# 2023-public
https://www.thebluealliance.com/team/7439

## Naming Stuff / Styling

### 1 - kebab-case in module names. (Evet, ismini gercekten bu koymuslar :O)
```
touch robot-container.py
```
### 2 - camelCase for variables.
```
self.frontLeftMotor = ctre.WPI_VictorSPX(2)
```
### 3 - CamelCase for methods/functions.
```
    def myMethod(self):
        pass
```
### 4 - PascalCase for Classes.
```
    class MyRobot(wpilib.TimedRobot):
    
        def robotInit(self):
            pass
```
### 5 - Import subsystem/command classes using "from _ import _".
```
from robot-container import RobotContainer
from commands.pid-command-example import PidCommandExample
```
### 6 - Import frequently used classes/methods using "from _ import _".
```
from wpilib import SmartDashboard
```
### 7 - Format with black.
```
~/2023-public$ black *
```
