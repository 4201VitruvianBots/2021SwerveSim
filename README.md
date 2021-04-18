# 2021SwerveSim
This repository contains two code examples on how to implement a working swerve sim using the 2021 WPILib libraries.

## WPILib_SwerveControllerCommand
This is an example template that uses the WPILib SwerveControllerCommand as the baseline to implement a swerve sim. This tries to make as little changes as possible from the baseline while also following the WPILib style of making then code as generic as possible to make it easier to adapt for teams using different vendor motors.

## Swerve2021
This is a snapshot of our 2021 Swerve Code. The main repo is [TRex2021](https://github.com/4201VitruvianBots/TRex2021), but since it is currently in development, I've opted to make a snapshot of it to make it easier for other teams to look at it.

### Swerve Hardware Specs
* SDS MKIII Modules with Falcon 500 motors
* Geared for ~ 13.6 ft/s (8.16:1)

Note: We currently do not have CANCoders so it does not appear in the code. We plan on implementing them once we have the time/resources to do so. Otherwise, we have not noticed any significant issues running the swerve bot without an encoder directly to the swerve's rotation.

