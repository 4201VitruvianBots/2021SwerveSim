# 2021SwerveSim
This repository contains two code examples on how to implement a working swerve sim using the 2021 WPILib libraries. This was done utilizing the FlywheelSim object as a model for the swerve motors and then combining them together to make a somewhat realizitic model for a swerve drive. This can operate the robot using both Autonomous and Teleop Code. The only limitation this model has is that you cannot define a trajectory that utilizes a moving heading for the swerve bot to use (e.g. strafe/orbit a target). Otherwise, for most teams attempting to develop swerve code, this should cover 95% of their needs.

More details on how this was developed can be found in the [ChiefDelphi post](https://www.chiefdelphi.com/t/simulating-a-swerve-drive-with-the-2021-wpilib-libraries/393534) we made.

See the [wiki](https://github.com/4201VitruvianBots/2021SwerveSim/wiki) on this repo for more details on how to implement this onto your own code.

## WPILib_SwerveControllerCommand
This is an example template that uses the WPILib SwerveControllerCommand as the baseline to implement a swerve sim. This tries to make as little changes as possible while also following the WPILib style of making the code as generic as possible to make it easier to adapt for teams using hardware/libraries from different vendors.

## Swerve2021
This is a snapshot of our 2021 Swerve Code. The main repo is [TRex2021](https://github.com/4201VitruvianBots/TRex2021), but since it is currently in development, we've opted to make a snapshot of it to make it easier for other teams to use.

### Swerve Hardware Specs
* SDS MKIII Modules with Falcon 500 motors
* Geared for 13.6 ft/s (8.16:1)
* Steering Gear ratio is 12.8:1 (See Patrick's post on [ChiefDelphi](https://www.chiefdelphi.com/t/sds-mk3-swerve-module/388331/60))

Note: We currently do not have CANCoders so it does not appear in the code. We plan on implementing them once we have the time/resources to do so. Otherwise, we have not noticed any significant issues running the swerve bot using the Falcon 500's internal encoders.
