// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

import static frc.robot.Constants.DriveConstants.*;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderPorts,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftDriveEncoderPorts,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderPorts,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightDriveEncoderPorts,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  private final SwerveModule[] m_swerveModules = {
          m_frontLeft,
          m_frontRight,
          m_rearLeft,
          m_rearRight
  };

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  Pose2d[] m_modulePose = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

  Field2d m_fieldSim = new Field2d();

  // Using a DifferentialDrivetrainSim to represent chassis rotation
  private DifferentialDrivetrainSim m_chassisSim = new DifferentialDrivetrainSim(
          Constants.DriveConstants.kDrivetrainPlant,
          Constants.DriveConstants.kDriveGearbox,
          Constants.ModuleConstants.kDriveGearRatio,
          Constants.DriveConstants.kTrackWidth,
          Constants.ModuleConstants.kWheelDiameterMeters / 2.0,
          null
  );

  // Simulation variables
  private double m_rotationInput;
  private double m_lastRotationSign;
  private double m_invertRotationInput;

  // To fake a rotation, we need to make the simulated chassis rotation speed equal to what we command it to be.
  // Use the formula for angular velocity
  // m_turnAdjustment = m_chassisSim.getRightVelocityMetersPerSecond() / (kTrackWidth / 2)
  private double m_rotationAdjustment = 1;

  private SwerveModuleState[] m_inputStates = {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
  };

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        new Rotation2d(getHeading()),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());

    // Update the poses for the swerveModules. Note that the order of rotating the position and then
    // adding the translation matters
    for (int i = 0; i < m_swerveModules.length; i++) {
      var modulePositionFromChassis = kModulePositions[i]
              .rotateBy(new Rotation2d(getHeading()))
              .plus(getPose().getTranslation());

      m_modulePose[i] = new Pose2d(modulePositionFromChassis, m_swerveModules[i].getState().angle);
    }

    m_fieldSim.setRobotPose(getPose());
    m_fieldSim.getObject("Swerve Modules").setPoses(m_modulePose);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Save rotation input for chassis rotation sim
    m_rotationInput = rot;
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_inputStates = swerveModuleStates;
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Save rotation input for chassis rotation sim
    m_rotationInput = kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond;

    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_inputStates = desiredStates;
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic(0.02);
    m_rearLeft.simulationPeriodic(0.02);
    m_frontRight.simulationPeriodic(0.02);
    m_rearRight.simulationPeriodic(0.02);

    // Fake chassis input by using rotation input / max rotation to get a [-1, 1] range, then multiply by battery voltage
    // and then the rotation adjustment value divided by 2 (To cut in half for both sides)
    double chassisInputVoltage = (m_rotationInput / Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond) *
            RobotController.getBatteryVoltage() * m_rotationAdjustment  / 2;

    // If the rotation changes due to SwerveDriveState.optimize(), you must rotate the chassis the other way.
    // You should only need to look at one of the modules for this.
    double currentRotationSign = Math.signum(m_inputStates[0].angle.getRadians());
    if(m_lastRotationSign != currentRotationSign){
      m_invertRotationInput = -m_invertRotationInput;
    }
    m_lastRotationSign = currentRotationSign;
    chassisInputVoltage *= m_invertRotationInput;

    m_chassisSim.setInputs(chassisInputVoltage, -chassisInputVoltage);
    m_chassisSim.update(0.02);

    m_gyroSim.setAngle(m_chassisSim.getHeading().getDegrees());
  }
}
