// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import static frc.robot.Constants.ModuleConstants.kDriveGearRatio;
import static frc.robot.Constants.ModuleConstants.kTurnGearRatio;

public class SwerveModule {
  private final Spark m_driveMotor;
  private final Spark m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  private EncoderSim m_driveEncoderSim;
  private EncoderSim m_turningEncoderSim;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // Using FlywheelSim as a stand-in for a simple motor
  private final FlywheelSim m_turnMotorSim = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(ModuleConstants.kvVoltSecondsPerMeter, ModuleConstants.kaVoltSecondsSquaredPerMeter),
          DCMotor.getFalcon500(1),
          kTurnGearRatio
  );

  private final FlywheelSim m_driveMotorSim = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
          DCMotor.getFalcon500(1),
          kDriveGearRatio
  );

  private double m_driveOutput;
  private double m_turnOutput;

  private double m_simTurnEncoderDistance;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderPorts,
      int[] turningEncoderPorts,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {

    m_driveMotor = new Spark(driveMotorChannel);
    m_turningMotor = new Spark(turningMotorChannel);

    this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

    this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    this.m_driveEncoderSim = new EncoderSim(m_driveEncoder);

    this.m_turningEncoderSim = new EncoderSim(m_turningEncoder);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
    m_driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    m_turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(m_driveOutput);
    m_turningMotor.set(m_turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }

  /** Simulate the SwerveModule */
  public void simulationPeriodic(double dt) {
    m_turnMotorSim.setInputVoltage(m_turnOutput / ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond * RobotController.getBatteryVoltage());
    m_driveMotorSim.setInputVoltage(m_driveOutput / Constants.DriveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());

    m_turnMotorSim.update(dt);
    m_driveMotorSim.update(dt);

    // Calculate distance traveled using RPM * dt
    m_simTurnEncoderDistance += m_turnMotorSim.getAngularVelocityRPM() * dt;
    m_turningEncoderSim.setDistance(m_simTurnEncoderDistance);
    m_turningEncoderSim.setRate(m_turnMotorSim.getAngularVelocityRPM());

    m_driveEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRPM());
  }
}
