/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

    public static final double kMaxAngularSpeed = kMaxChassisRotationSpeed; // 3 meters per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private int navXDebug = 0;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getHeadingRotation2d());
//    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
//        getRotation(),
//        new Pose2d(),
//        kDriveKinematics,
//        VecBuilder.fill(0.1, 0.1, 0.1),
//        VecBuilder.fill(0.05),
//        VecBuilder.fill(0.1, 0.1, 0.1));

    PowerDistributionPanel m_pdp;

    private double m_trajectoryTime;
    private Trajectory currentTrajectory;

    /**
     * Just like a graph's quadrants
     * 0 is Front Left
     * 1 is Front Right
     * 2 is Back Left
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
        new SwerveModule(0, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), 0, true, false),
        new SwerveModule(1, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), 0, true, false), //true
        new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), 0, true, false),
        new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), 0, true, false) //true
    };

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);

    int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    double simYaw = 0;

    public SwerveDrive(PowerDistributionPanel pdp) {
        m_pdp = pdp;

        SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this);
        if (RobotBase.isSimulation()) {

        }
    }


    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return mNavX.getRate();
    }

    /**
     * Returns the heading of the robot in degrees.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeadingDegrees() {
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++){
            mSwerveModules[i].resetEncoders();
        }
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        mNavX.reset();
    }


    public SwerveModule getSwerveModule(int i) {
        return mSwerveModules[i];
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
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= kMaxSpeedMetersPerSecond;
        ySpeed *= kMaxSpeedMetersPerSecond;
        rot *= kMaxAngularSpeed;

        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getHeadingRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        ); //from 2910's code
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

        mSwerveModules[0].setDesiredState(swerveModuleStates[0]);
        mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
        mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
        mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    public void setSwerveDriveNeutralMode(boolean mode) {
        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setBrakeMode(mode);
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        mSwerveModules[0].setDesiredState(desiredStates[0]);
        mSwerveModules[1].setDesiredState(desiredStates[1]);
        mSwerveModules[2].setDesiredState(desiredStates[2]);
        mSwerveModules[3].setDesiredState(desiredStates[3]);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        m_odometry.update(
            getHeadingRotation2d(),
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        );
        // Update module positions based on the chassis' position, but keep the module heading
        for (int i = 0; i < mSwerveModules.length; i++) {
            var modulePositionFromChassis = modulePositions[i].rotateBy(getHeadingRotation2d()).plus(getPose().getTranslation());
            mSwerveModules[i].setPose(new Pose2d(modulePositionFromChassis, mSwerveModules[i].getHeading().plus(getHeadingRotation2d())));
        }

//        m_odometry.addVisionMeasurement(SimulationReferencePose.getRobotFieldPose(), Timer.getFPGATimestamp() + 0.2);

//        System.out.println("Swerve Pose: " + getPose());
//        System.out.println("Field Pose: " + SimulationReferencePose.getRobotFieldPose());
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        m_odometry.resetPosition(pose, rotation);

        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setPose(pose);
            mSwerveModules[i].resetEncoders();
        }
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("SwerveDrive","Chassis Angle",getHeadingDegrees());
        for(int i = 0; i < mSwerveModules.length; i++) {
            SmartDashboardTab.putNumber("SwerveDrive", "Swerve Module " + i + " Angle", mSwerveModules[i].getState().angle.getDegrees());
            SmartDashboardTab.putNumber("SwerveDrive", "Swerve Module " + i + " Speed", mSwerveModules[i].getState().speedMetersPerSecond);
        }

        SmartDashboardTab.putNumber("SwerveDrive", "X coordinate", getPose().getX());
        SmartDashboardTab.putNumber("SwerveDrive", "Y coordinate", getPose().getY());
    }

    @Override
    public void periodic() {
        sampleTrajectory();
        updateOdometry();
        updateSmartDashboard();
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = {
            mSwerveModules[0].getPose(),
            mSwerveModules[1].getPose(),
            mSwerveModules[2].getPose(),
            mSwerveModules[3].getPose()
        };
        return modulePoses;
    }

    @Override
    public void simulationPeriodic() {
        SwerveModuleState[] moduleStates = {
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        };

        var chassisSpeed = kDriveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        simYaw += chassisRotationSpeed * 0.02;
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
        angle.set(-Units.radiansToDegrees(simYaw));
    }

    private void sampleTrajectory() {
        if(DriverStation.getInstance().isAutonomous()) {
            try {
                var currentTrajectoryState = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime);

                System.out.println("Trajectory Time: " + (Timer.getFPGATimestamp() - startTime));
                System.out.println("Trajectory Pose: " + currentTrajectoryState.poseMeters);
                System.out.println("Trajectory Speed: " + currentTrajectoryState.velocityMetersPerSecond);
                System.out.println("Trajectory angular speed: " + currentTrajectoryState.curvatureRadPerMeter);
            } catch (Exception e) {

            }
        }

    }

    public void setTrajectoryTime(double trajectoryTime) {
        m_trajectoryTime = trajectoryTime;
    }

    double startTime;
    public void setCurrentTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        startTime = Timer.getFPGATimestamp();
    }
}
