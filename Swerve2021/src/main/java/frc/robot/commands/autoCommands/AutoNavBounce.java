package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

//import frc.robot.utils.TrajectoryUtils;

public class AutoNavBounce extends SequentialCommandGroup {
    public AutoNavBounce(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypointsA = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
        };
        Pose2d[] waypointsB = {
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-250))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-240))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
        };
        Pose2d[] waypointsC = {
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
        };
        Pose2d[] waypointsD = {
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180)))
        };
        Pose2d startPosition = waypointsA[0];

        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
                new SetDriveNeutralMode(swerveDrive, true)
        );

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
        config.setReversed(false);


        var trajectoryA = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsA.clone()), config);
        var trajectoryC = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsC.clone()), config);
        config.setReversed(true);
        var trajectoryB = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsB.clone()), config);
        var trajectoryD = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsD.clone()), config);

        var trajectoryStates = new ArrayList<Pose2d>();
        trajectoryStates.addAll(trajectoryA.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryB.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryC.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryD.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryA, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryB, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryC, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryD, ()-> new Rotation2d()));

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false)));// Run path following command, then stop at the end.
    }
}
