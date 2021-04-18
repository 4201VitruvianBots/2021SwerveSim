package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.shooter.SetRPM;
import frc.robot.commands.shooter.ControlledFireNew;
import frc.robot.simulation.FieldSim;
import frc.robot.utils.TrajectoryUtils;

//import frc.robot.utils.TrajectoryUtils;

public class AccuracyChallenge extends SequentialCommandGroup {
    private int baseRPM = 3000;
    public AccuracyChallenge(SwerveDrive swerveDrive, Shooter shooter, Indexer indexer, FieldSim fieldSim, int index) {
        double[] distancesToDrive = {
                Units.inchesToMeters(180 + 6), // Green
                Units.inchesToMeters(120 + 6), // Yellow
                Units.inchesToMeters(120 - 6), // Blue
                Units.inchesToMeters(60 - 6) // Red
                // TODO: Add/subtract T-rex's length
            };
        addCommands(
            new SetOdometry(swerveDrive, new Pose2d()),
            new SetDriveNeutralMode(swerveDrive, true),
            new SetRPM(shooter, baseRPM),
            new DriveForwardDistance(swerveDrive, fieldSim, distancesToDrive[index]).andThen(() -> swerveDrive.drive(0, 0, 0, false)),
            //new InstantCommand(() -> shooter.setIdealRPM()),
            new ControlledFireNew(shooter, indexer).withTimeout(3),
            new SetRPM(shooter, baseRPM),
            new DriveBackwardDistance(swerveDrive, distancesToDrive[index]).andThen(() -> swerveDrive.drive(0, 0, 0, false))
        );
    }
}
