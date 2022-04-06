package frc.robot;

import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TrajectoryBuilderWrapper {
    private static final TrajectoryConfig config = new TrajectoryConfig(DriveTrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3.0d).setKinematics(RobotContainer.getDriveTrainSubsystem().getKinematics());
    private Trajectory trajectory;

    public TrajectoryBuilderWrapper(String json_path) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json_path);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch (Exception e) {
            System.out.println(String.format("[TrajectoryBuilder] Unable to find trajectory path: %s.", json_path));
        }
    }

    public TrajectoryBuilderWrapper(Pose2d initalPose, List<Translation2d> interiorWaypoints, Pose2d finalPose) {
        trajectory = TrajectoryGenerator.generateTrajectory(initalPose, interiorWaypoints, finalPose, config);
    }
    
    public SwerveControllerCommand getSwerveControllerCommand() {
        return new SwerveControllerCommand(
            trajectory,
            RobotContainer.getDriveTrainSubsystem()::getPose,
            RobotContainer.getDriveTrainSubsystem().getKinematics(),
            RobotContainer.getDriveTrainSubsystem().getXController(),
            RobotContainer.getDriveTrainSubsystem().getYController(),
            RobotContainer.getDriveTrainSubsystem().getThetaController(),
            RobotContainer.getDriveTrainSubsystem()::setModuleStates,
            RobotContainer.getDriveTrainSubsystem()
        );
    }

    // Returns a Command that sets the position of the robot to the initial pose in the trajectory, follows the trajectory, and stops all of the swerve modules.
    public Command getPathFollowingCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem().resetOdometry(trajectory.getInitialPose())),
            getSwerveControllerCommand(),
            new InstantCommand(() -> RobotContainer.getDriveTrainSubsystem().stopModules())
        );
    }
}
