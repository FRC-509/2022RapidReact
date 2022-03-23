package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TrajectoryBuilderWrapper {
    private static final TrajectoryConfig config = new TrajectoryConfig(SDSConstants.MAX_VELOCITY_METERS_PER_SECOND, 3.0d).setKinematics(RobotContainer.s_swerveDrive.getKinematics());
    private Trajectory trajectory;
    private Path trajectoryPath;

    public TrajectoryBuilderWrapper(String json_path) {
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json_path);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch (Exception e) {
            System.out.println(String.format("[TrajectoryBuilder] Unable to find trajectory path: %s.", json_path));
        }
    }

    public SwerveControllerCommand getSwerveControllerCommand() {
        return new SwerveControllerCommand(
            trajectory,
            RobotContainer.s_swerveDrive::getPose,
            RobotContainer.s_swerveDrive.getKinematics(),
            RobotContainer.s_swerveDrive.xController,
            RobotContainer.s_swerveDrive.yController,
            RobotContainer.s_swerveDrive.thetaController,
            RobotContainer.s_swerveDrive::setModuleStates,
            RobotContainer.s_swerveDrive
        );
    }

    // Returns a Command that sets the position of the robot to the initial pose in the trajectory, follows the trajectory, and stops all of the swerve modules.
    public Command getPathFollowingCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.s_swerveDrive.setPose(trajectory.getInitialPose())),
            getSwerveControllerCommand(),
            new InstantCommand(() -> RobotContainer.s_swerveDrive.stopAllModules())
        );
    }

}
