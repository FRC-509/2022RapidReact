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
import frc.robot.commands.HolonomicSwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryBuilderWrapper {
    private static final TrajectoryConfig config = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3.0d).setKinematics(RobotContainer.s_drivetrainSubsystem.getKinematics());
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

    public SwerveControllerCommand getSwerveControllerCommand() {
        return new SwerveControllerCommand(
            trajectory,
            RobotContainer.s_drivetrainSubsystem::getPose,
            RobotContainer.s_drivetrainSubsystem.getKinematics(),
            RobotContainer.s_drivetrainSubsystem.getXController(),
            RobotContainer.s_drivetrainSubsystem.getYController(),
            RobotContainer.s_drivetrainSubsystem.getThetaController(),
            RobotContainer.s_drivetrainSubsystem::setModuleStates,
            RobotContainer.s_drivetrainSubsystem
        );
    }

    public HolonomicSwerveControllerCommand getHolonomicSwerveControllerCommand() {
        return new HolonomicSwerveControllerCommand(trajectory);
    }
}
