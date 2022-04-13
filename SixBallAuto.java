package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.TrajectoryBuilderWrapper;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeSpin;

public class SixBallAuto extends SequentialCommandGroup{

    public SixBallAuto() {
        TrajectoryBuilderWrapper wrapper;

        wrapper = new TrajectoryBuilderWrapper("paths/literallyDriveForward.wpilib.json");
        Command literallyDriveForward = wrapper.getPathFollowingCommand();

        wrapper = new TrajectoryBuilderWrapper("paths/DriveOffTheDamnTarmac.wpilib.json");
        Command driveOffTarmac = wrapper.getPathFollowingCommand();

        wrapper = new TrajectoryBuilderWrapper("paths/ThisIsSomeFuckShit.wpilib.json");
        Command driveToThirdBall = wrapper.getPathFollowingCommand();

        // wrapper = new TrajectoryBuilderWrapper("paths/DriveToHumanPlayerStation.wpilib.json");
        // SwerveControllerCommand driveToHumanPlayerStation = wrapper.getSwerveControllerCommand();

        // wrapper = new TrajectoryBuilderWrapper("paths/DriveToSixthBall.wpilib.json");
        // SwerveControllerCommand driveToSixthBall = wrapper.getSwerveControllerCommand();

        // new IntakeSpin().withTimeout(1.0d),
        // new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> 2.7d).withTimeout(1.2),
        // driveToThirdBall,
        // driveToHumanPlayerStation,
        // driveToSixthBall
        addCommands(
            literallyDriveForward
        );
    }

}
