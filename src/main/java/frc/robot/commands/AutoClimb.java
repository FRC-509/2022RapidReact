package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommand;

public class AutoClimb extends SequentialCommandGroup {
  
  
  public final double elevatorUp = 60000;
  public final double elevatorDown = -5000;
  
  public final double armVForward = 600000;
  public final double armForward = 580000;
  public final double armBackward = 100000;
  public final double armVBackward = 50000;
  
  public AutoClimb() {
    addRequirements(RobotContainer.s_elevator);
    addCommands(
      new SequentialCommandGroup(
        //pull up to mid rung
        new ElevatorPosCommand(() -> armVBackward, () -> elevatorDown),
        //arm on to mid rung
        new ElevatorPosCommand(() -> armBackward, () -> elevatorDown),
        //reach for high rung
        new ElevatorPosCommand(() -> armVForward, () -> elevatorUp),
        //elevator on to high rung
        new ElevatorPosCommand(() -> armForward, () -> elevatorUp),
        //pull up to high rung
        new ElevatorPosCommand(() -> armVBackward, () -> elevatorDown),
        //arm on to high rung
        new ElevatorPosCommand(() -> armBackward, () -> elevatorDown),
        //reach for traversal rung
        new ElevatorPosCommand(() -> armVForward, () -> elevatorUp),
        //elevator on to traversal rung
        new ElevatorPosCommand(() -> armForward, () -> elevatorUp),
        //pull up to traversal rung
        new ElevatorPosCommand(() -> armVBackward, () -> elevatorDown),
        //arm on to traversal rung
        new ElevatorPosCommand(() -> armBackward, () -> elevatorDown)
      )
    );
  }

}