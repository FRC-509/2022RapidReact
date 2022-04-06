package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTheWheels extends CommandBase {
  public ZeroTheWheels() {
    addRequirements(RobotContainer.getDriveTrainSubsystem());
  }
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getDriveTrainSubsystem().zeroTheWheels();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.getDriveTrainSubsystem().areTheWheelsZeroed();
  }
}
