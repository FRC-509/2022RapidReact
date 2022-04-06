package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDown extends CommandBase {
  public IntakeDown() {
    addRequirements(RobotContainer.getIntake());
  }
  
  @Override
  public void initialize() {
    RobotContainer.getIntake().setSolenoidValue(Value.kReverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getIntake().setSolenoidValue(Value.kForward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
