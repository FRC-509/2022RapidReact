package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeUp extends CommandBase {
  public IntakeUp() {
    addRequirements(RobotContainer.s_intake);
  }
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
    {//RobotContainer.s_intake.upDown(Value.kOff);
      RobotContainer.s_intake.upDown();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_intake.upDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
