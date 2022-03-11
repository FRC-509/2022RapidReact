
package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorCommand extends CommandBase {
  private final DoubleSupplier m_armYSupplier;
  private final DoubleSupplier m_elevatorYSupplier;

  public ElevatorCommand(DoubleSupplier armYSupplier, DoubleSupplier elevatorYSupplier) {
    m_armYSupplier = armYSupplier;
    m_elevatorYSupplier = elevatorYSupplier;
    addRequirements(RobotContainer.s_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_elevator.moveElevator(m_elevatorYSupplier.getAsDouble());
    RobotContainer.s_elevator.moveArm(m_armYSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_elevator.elevatorMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

