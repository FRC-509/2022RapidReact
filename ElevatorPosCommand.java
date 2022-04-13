package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class ElevatorPosCommand extends CommandBase {
  private final DoubleSupplier m_armPosSupplier;
  private final DoubleSupplier m_elevatorPosSupplier;

  private double elevatorPos;
  private double armPos;
  private boolean exit = false;

  public ElevatorPosCommand(DoubleSupplier armPosSupplier, DoubleSupplier elevatorPosSupplier) {
    m_armPosSupplier = armPosSupplier;
    m_elevatorPosSupplier = elevatorPosSupplier;
    addRequirements(RobotContainer.s_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_elevator.moveElevatorPos(m_elevatorPosSupplier.getAsDouble());
    RobotContainer.s_elevator.moveArmPos(m_armPosSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorPos = RobotContainer.s_elevator.elevatorMotor.getSelectedSensorPosition();
    armPos = RobotContainer.s_elevator.armMotor1.getSelectedSensorPosition();
    if (
      (elevatorPos > m_elevatorPosSupplier.getAsDouble() - 5000) &&
      (elevatorPos < m_elevatorPosSupplier.getAsDouble() + 5000) &&
      (armPos > m_armPosSupplier.getAsDouble() - 5000) &&
      (armPos < m_armPosSupplier.getAsDouble() + 5000)
       ) 
     {
      exit = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_elevator.moveElevator(0.0d);
    RobotContainer.s_elevator.moveArm(0.0d);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit;
  }
}