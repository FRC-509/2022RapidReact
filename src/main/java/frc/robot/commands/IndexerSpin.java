package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class IndexerSpin extends CommandBase {

  private final DoubleSupplier m_speedSupplier;

  public IndexerSpin(DoubleSupplier speedSupplier) {
    m_speedSupplier = speedSupplier;
    addRequirements(RobotContainer.getIndexer());
  }

  // Default Constructor. Sets speed supplier to always return 0.3.
  public IndexerSpin() {
    m_speedSupplier = () -> 0.3d;
    addRequirements(RobotContainer.getIndexer());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getIndexer().moveTheThing(m_speedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getIndexer().moveTheThing(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
