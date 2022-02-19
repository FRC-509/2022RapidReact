// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class AutonomousCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final SubsystemBase m_subsystem;
  private final Timer m_timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommand() {
    // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    while (m_timer.get() < 2.0) {
      new IntakeSpin(RobotContainer.getInstance().m_intake, true);
      new DefaultDriveCommand(RobotContainer.getInstance().m_drivetrainSubsystem, () -> 0, () -> 1, () -> 0);
      // spin intake and move on the y axis for a hot 2 seconds
      SmartDashboard.putString("Driving", "yes");
    }
    SmartDashboard.putString("Driving", "no");

    new DefaultDriveCommand(RobotContainer.getInstance().m_drivetrainSubsystem, () -> 0, () -> 0, () -> 180);
    // turn around
    // every now and then..
    new IndexerCommand(RobotContainer.getInstance().m_indexer);
    new ShooterCommand(RobotContainer.getInstance().m_shooterSubsystem, true);
    SmartDashboard.putString("Shooter", "yes");

    try {
      m_timer.wait((long)(20-m_timer.get()));
    } catch (InterruptedException e) {
      return;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
