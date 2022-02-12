
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  private final DriveTrain m_driveTrain;
  private GenericHID m_logiController = new GenericHID(0);
  
  public TankDrive(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_driveTrain.drive(m_logiController.getRawAxis(1), m_logiController.getRawAxis(4));
  }

  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0,0);
  }
  
  @Override
  public boolean isFinished(){
    return false;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
