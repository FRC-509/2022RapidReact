package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DefaultDriveCommand;

import java.util.function.DoubleSupplier;

public class DriveFeedback extends CommandBase {
  //inputting desired x and y distance/offsets + angle
  private final DoubleSupplier m_XSupplier;
  private final DoubleSupplier m_YSupplier;
  private final DoubleSupplier m_ASupplier;
  
  //XYDeadband constant [how close you want to get to a specific distance]
  private final double XYDeadband = 0.25;
  
  //XYkP constant
  private final double XYkP = 1.5;
  
  //ADeadband constant [how close you want to get to a specific heading, in degrees]
  private final double ADeadband = 15;
  
  //AkP constant
  private final double AkP = 0.035;
  
  //for storing the initial positions
  private double m_XInitial;
  private double m_YInitial;
  private double m_AInitial;
  
  //for storing the current positions
  private double m_X;
  private double m_Y;
  private double m_A;
    
  //for storing the error between initial and current positions
  private double m_XError;
  private double m_YError;
  private double m_AError;

  //jameson wang stupid exit strategy
  private boolean exit;
    
  public DriveFeedback(DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier ASupplier) {
    m_XSupplier = XSupplier;
    m_YSupplier = YSupplier;
    m_ASupplier = ASupplier;
    addRequirements(RobotContainer.s_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //setting initial positions
    m_XInitial = RobotContainer.s_drivetrainSubsystem.getPose().getX();
    m_YInitial = RobotContainer.s_drivetrainSubsystem.getPose().getY();
    m_AInitial = RobotContainer.s_drivetrainSubsystem.m_pigeon.getYaw();
    
    //unnecessary, just dumb to be here
    exit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get current robot positions
    m_X = RobotContainer.s_drivetrainSubsystem.getPose().getX();
    m_Y = RobotContainer.s_drivetrainSubsystem.getPose().getY();
    m_A = RobotContainer.s_drivetrainSubsystem.m_pigeon.getYaw();
    
    //updating error
    m_XError = -((m_X - m_XInitial) - m_XSupplier.getAsDouble());
    m_YError = -((m_Y - m_YInitial) - m_YSupplier.getAsDouble());
    m_AError = ((m_A - m_AInitial) - m_ASupplier.getAsDouble());
    
    //possibly drives to the distance???
    RobotContainer.s_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_XError * XYkP, m_YError * XYkP, m_AError * AkP, RobotContainer.s_drivetrainSubsystem.getGyroscopeRotation()));
    
    if (within(m_XError, XYDeadband) && within(m_YError, XYDeadband) && within(m_AError, ADeadband)){
      exit = true;
    }
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit;
  }

  private boolean within(double input, double deadband){
    boolean withi = false;
    if (input <= deadband && input >= -deadband){
      withi = true;
    }
    return withi;
  }
}