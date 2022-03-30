package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;

public class HolonomicSwerveControllerCommand extends CommandBase {

  private HolonomicDriveController m_holonomicController;
  private Trajectory trajectory;
  private double startTime;
  private int stateNumber;
  
  public HolonomicSwerveControllerCommand(Trajectory trajectory) {
    m_holonomicController = RobotContainer.s_drivetrainSubsystem.getHolonomicDriveController();
    this.trajectory = trajectory;
    stateNumber = 0;
  }

  @Override
  public void initialize() {
	  startTime = Timer.getFPGATimestamp();
  }
  
  @Override
  public void execute() {
    double trajTime = 0.0;
    for(int idx = stateNumber; idx < trajectory.getStates().size(); idx++) {
      if(trajectory.getStates().get(idx).timeSeconds < Timer.getFPGATimestamp() - startTime) {
        trajTime = trajectory.getStates().get(idx).timeSeconds;
        stateNumber = idx;
        break;
      }
    }
    
    Trajectory.State nextState = trajectory.sample(trajTime);
    ChassisSpeeds adjustedSpeeds = m_holonomicController.calculate(RobotContainer.s_drivetrainSubsystem.getPose(), nextState, nextState.poseMeters.getRotation());
    RobotContainer.s_drivetrainSubsystem.drive(adjustedSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < Timer.getFPGATimestamp() - startTime;
  } 
}