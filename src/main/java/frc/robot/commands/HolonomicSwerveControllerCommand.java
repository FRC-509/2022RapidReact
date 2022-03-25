package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;

public class HolonomicSwerveControllerCommand extends CommandBase {

  private HolonomicDriveController holonomicController;
  private Trajectory trajectory;
  private double startTime;
  private int stateNumber;
  
  public HolonomicSwerveControllerCommand(Trajectory trajectory) {
    holonomicController = new HolonomicDriveController(
		new PIDController(1, 0, 0), // X Controller
		new PIDController(1, 0, 0), // Y Controller
		new ProfiledPIDController(1, 0, 0, // Theta Controller
		new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2 * Math.PI))
	); // Constraints are maxVel and maxAccel both in radians

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
        trajTime = trajectory.getStates().get(idx - 1).timeSeconds;
        stateNumber = idx - 1;
        break;
      }
    }
       
    Trajectory.State nextState = trajectory.sample(trajTime);
    ChassisSpeeds adjustedSpeeds = holonomicController.calculate(DrivetrainSubsystem.m_odometer.getPoseMeters(), nextState, nextState.poseMeters.getRotation());
	
    RobotContainer.s_drivetrainSubsystem.drive(adjustedSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < Timer.getFPGATimestamp() - startTime;
  } 
}