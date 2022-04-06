package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimeLightWrapper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Elevator;

public class TraversalClimb extends SequentialCommandGroup {

  private static boolean isCloseToZero(double in, double threshold) {
    return in < threshold && in > -threshold;
  }


  public TraversalClimb() {

    addRequirements(RobotContainer.getElevator());
    addRequirements(RobotContainer.getDriveTrainSubsystem());

    Elevator elevator = RobotContainer.getElevator();
    DriveTrainSubsystem driveTrain = RobotContainer.getDriveTrainSubsystem();
    Pigeon2 gyro = driveTrain.getGyro();
    double startgyro = gyro.getPitch();

    System.out.println("[TraversalClimb] 509 Says: \"Hey, Let\'s Go Climbing!\"");
    
    addCommands(

      // move arms backward -21.8 degrees
      new InstantCommand( () -> elevator.setArmAngleDegrees(-21.8d), elevator),

      // fully extend elevator
      new InstantCommand( () -> elevator.fullyExtendElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() - Elevator.ELEVATOR_FULLY_EXTENDED, 1000)
      ),

      // drive forward to meet mid rung
      new DefaultDriveCommand(() -> 0.0,() -> 0.1, () -> 0.0, () -> true).withTimeout(0.01d),
      
      // fully retract elevator
      new InstantCommand( () -> elevator.fullyRetractElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() - Elevator.ELEVATOR_FULLY_RETRACTED, 1000)
      ),

      // move arms to -14.1 degrees (now both arms AND the elevator should be on the mid rung)
      new InstantCommand( () -> elevator.setArmAngleDegrees(-14.1d), elevator),
      
      // extend elevator halfway
      new InstantCommand( () -> elevator.fullyRetractElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() + 600000, 1000)
      ),
      
      // move arms to 18.7 degrees
      new InstantCommand( () -> elevator.setArmAngleDegrees(18.7d), elevator),

      // wait for alice to swing
      new WaitCommand(0.2), // gotta get this just right

      // fully extend elevator
      new InstantCommand( () -> elevator.fullyExtendElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() - Elevator.ELEVATOR_FULLY_EXTENDED, 1000)
      ),

      // wait for alice to swing towards high rung
      new WaitCommand(0.2), // gotta get this just right

      // fully retract elevator
      new InstantCommand( () -> elevator.fullyRetractElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() - Elevator.ELEVATOR_FULLY_RETRACTED, 1000)
      ),

      // move arms to -14.1 degrees (now both arms AND the elevator should be on the high rung)
      new InstantCommand( () -> elevator.setArmAngleDegrees(-14.1d), elevator),

      // at this point, we should be at the high rung

      // extend elevator halfway
      new InstantCommand( () -> elevator.fullyRetractElevator(), elevator)
      .withInterrupt( 
        () -> isCloseToZero(elevator.getElevatorPosition() + 600000, 1000)
      )

      

    );
  }
}