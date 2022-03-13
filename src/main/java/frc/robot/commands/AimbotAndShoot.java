package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimeLightWrapper;

public class AimbotAndShoot extends SequentialCommandGroup {

  private static boolean isCloseToZero(double in, double threshold) {
    return in < threshold && in > -threshold;
  }

  public AimbotAndShoot() {
    addCommands(
      // keep turning until target is visible
      new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> -1).withInterrupt( () -> LimeLightWrapper.hasTarget()),
      // keep turning until horizontal offset is 0 
      new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> LimeLightWrapper.getX()/5).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getX(), 0.1)),
      // keep moving until vertical offset is 0
      new DefaultDriveCommand(() -> 0.0,() -> 1.0, () -> 0.0).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getY(), 0.1) )
      // shoot for 2 seconds
      // new ShooterCommand(true).withTimeout(2000)
    );
  }
}