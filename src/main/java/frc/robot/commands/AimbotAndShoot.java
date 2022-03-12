package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLightWrapper;

public class AimbotAndShoot extends SequentialCommandGroup {

  private static boolean isCloseToZero(double in, double threshold) {
    return in < threshold && in > -threshold;
  }

  public AimbotAndShoot() {
    addCommands(
      new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> -LimeLightWrapper.getX()).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getX(), 0.1)),
      // keep turning until horizontal offset is 0 
      new DefaultDriveCommand(() -> 0.0,() -> 1.0, () -> 0.0).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getY(), 0.1) ),
      // keep moving until vertical offset is 0
      new ShooterCommand(true).withTimeout(5000)
      // shoot for 5 seconds
    );
  }
}