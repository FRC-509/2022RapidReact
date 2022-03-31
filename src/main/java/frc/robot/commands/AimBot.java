package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLightWrapper;

public class AimBot extends SequentialCommandGroup {

  private static boolean isCloseToZero(double in, double threshold) {
    return in < threshold && in > -threshold;
  }

  public AimBot() {
    addCommands(
      // keep turning until target is visible
      new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> -1).withInterrupt( () -> LimeLightWrapper.hasTarget()),
      // keep turning until horizontal offset is 0 
      new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> LimeLightWrapper.getX()/3).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getX(), 0.1))    );
  }
}