package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLightWrapper;

public class AimbotAndShoot extends SequentialCommandGroup {
    
    private static final double TARGET_DISTANCE = (73.5d / Math.tan(35.0d));

    private static boolean isCloseToZero(double in, double threshold) {
        return in < threshold && in > -threshold;
    }

    public AimbotAndShoot() {
      addCommands(
        new DefaultDriveCommand(() -> 0.0,() -> 0.0, () -> -LimeLightWrapper.getSkewAngle()).withInterrupt( () -> isCloseToZero(LimeLightWrapper.getSkewAngle(), 0.1)),
        // keep turning until skew angle is 0 
        new DefaultDriveCommand(() -> 0.0,() -> TARGET_DISTANCE-LimeLightWrapper.getDistanceToHighTarget(), () -> 0.0).withInterrupt( () -> isCloseToZero(TARGET_DISTANCE - LimeLightWrapper.getDistanceToHighTarget(), 0.1) ),
        // keep moving until distance is equal to target
        new ShooterCommand(true).withTimeout(5000)
        // shoot for 5 seconds
        );
    }
  }