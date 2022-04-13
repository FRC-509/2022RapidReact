package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class AutoDriveCommand extends CommandBase {

    // private final DoubleSupplier m_translationXSupplier;
    // private final DoubleSupplier m_translationYSupplier;
    // private final DoubleSupplier m_rotationSupplier;
    private final DoubleSupplier m_targetX;
    private final DoubleSupplier m_targetY;
    private final DoubleSupplier m_targetAngle;

    private static double initXPos = RobotContainer.s_drivetrainSubsystem.getPose().getX();
    private static double initYPos = RobotContainer.s_drivetrainSubsystem.getPose().getY();
    private static double initAngle = RobotContainer.s_drivetrainSubsystem.m_pigeon.getYaw();
    
    private static double xPos; 
    private static double yPos;
    private static double aPos;

    private static double xError;
    private static double yError;
    private static double aError;

    private final double xyKp = 1;
    private final double aKp = 0.5;

    public AutoDriveCommand(DoubleSupplier targetX,
                            DoubleSupplier targetY,
                            DoubleSupplier targetAngle) {
        this.m_targetX = targetX;
        this.m_targetY = targetY;
        this.m_targetAngle = targetAngle;
        addRequirements(RobotContainer.s_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        xPos = RobotContainer.s_drivetrainSubsystem.getPose().getX();
        yPos = RobotContainer.s_drivetrainSubsystem.getPose().getY();
        aPos = RobotContainer.s_drivetrainSubsystem.m_pigeon.getYaw();
        
        xError = ((xPos - initXPos) - m_targetX.getAsDouble());
        yError = ((yPos - initYPos) - m_targetY.getAsDouble());
        aError = ((aPos - initAngle) - m_targetAngle.getAsDouble());
        aError = 0;
        // drive until death
        if (!(
                (
                 (xError < 0.5) && (xError > -0.5)
                )
                &&
                (
                 (yError < 0.5) && (yError > -0.5)
                )
                &&
                (
                 (aError < 5) && (aError > -5)
                )
            )
            )
        {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        RobotContainer.s_drivetrainSubsystem.drive(
                // new ChassisSpeeds(
                //     m_translationXSupplier.getAsDouble(),
                //     m_translationYSupplier.getAsDouble(),
                //     m_rotationSupplier.getAsDouble()
                // )
                
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xError*xyKp,
                    yError*xyKp,
                    aError*aKp,
                    RobotContainer.s_drivetrainSubsystem.getGyroscopeRotation()
                )
                
        );
        } else {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
