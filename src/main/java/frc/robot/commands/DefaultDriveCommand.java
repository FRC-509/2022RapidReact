package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(RobotContainer.s_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        
        RobotContainer.s_drivetrainSubsystem.drive(
                // new ChassisSpeeds(
                //     m_translationXSupplier.getAsDouble(),
                //     m_translationYSupplier.getAsDouble(),
                //     m_rotationSupplier.getAsDouble()
                // )
                
                ChassisSpeeds.fromFieldRelativeSpeeds(
                     m_translationXSupplier.getAsDouble(),
                     m_translationYSupplier.getAsDouble(),
                     m_rotationSupplier.getAsDouble(),
                    RobotContainer.s_drivetrainSubsystem.getGyroscopeRotation()
                )
                
        );
        
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
