package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_robotRelativeSupplier;

    public DefaultDriveCommand(DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier driveRobotRelative) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_robotRelativeSupplier = driveRobotRelative;

        addRequirements(RobotContainer.getDriveTrainSubsystem());
    }

    public DefaultDriveCommand(DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_robotRelativeSupplier = () -> true;

        addRequirements(RobotContainer.getDriveTrainSubsystem());
    }

    @Override
    public void execute() {
        // Drive relative to the robot.
        if (m_robotRelativeSupplier.getAsBoolean()) {
            RobotContainer.getDriveTrainSubsystem().drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble()
                )
            );
        }
        // Drive relative to the field.
        else {
            RobotContainer.getDriveTrainSubsystem().drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble(),
                    RobotContainer.getDriveTrainSubsystem().getGyroscopeRotation()
                )
            );
        }        
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getDriveTrainSubsystem().drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
