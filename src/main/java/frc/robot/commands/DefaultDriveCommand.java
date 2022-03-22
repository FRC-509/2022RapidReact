package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(RobotContainer.s_swerveDrive);
    }

    @Override
    public void execute() {
        // Pass false for robot-oriented movement, and true for field-oriented movement.
        RobotContainer.s_swerveDrive.drive(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble(),
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_swerveDrive.drive(0.0, 0.0, 0.0, false);
    }
}
