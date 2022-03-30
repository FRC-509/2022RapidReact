package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class ShooterCommand extends CommandBase {
    private DoubleSupplier m_speed;
    
    public ShooterCommand(DoubleSupplier speed) {
        m_speed = speed;
        addRequirements(RobotContainer.s_shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.s_shooterSubsystem.shoot(.35);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_shooterSubsystem.shoot(0.0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}