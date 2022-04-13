package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

public class ShooterCommand extends CommandBase {
    private BooleanSupplier m_shooting;
    
    public ShooterCommand(BooleanSupplier shooting) {
        m_shooting = shooting;
        addRequirements(RobotContainer.s_shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
<<<<<<< HEAD
        RobotContainer.s_shooterSubsystem.shoot(m_shooting.getAsBoolean());
=======
        RobotContainer.s_shooterSubsystem.shoot(m_speed.getAsDouble());
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_shooterSubsystem.shoot(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}