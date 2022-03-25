package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeSpin extends CommandBase{
    public IntakeSpin() {
        addRequirements(RobotContainer.s_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.s_intake.spin(1.2d, Value.kReverse);
        RobotContainer.s_indexer.moveTheThing(.25);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interupted) {
        RobotContainer.s_intake.spin(0.0d, Value.kForward);
        RobotContainer.s_indexer.moveTheThing((0.0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
