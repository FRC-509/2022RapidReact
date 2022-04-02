package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeSpit extends CommandBase{
    public IntakeSpit() {
        addRequirements(RobotContainer.s_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.s_indexer.moveTheThing(-0.2);
        RobotContainer.s_intake.spin(-1.0d, Value.kReverse);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interupted) {
        RobotContainer.s_indexer.moveTheThing(0.0d);
        RobotContainer.s_intake.spin(0.0d, Value.kForward);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
