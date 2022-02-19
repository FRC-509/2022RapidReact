package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeSpin extends CommandBase{
    private boolean m_spinForward;

    public IntakeSpin(boolean _spinForward) {
        m_spinForward = _spinForward;
        addRequirements(RobotContainer.m_intake);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_spinForward) {
            RobotContainer.m_intake.spin(-.25);
        }
        else {
            RobotContainer.m_intake.spin(.25);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interupted) {
        RobotContainer.m_intake.spin(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
