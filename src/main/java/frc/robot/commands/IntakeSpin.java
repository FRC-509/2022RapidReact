package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeSpin extends CommandBase{

    private final DoubleSupplier m_speedSupplier;
    
    public IntakeSpin(DoubleSupplier speedSupplier) {
        m_speedSupplier = speedSupplier;
        addRequirements(RobotContainer.getIntake());
    }

    // Default Constructor. Sets speed supplier to always return 1.0.
    public IntakeSpin() {
        m_speedSupplier = () -> 1.0;
        addRequirements(RobotContainer.getIntake());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.getIntake().spin(m_speedSupplier.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interupted) {
        RobotContainer.getIntake().spin(0.0d);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
