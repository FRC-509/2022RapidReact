package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeUpDown extends CommandBase {

    public IntakeUpDown() {
        addRequirements(RobotContainer.s_intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.s_intake.upDown();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_intake.upDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}