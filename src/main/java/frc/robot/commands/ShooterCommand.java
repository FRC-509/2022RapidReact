package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    private final boolean m_far;

    public ShooterCommand(boolean far){
      m_far = far;
      addRequirements(RobotContainer.m_shooterSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (m_far)
            RobotContainer.m_shooterSubsystem.shoot(0.5);
        else
            RobotContainer.m_shooterSubsystem.shoot(0.3);
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.m_shooterSubsystem.shoot(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}