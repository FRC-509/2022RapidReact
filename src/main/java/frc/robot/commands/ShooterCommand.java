package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    private final ShooterSubsystem  m_shooterSubsystem;
    private final boolean m_far;

    public ShooterCommand(ShooterSubsystem subsystem, boolean far){
      m_shooterSubsystem = subsystem;
      m_far = far;
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (m_far)
            m_shooterSubsystem.shoot(0.5);
        else
            m_shooterSubsystem.shoot(0.3);
    }

    @Override
    public void end(boolean interrupted){
       m_shooterSubsystem.shoot(0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}