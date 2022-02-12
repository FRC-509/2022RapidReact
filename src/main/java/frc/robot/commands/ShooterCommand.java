package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    private final ShooterSubsystem  m_shooterSubsystem;
    
    public ShooterCommand(ShooterSubsystem subsystem){
      m_shooterSubsystem = subsystem;
      
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_shooterSubsystem.shoot(1.0);
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