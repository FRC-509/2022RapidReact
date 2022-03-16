package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterCommand extends CommandBase{
    private boolean m_far;
    private boolean m_shootForTime; 
    private double m_timeToShoot;
    private Timer m_timer;

    public ShooterCommand(boolean far){
      m_far = far;
      m_shootForTime = false;
      addRequirements(RobotContainer.s_shooterSubsystem);
    }

    public ShooterCommand(boolean far, double time){
        m_far = far;
        m_shootForTime = true;
        m_timeToShoot = time;
        m_timer = new Timer();
        m_timer.start();
        addRequirements(RobotContainer.s_shooterSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (m_far) {
            RobotContainer.s_shooterSubsystem.shoot(0.5);
        }
        else
            RobotContainer.s_shooterSubsystem.shoot(0.3);
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.s_shooterSubsystem.shoot(0.0);
    }
    
    @Override
    public boolean isFinished(){
        if (m_shootForTime)
            return m_timer.get() > m_timeToShoot;
        else
            return false;
    }
}