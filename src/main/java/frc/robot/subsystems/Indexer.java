package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLightWrapper;
import frc.robot.RobotContainer;

public class Indexer extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(9, Constants.CANIVORE);
  private final DigitalInput bottomInput = new DigitalInput(0);
  private final DigitalInput topInput = new DigitalInput(1);

  public Indexer() {
    motor.setInverted(false);
    //motor.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double speed = 0;
    if (LimeLightWrapper.getX() >=-1.5 && LimeLightWrapper.getX() <= 1.5 && LimeLightWrapper.hasTarget()){
      double y = LimeLightWrapper.getY();
      //speed = (-19.7654)*(y*y) + (281.237)*(y) + 14111.9;
      speed = (-79.6142)*(y) + 15041.2;
    } else {
      speed = 14800;
    }
    boolean wtf = false;
    SmartDashboard.putBoolean("bottom", bottomInput.get());
    SmartDashboard.putBoolean("top", topInput.get());
    SmartDashboard.putNumber("soooter velocidad from indexer", RobotContainer.s_shooterSubsystem.motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("shooder index speed", speed);
    if (LimeLightWrapper.getX() >= -1.0 && LimeLightWrapper.getX() <= 1.0 && RobotContainer.s_shooterSubsystem.motor.getSelectedSensorVelocity() >= (speed - 1000) && speed > 10000){
      wtf = true;
      moveTheThing(0.5d);
    } else {
    if (topInput.get()) {
      if (!bottomInput.get())
        moveTheThing(0.15d);
      else
        moveTheThing(0.0d);
    }
    
    else{
      wtf = false;
      moveTheThing(0.0d);
    
    }
  }
    SmartDashboard.putBoolean("should be shooting", wtf);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void moveTheThing(double speed) {
    motor.set(speed);
  }
}
