package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLightWrapper;

public class ShooterSubsystem extends SubsystemBase {
  public final WPI_TalonFX motor = new WPI_TalonFX(10, Constants.CANIVORE);
  public final WPI_TalonFX motor2 = new WPI_TalonFX(11, Constants.CANIVORE);
  
  public MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
  public ShooterSubsystem() {
    //motor2.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter fuckvelocty", motor.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

<<<<<<< HEAD
  public void shoot(boolean shooting) {
    // if (speed == 0.0d){
    //   m_motorGroup.set(speed);
    // } else {
    //   m_motorGroup.set(speed / 2.0d + 0.5);
    // }
    
    SmartDashboard.putBoolean("speeeeed man", shooting);
    if (shooting != true){
      motor.set(ControlMode.PercentOutput, 0);
      motor2.set(ControlMode.PercentOutput, 0);
    } else {
      double speed = 0;
      if (LimeLightWrapper.hasTarget()){
        double y = LimeLightWrapper.getY();
        //speed = (-19.7654)*(y*y) + (281.237)*(y) + 14111.9;
        speed = (-74.6142)*(y) + 12041.2;
      } else {
        speed = 11800;
      }
      SmartDashboard.putNumber("adfhajdsfksajdhflkjsahfdkjasdf", speed);
      speed *= 5.5;
      motor.set(ControlMode.Velocity, speed);
      motor2.set(ControlMode.Velocity, -speed);
    }
=======
  public void shoot(double speed) {
    if (speed == 0.0d){
      m_motorGroup.set(speed);
    } else {
      m_motorGroup.set(speed / 2.0d + 0.5);
    }

    // if (speed == 0.0d){
    //   motor.set(ControlMode.Velocity, speed);
    //   motor2.set(ControlMode.Velocity, speed);
    // } else {
    //   motor.set(ControlMode.Velocity, speed / 2.0d + 0.5);
    //   motor2.set(ControlMode.Velocity, speed / 2.0d + 0.5);
    // }
>>>>>>> 6dc5adb53cadb7f7a56e8c3675451a0c2f307ec7
  }
}
