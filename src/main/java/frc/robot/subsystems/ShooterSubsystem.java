// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightWrapper;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(10);
  private final WPI_TalonFX motor2 = new WPI_TalonFX(11);

  private MotorControllerGroup m_motorGroup = new MotorControllerGroup(motor, motor2);
  
  public ShooterSubsystem() {
    motor2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void shoot(double speed) {
    m_motorGroup.set(speed);
    LimeLightWrapper.turnON();
  }
  public double getDistanceToTarget() {
    //height between limelight and target
    return 73.5d / Math.tan(LimeLightWrapper.getAngle() + 35.0d);
  }
}
