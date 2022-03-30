package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(9, Constants.CANIVORE);
  private final DigitalInput im_a_bottom = new DigitalInput(0);
  private final DigitalInput im_a_top = new DigitalInput(1);

  public Indexer() {
    motor.setInverted(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("bottom", im_a_bottom.get());
    SmartDashboard.putBoolean("top", im_a_top.get());
    if (im_a_top.get()){
      if(!im_a_bottom.get()){
        moveTheThing(0.1);
      } else {
        moveTheThing(0);
      }
    } else {
      moveTheThing(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void moveTheThing(double speed) {
    motor.set(speed);
  }
}
