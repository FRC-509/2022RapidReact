package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> origin/thinkpad
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(9, Constants.CANIVORE);
<<<<<<< HEAD
  private final DigitalInput bottomInput = new DigitalInput(0);
  private final DigitalInput topInput = new DigitalInput(1);
=======
  private final DigitalInput im_a_bottom = new DigitalInput(0);
  private final DigitalInput im_a_top = new DigitalInput(1);
>>>>>>> origin/thinkpad

  public Indexer() {
    motor.setInverted(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
<<<<<<< HEAD
    if (topInput.get()) {
      if (!bottomInput.get())
        moveTheThing(0.1d);
      else
        moveTheThing(0.0d);
    }
    else
      moveTheThing(0.0d);
=======
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
>>>>>>> origin/thinkpad
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void moveTheThing(double speed) {
    motor.set(speed);
  }
}
