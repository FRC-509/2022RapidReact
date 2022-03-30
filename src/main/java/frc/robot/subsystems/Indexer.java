package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final WPI_TalonFX motor = new WPI_TalonFX(9, Constants.CANIVORE);
  private final DigitalInput bottomInput = new DigitalInput(0);
  private final DigitalInput topInput = new DigitalInput(1);

  public Indexer() {
    motor.setInverted(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("bottom", bottomInput.get());
    SmartDashboard.putBoolean("top", topInput.get());
    if (topInput.get()) {
      if (!bottomInput.get())
        moveTheThing(0.1d);
      else
        moveTheThing(0.0d);
    }
    else
      moveTheThing(0.0d);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  public void moveTheThing(double speed) {
    motor.set(speed);
  }
}
