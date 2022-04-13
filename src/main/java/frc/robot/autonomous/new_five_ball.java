// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimBot;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveFeedback;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class new_five_ball extends SequentialCommandGroup {
  /** Creates a new auto1. */
  public new_five_ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //to make sure the auto ends
      new ParallelRaceGroup(
        //auto stringed together
        new SequentialCommandGroup(
          //intake down
          new ParallelRaceGroup(
            new IntakeDown(),
            new WaitCommand(0.01)
          ),
          new ParallelRaceGroup(
            //forward
            new DriveFeedback(() -> -1.0d, () -> 0.0d, () -> 0.0d),
            //intake
            new IntakeSpin()
          ),
          //two balls aquired
          //intake up
          new ParallelRaceGroup(
            new IntakeUp(),
            new WaitCommand(0.01)
          ),
          //backing up from wall and turn to shoot
            new DriveFeedback(() -> 1.0d, () -> 0.0d, () -> -180.0d),
          //aim
          new ParallelRaceGroup(
            new AimBot(),
            new WaitCommand(0.5)
          ),
          //shoot
          new ParallelRaceGroup(
            //FIXME replace with auto shooting distance code
            new ShooterCommand(()  -> true),
            //FIXME have this end with an auto shooting command
            new IndexerCommand(),
            new WaitCommand(1.5)
          ),
          //backing up MAY BE UNNECESSARY
          new DriveFeedback(() -> -1.0d, () -> 0.0d, () -> 0.0d),
          //two balls shot, on to the third
          //intake down
          new ParallelRaceGroup(
            new IntakeDown(),
            new WaitCommand(0.01)
          ),
          //drive and intake
          new ParallelRaceGroup(
            new DriveFeedback(() -> 1.5d, () -> 5.25d, () -> 0.0d),
            new IntakeSpin()
          ),
          //intake up
          new ParallelRaceGroup(
            new IntakeUp(),
            new WaitCommand(0.01)
          ),
          //turn to hub
          new DriveFeedback(() -> 0.0d, () -> 0.0d, () -> -45.0d),
          //aim
          new ParallelRaceGroup(
            new AimBot(),
            new WaitCommand(0.5)
          ),
          //shoot
          //shot 3rd
          new ParallelRaceGroup(
            //FIXME replace with auto shooting distance code
            new ShooterCommand(()  -> true),
            //FIXME have this end with an auto shooting command
            new IndexerCommand(),
            new WaitCommand(1.5)
          ),
          //drive to next ball
          new DriveFeedback(() -> -0.5d, () -> 8.5d, () -> -90.0d),
          //wait for zach's slow ass to give ball
          new ParallelRaceGroup(
            new IntakeSpin(),
            new WaitCommand(2)
          ),
          //intake down
          new ParallelRaceGroup(
            new IntakeDown(),
            new WaitCommand(0.1)
          ),
          //drive moment
          new DriveFeedback(() -> 0.5d, () -> 8.5d, () -> 90.0d),
          //aim
          new ParallelRaceGroup(
            new AimBot(),
            new WaitCommand(0.5)
          ),
          //shoot
          //shot 4 and 5
          new ParallelRaceGroup(
            //FIXME replace with auto shooting distance code
            new ShooterCommand(()  -> true),
            new IndexerCommand(),
            //FIXME have this end with an auto shooting command
            new WaitCommand(1.5)
          )
        ),
        //cuts off the robot at end of auto (in case)
        new WaitCommand(14.9)
      )
    );
  }
}
