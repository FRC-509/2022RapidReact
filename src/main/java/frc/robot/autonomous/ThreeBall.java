package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimBot;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IndexerSpin;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.ShooterCommand;

public class ThreeBall extends SequentialCommandGroup {
  public ThreeBall() {
    addCommands(
      //to make sure the auto ends
      new ParallelRaceGroup(
        //auto stringed together
        new SequentialCommandGroup(
          //intake down
          new ParallelRaceGroup(
            new IntakeDown(),
            new WaitCommand(0.1)
          ),
          new ParallelRaceGroup(
            //forward
            new DefaultDriveCommand(() -> -1.0d, () -> 0.2d, ()-> -0.1d),
            //intake
            new IntakeSpin(),
            //waiting un segundo
            new WaitCommand(1.0)
          ),
          //two balls aquired
          //intake up
          new ParallelRaceGroup(
            new IntakeUp(),
            new WaitCommand(0.1)
          ),
          //backing up from wall
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> 1.0d, () -> 0.0d, () -> 0.0d),
            new WaitCommand(0.5)
          ),
          //time to turn and shoot
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> 2.7d),
            new WaitCommand(1.2)
          ),
          //aim
          new ParallelRaceGroup(
            new AimBot(),
            new WaitCommand(1.0)
          ),
          //shoot spin up
          new SequentialCommandGroup(
            new ShooterCommand(()  -> 0.32d)
          ).withTimeout(0.5),
          //shoot
          new ParallelCommandGroup(
            new ShooterCommand(()  -> 0.32d),
            new IndexerSpin()
          ).withTimeout(2),
          //backing up
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> 2.0d, () -> 0.0d, () -> 0.0d),
            new WaitCommand(0.25)
          ),
          //two balls shot, on to the third
          //turn to next ball
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> -4.0d),
            new WaitCommand(0.465)
          ),
          //intake down
          new ParallelRaceGroup(
            new IntakeDown(),
            new WaitCommand(0.1)
          ),
          //drive and intake
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> -2.0d, () -> 0.35d, () -> 0.0d),
            new IntakeSpin(),
            new WaitCommand(1.2)
          ),
          //intake up
          new ParallelRaceGroup(
            new IntakeUp(),
            new WaitCommand(0.1)
          ),
          //turn to hub
          new ParallelRaceGroup(
            new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> 4.0d),
            new WaitCommand(0.5)
          ),
          //aim
          new ParallelRaceGroup(
            new AimBot(),
            new WaitCommand(1.5)
          ),
          //shoot
          //shot 3rd
          new ParallelCommandGroup(
            new ShooterCommand(()  -> 0.32d),
            new IndexerSpin()
          ).withTimeout(2)
          //this shit is experimental
          // //turn to next ball ( 180)
          // new ParallelRaceGroup(
          //   new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> -3.8d),
          //   new WaitCommand(.5)
          // ),
          // //intake down
          // new ParallelRaceGroup(
          //   new IntakeDown(),
          //   new WaitCommand(0.1)
          // ),
          // //drive to next ball
          // new ParallelRaceGroup(
          //   new DefaultDriveCommand(() -> -4.0d, () -> 0.0d, () -> -1.0d),
          //   new IntakeSpin(),
          //   new WaitCommand(1.6)
          // ),
          // //wait for zach's slow ass to give ball
          // new ParallelRaceGroup(
          //   new IntakeSpin(),
          //   new WaitCommand(2)
          // ),
          // //turn round
          // new ParallelRaceGroup(
          //   new DefaultDriveCommand(() -> 0.0d, () -> 0.0d, () -> -4.0d),
          //   new WaitCommand(.8)
          // ),
          // //drive moment
          // new ParallelRaceGroup(
          //   new DefaultDriveCommand(() -> -4.0d, () -> 0.0d, () -> 0.0d),
          //   new WaitCommand(2)
          // ),
          // //aim
          // new ParallelRaceGroup(
          //   new AimBot(),
          //   new WaitCommand(1)
          // ),
          // //shoot
          // //shot 4 and 5
          // new ParallelRaceGroup(
          //   new ShooterCommand(()  -> 0.75d),
          //   new IndexerCommand(),
          //   new WaitCommand(2)
          // )
        ),
        //cuts off the robot at end of auto (in case)
        new WaitCommand(14.5)
      )
    );
  }
}
