package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterhood.ShooterHood;

public class AutoCommands {
  public static Command shootPreloaded(
      Drive drive,
      Shooter shooter,
      DualRollerIntake shooterIntake,
      CollectorDeployer collectorDeployer,
      DualRollerIntake collectorExteriorRoller) {
    return Commands.sequence(
            Commands.parallel(
                ShooterCommands.shootAtVelocity(3000, shooter, shooterIntake),
                new WaitCommand(1)
                    .andThen(CollectorCommands.manualDeploy(collectorDeployer).withTimeout(1)),
                new WaitCommand(1).andThen(CollectorCommands.intake(collectorExteriorRoller))))
        .withTimeout(3.5);
  }

  public static Command shootAtDepot(
      Shooter shooter, DualRollerIntake shooterIntake, ShooterHood shooterHood) {
    return Commands.sequence(
            ShooterCommands.hoodDepot(shooterHood),
            new WaitCommand(1),
            ShooterCommands.shootAtVelocity(3500, shooter, shooterIntake))
        .withTimeout(20);
  }
}
