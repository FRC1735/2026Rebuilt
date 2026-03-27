package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands {
  public static Command shootPreloaded(
      Drive drive,
      Shooter shooter,
      RollerIntake shooterIntake,
      CollectorDeployer collectorDeployer,
      RollerIntake collectorExteriorRoller) {
    return Commands.sequence(
            DriveCommands.resetPoseForAlliance(drive),
            Commands.parallel(
                ShooterCommands.shootAtVelocity(3000, shooter, shooterIntake),
                new WaitCommand(1)
                    .andThen(CollectorCommands.manualDeploy(collectorDeployer).withTimeout(1)),
                new WaitCommand(1).andThen(CollectorCommands.intake(collectorExteriorRoller))))
        .withTimeout(20);
  }
}
