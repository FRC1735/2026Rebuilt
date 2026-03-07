package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.rollerintake.RollerIntake;

public class CollectorCommands {
  private CollectorCommands() {}

  public static Command deploy(CollectorDeployer collectorDeployer) {
    return Commands.runOnce(collectorDeployer::deploy, collectorDeployer);
  }

  public static Command retract(CollectorDeployer collectorDeployer) {
    return Commands.runOnce(collectorDeployer::retract, collectorDeployer);
  }

  public static Command intake(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
    return Commands.parallel(
        new StartEndCommand(interiorRoller::in, interiorRoller::stop, interiorRoller),
        new StartEndCommand(exteriorRoller::in, exteriorRoller::stop, exteriorRoller));
  }

  public static Command outtake(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
    return Commands.parallel(
        new StartEndCommand(interiorRoller::out, interiorRoller::stop, interiorRoller),
        new StartEndCommand(exteriorRoller::out, exteriorRoller::stop, exteriorRoller));
  }

  public static Command stopCollection(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
    return Commands.parallel(
        Commands.runOnce(interiorRoller::stop), Commands.runOnce(exteriorRoller::stop));
  }

  public static Command manualDeploy(CollectorDeployer collectorDeployer) {
    // return Commands.runOnce(collectorDeployer::manualDeploy);
    return new StartEndCommand(
        collectorDeployer::manualDeploy, collectorDeployer::stop, collectorDeployer);
  }

  public static Command manualRetract(CollectorDeployer collectorDeployer) {
    // return Commands.runOnce(collectorDeployer::manualRetract);
    return new StartEndCommand(
        collectorDeployer::manualRetract, collectorDeployer::stop, collectorDeployer);
  }
}
