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

  public static Command close(CollectorDeployer collectorDeployer) {
    return Commands.runOnce(collectorDeployer::close, collectorDeployer);
  }

  public static Command intake(RollerIntake exteriorRoller) {
    return new StartEndCommand(exteriorRoller::in, exteriorRoller::stop, exteriorRoller);
  }

  public static Command outtake(RollerIntake exteriorRoller) {
    return new StartEndCommand(exteriorRoller::out, exteriorRoller::stop, exteriorRoller);
  }

  public static Command stopCollection(RollerIntake exteriorRoller) {
    return Commands.runOnce(exteriorRoller::stop);
  }

  public static Command manualDeploy(CollectorDeployer collectorDeployer) {
    return new StartEndCommand(
        collectorDeployer::manualDeploy, collectorDeployer::stop, collectorDeployer);
  }

  public static Command manualClose(CollectorDeployer collectorDeployer) {
    return new StartEndCommand(
        collectorDeployer::manualClose, collectorDeployer::stop, collectorDeployer);
  }
}
