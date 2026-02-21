package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;
import frc.robot.subsystems.rollerintake.RollerIntake;

public class RightCollectorCommands {
    private RightCollectorCommands() {}

    public static Command deploy(CollectorDeployer collectorDeployer) {
        return Commands.runOnce(collectorDeployer::deploy, collectorDeployer);
    }

    public static Command retract(CollectorDeployer collectorDeployer) {
        return Commands.runOnce(collectorDeployer::retract, collectorDeployer);
    }

    public static Command intake(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
        return Commands.parallel(
                Commands.run(interiorRoller::in),
                Commands.run(exteriorRoller::in)
        );
    }

    public static Command outtake(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
        return Commands.parallel(
                Commands.run(interiorRoller::out),
                Commands.run(exteriorRoller::out)
        );
    }

    public static Command stopCollection(RollerIntake interiorRoller, RollerIntake exteriorRoller) {
        return Commands.parallel(
                Commands.runOnce(interiorRoller::stop),
                Commands.runOnce(exteriorRoller::stop)
        );
    }

    public static Command manualDeploy(CollectorDeployer collectorDeployer) {
        return Commands.runOnce(collectorDeployer::incrementTargetPosition);
    }

    public static Command manualRetract(CollectorDeployer collectorDeployer) {
        return Commands.runOnce(collectorDeployer::decrementTargetPosition);
    }
}