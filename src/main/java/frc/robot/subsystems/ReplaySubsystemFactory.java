package frc.robot.subsystems;

import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.dualrollerintake.DualRollerIntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIO;

public class ReplaySubsystemFactory {
  /// Drive
  public static Drive createReplayDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {});
  }

  /// Shooter
  public static Shooter createReplayShooter() {
    return new Shooter(new ShooterIO() {});
  }

  public static ShooterHood createReplayShooterHood() {
    return new ShooterHood(new ShooterHoodIO() {});
  }

  public static DualRollerIntake createReplayShooterIntake() {
    return new DualRollerIntake(new DualRollerIntakeIO() {}, "Shooter Intake");
  }

  /// Collector
  public static CollectorDeployer createReplayCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIO() {}, "Collector Deployer");
  }

  public static DualRollerIntake createReplayCollectorExteriorRoller() {
    return new DualRollerIntake(new DualRollerIntakeIO() {}, "Collector Exterior Roller");
  }
}
