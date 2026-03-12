package frc.robot.subsystems;

import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.rollerintake.RollerIntakeIO;
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

  public static RollerIntake createReplayShooterIntake() {
    return new RollerIntake(new RollerIntakeIO() {}, "Shooter Intake");
  }

  /// Collector
  public static CollectorDeployer createReplayCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIO() {}, "Collector Deployer");
  }

  public static RollerIntake createReplayCollectorExteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Collector Exterior Roller");
  }

  public static RollerIntake createReplayCollectorInteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Collector Interior Roller");
  }
}
