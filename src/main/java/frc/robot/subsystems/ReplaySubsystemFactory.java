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

  /// Left Collector
  public static CollectorDeployer createReplayLeftCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIO() {}, "Left Collector Deployer");
  }

  public static RollerIntake createReplayLeftCollectorExteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Left Collector Exterior Roller");
  }

  public static RollerIntake createReplayLeftCollectorInteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Left Collector Interior Roller");
  }

  /// Right Collector
  public static CollectorDeployer createReplayRightCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIO() {}, "Right Collector Deployer");
  }

  public static RollerIntake createReplayRightCollectorExteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Right Collector Exterior Roller");
  }

  public static RollerIntake createReplayRightCollectorInteriorRoller() {
    return new RollerIntake(new RollerIntakeIO() {}, "Right Collector Interior Roller");
  }
}
