package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.rollerintake.RollerIntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;

public class SimulatedSubsystemFactory {

  /// Drive
  public static Drive createSimulatedDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(TunerConstants.FrontLeft),
        new ModuleIOSim(TunerConstants.FrontRight),
        new ModuleIOSim(TunerConstants.BackLeft),
        new ModuleIOSim(TunerConstants.BackRight));
  }

  /// Shooter
  public static Shooter createSimulatedShooter() {
    return new Shooter(new ShooterIOSim());
  }

  public static ShooterHood createSimulatedShooterHood() {
    return new ShooterHood(new ShooterHoodIOSim());
  }

  public static RollerIntake createSimulatedShooterIntake() {
    return new RollerIntake(new RollerIntakeIOSim(), "Shooter Intake");
  }

  /// Left Collector
  public static CollectorDeployer createSimulatedLeftCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIOSim());
  }

  public static RollerIntake createSimulatedLeftCollectorExteriorRoller() {
    return new RollerIntake(new RollerIntakeIOSim(), "Left Collector Exterior Intake");
  }

  public static RollerIntake createSimulatedLeftCollectorInteriorRoller() {
    return new RollerIntake(new RollerIntakeIOSim(), "Left Collector Interior Intake");
  }

  /// Right Collector
  public static CollectorDeployer createSimulatedRightCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIOSim());
  }

  public static RollerIntake createSimulatedRightCollectorExteriorRoller() {
    return new RollerIntake(new RollerIntakeIOSim(), "Right Collector Exterior Intake");
  }

  public static RollerIntake createSimulatedRightCollectorInteriorRoller() {
    return new RollerIntake(new RollerIntakeIOSim(), "Right Collector Interior Intake");
  }
}
