package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.dualrollerintake.DualRollerIntakeIOSim;
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

  public static DualRollerIntake createSimulatedShooterIntake() {
    return new DualRollerIntake(new DualRollerIntakeIOSim(), "Shooter Intake");
  }

  /// Collector
  public static CollectorDeployer createSimulatedCollectorDeployer() {
    return new CollectorDeployer(new CollectorDeployerIOSim(), "Collector Deployer");
  }

  public static DualRollerIntake createSimulatedCollectorExteriorRoller() {
    return new DualRollerIntake(new DualRollerIntakeIOSim(), "Collector Exterior Intake");
  }
}
