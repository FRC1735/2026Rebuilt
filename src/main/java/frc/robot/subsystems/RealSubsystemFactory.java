package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIOSparkFlex;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.dualrollerintake.DualRollerIntakeIO;
import frc.robot.subsystems.dualrollerintake.DualRollerIntakeIOSparkFlex;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIO;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSparkFlex;

public class RealSubsystemFactory {
  /// Drive
  public static Drive createRealDrive() {
    return new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight));
  }

  /// Shooter
  public static Shooter createRealShooter() {
    return new Shooter(
        Constants.SHOOTER_ENABLED
            ? new ShooterIOSparkFlex(
                Constants.SHOOTER_LEADER_CAN_ID, Constants.SHOOTER_FOLLOWER_CAN_ID)
            : new ShooterIO() {});
  }

  public static ShooterHood createRealShooterHood() {
    return new ShooterHood(
        Constants.SHOOTER_HOOD_ENABLED
            ? new ShooterHoodIOSparkFlex(Constants.SHOOTER_HOOD_CAN_ID)
            : new ShooterHoodIO() {});
  }

  public static DualRollerIntake createRealShooterIntake() {
    return new DualRollerIntake(
        Constants.SHOOTER_INTAKE_ENABLED
            ? new DualRollerIntakeIOSparkFlex(
                Constants.SHOOTER_LEADER_INTAKE_CAN_ID,
                Constants.SHOOTER_FOLLOWER_INTAKE_CAN_ID,
                "Shooter Intake",
                true) // verify in new build
            : new DualRollerIntakeIO() {},
        "Shooter Intake");
  }

  /// Collector
  public static CollectorDeployer createRealCollectorDeployer() {
    return new CollectorDeployer(
        Constants.COLLECTOR_DEPLOYER_ENABLED
            ? new CollectorDeployerIOSparkFlex(
                Constants.COLLECTOR_DEPLOYER_CAN_ID, Constants.COLLECTOR_DEPLOYER_ENCODER_CAN_ID)
            : new CollectorDeployerIO() {},
        "Collector Deployer");
  }

  public static DualRollerIntake createRealCollectorExteriorRoller() {
    return new DualRollerIntake(
        Constants.COLLECTOR_ROLLER_EXTERIOR_ENABLED
            ? new DualRollerIntakeIOSparkFlex(
                Constants.COLLECTOR_LEADER_ROLLER_CAN_ID,
                Constants.COLLECTOR_FOLLOWER_ROLLER_CAN_ID,
                "Collector Roller",
                true) // Verify in new build
            : new DualRollerIntakeIO() {},
        "Collector Exterior Roller");
  }
}
