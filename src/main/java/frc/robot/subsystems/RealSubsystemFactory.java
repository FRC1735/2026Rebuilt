package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIOSparkFlex;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.rollerintake.RollerIntakeIO;
import frc.robot.subsystems.rollerintake.RollerIntakeIOSparkFlex;
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

  public static RollerIntake createRealShooterIntake() {
    return new RollerIntake(
        Constants.SHOOTER_INTAKE_ENABLED
            ? new RollerIntakeIOSparkFlex(Constants.SHOOTER_INTAKE_CAN_ID, "Shooter Intake", true)
            : new RollerIntakeIO() {},
        "Shooter Intake");
  }

  /// Collector
  public static CollectorDeployer createRealCollectorDeployer() {
    return new CollectorDeployer(
        Constants.COLLECTOR_DEPLOYER_ENABLED
            ? new CollectorDeployerIOSparkFlex(
                Constants.COLLECTOR_DEPLOYER_CAN_ID,
                Constants.COLLECTOR_DEPLOYER_ENCODER_CAN_ID,
                "Collector Deployer",
                2.8, // deploy
                0.2,
                0.1,
                3.2, // retract
                0,
                0,
                0.5, // both
                0,
                false)
            : new CollectorDeployerIO() {},
        "Collector Deployer");
  }

  public static RollerIntake createRealCollectorExteriorRoller() {
    return new RollerIntake(
        Constants.COLLECTOR_ROLLER_EXTERIOR_ENABLED
            ? new RollerIntakeIOSparkFlex(
                Constants.COLLECTOR_ROLLER_EXTERIOR_CAN_ID, "Collector Exterior Roller", false)
            : new RollerIntakeIO() {},
        "Collector Exterior Roller");
  }
}
