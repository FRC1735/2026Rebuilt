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
            ? new ShooterIOSparkFlex(Constants.SHOOTER_LEFT_CAN_ID, Constants.SHOOTER_RIGHT_CAN_ID)
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
            ? new RollerIntakeIOSparkFlex(Constants.SHOOTER_INTAKE_CAN_ID, "Shooter Intake", false)
            : new RollerIntakeIO() {},
        "Shooter Intake");
  }

  /// Left Collector
  public static CollectorDeployer createRealLeftCollectorDeployer() {
    return new CollectorDeployer(
        Constants.LEFT_COLLECTOR_DEPLOYER_ENABLED
            ? new CollectorDeployerIOSparkFlex(
                Constants.LEFT_COLLECTOR_DEPLOYER_CAN_ID,
                Constants.LEFT_COLLECTOR_DEPLOYER_ENCODER_CAN_ID,
                "Left Collector Deployer",
                2.8, // deploy
                0.2,
                0.1,
                3.2, // retract
                0,
                0,
                0.5, // both
                0,
                false,
                0.2f)
            : new CollectorDeployerIO() {},
        "Left Collector Deployer");
  }

  public static RollerIntake createRealLeftCollectorExteriorRoller() {
    return new RollerIntake(
        Constants.LEFT_COLLECTOR_ROLLER_EXTERIOR_ENABLED
            ? new RollerIntakeIOSparkFlex(
                Constants.LEFT_COLLECTOR_ROLLER_EXTERIOR_CAN_ID,
                "Left Collector Exterior Roller",
                false)
            : new RollerIntakeIO() {},
        "Left Collector Exterior Roller");
  }

  public static RollerIntake createRealLeftCollectorInteriorRoller() {
    return new RollerIntake(
        Constants.LEFT_COLLECTOR_ROLLER_INTERIOR_ENABLED
            ? new RollerIntakeIOSparkFlex(
                Constants.LEFT_COLLECTOR_ROLLER_INTERIOR_CAN_ID,
                "Left Collector Interior Roller",
                false)
            : new RollerIntakeIO() {},
        "Left Collector Interior Roller");
  }

  /// Right Collector

}
