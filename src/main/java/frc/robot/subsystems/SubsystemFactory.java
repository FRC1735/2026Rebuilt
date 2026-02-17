package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.rollerintake.RollerIntakeIO;
import frc.robot.subsystems.rollerintake.RollerIntakeIOSim;
import frc.robot.subsystems.rollerintake.RollerIntakeIOSparkFlex;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIO;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSparkFlex;

public class SubsystemFactory {
  // Real Subsystems
  public static Drive createRealDrive() {
    return new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight));
  }

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
            : new RollerIntakeIO() {}, "Shooter Intake");
  }

  // Simulated Subsystems
  public static Drive createSimulatedDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(TunerConstants.FrontLeft),
        new ModuleIOSim(TunerConstants.FrontRight),
        new ModuleIOSim(TunerConstants.BackLeft),
        new ModuleIOSim(TunerConstants.BackRight));
  }

  public static Shooter createSimulatedShooter() {
    return new Shooter(new ShooterIOSim());
  }

  public static ShooterHood createSimulatedShooterHood() {
    return new ShooterHood(new ShooterHoodIOSim());
  }

  public static RollerIntake createSimulatedShooterIntake() {
    return new RollerIntake(new RollerIntakeIOSim(), "Shooter Intake");
  }

  // Replay Subsystems
  public static Drive createReplayDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {});
  }

  public static Shooter createReplayShooter() {
    return new Shooter(new ShooterIO() {});
  }

  public static ShooterHood createReplayShooterHood() {
    return new ShooterHood(new ShooterHoodIO() {});
  }

  public static RollerIntake createReplayShooterIntake() {
    return new RollerIntake(new RollerIntakeIO() {}, "Shooter Intake");
  }
}
