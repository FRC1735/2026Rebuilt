// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.limelight.LimelightLogger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  // private final ShooterHood shooterHood;
  // private final ShooterIntake shooterIntake;
  // private final CollectorDeployer collectorDeployer;
  // private final RollerIntake collectorExteriorRight;
  // private final RollerIntake collectorExteriorLeft;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Limelight logging
  private final LimelightLogger rearLimelightLogger = new LimelightLogger("limelight-rear");
  private final LimelightLogger fronLimelightLogger = new LimelightLogger("limelight-front");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        shooter =
            new Shooter(
                Constants.SHOOTER_ENABLED
                    ? new ShooterIOSparkFlex(
                        Constants.SHOOTER_LEFT_CAN_ID, Constants.SHOOTER_RIGHT_CAN_ID)
                    : new ShooterIO() {});

        /*
        shooterHood = new ShooterHood(new ShooterHoodIOSparkFlex(Constants.SHOOTER_HOOD_CAN_ID));

        shooterIntake =
            new ShooterIntake(new ShooterIntakeIOSparkFlex(Constants.SHOOTER_INTAKE_CAN_ID));


        collectorDeployer =
            new CollectorDeployer(
                new CollectorDeployerIOSparkFlex(
                    Constants.COLLECTOR_DEPLOYER_CAN_ID,
                    Constants.COLLECTOR_DEPLOYER_DETACHED_ENCODER_CAN_ID));

        collectorExteriorRight =
            new RollerIntake(
                new RollerIntakeIOSparkFlex(
                    Constants.COLLECTOR_EXTERIOR_RIGHT_CAN_ID, "CollectorExteriorRight", true),
                "CollectorExteriorRight");
        collectorExteriorLeft =
            new RollerIntake(
                new RollerIntakeIOSparkFlex(
                    Constants.COLLECTOR_EXTERIOR_LEFT_CAN_ID, "CollectorExteriorLeft", false),
                "CollectorExteriorLeft");
                */
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        shooter = new Shooter(new ShooterIOSim());
        /*
        shooterHood = new ShooterHood(new ShooterHoodIOSim());

        shooterIntake = new ShooterIntake(new ShooterIntakeIOSim());


        collectorDeployer = new CollectorDeployer(new CollectorDeployerIOSim());

        collectorExteriorRight =
            new RollerIntake(new RollerIntakeIOSim(), "CollectorExteriorRight");
        collectorExteriorLeft = new RollerIntake(new RollerIntakeIOSim(), "CollectorExteriorLeft");
        */
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter = new Shooter(new ShooterIO() {});
        /*
        shooterHood = new ShooterHood(new ShooterHoodIO() {});

        shooterIntake = new ShooterIntake(new ShooterIntakeIO() {});


        collectorDeployer = new CollectorDeployer(new CollectorDeployerIO() {});

        collectorExteriorRight =
            new RollerIntake(new RollerIntakeIO() {}, "CollectorExteriorRight");

        collectorExteriorLeft = new RollerIntake(new RollerIntakeIO() {}, "CollectorExteriorLeft");
        break;
        */
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void logLimelights() {
    rearLimelightLogger.log();
    fronLimelightLogger.log();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Set robot pose to 0, 0 which is the origin position on the Field 3D map.
    // Useful for determining robot model config offsets.
    if (Constants.DEBUG) {
      controller
          .y()
          .onTrue(
              Commands.runOnce(() -> drive.setPose(new Pose2d(0, 0, Rotation2d.kZero)), drive)
                  .ignoringDisable(true));
    }

    /*
    // Shoot Shooter - High Speed
    controller
        .rightBumper()
        .whileTrue(Commands.run(shooter::shootHigh, shooter))
        .onFalse(Commands.runOnce(shooter::stop, shooter));

    // Shoot Shooter - Low Speed
    controller
        .leftBumper()
        .whileTrue(Commands.run(shooter::shootLow, shooter))
        .onFalse(Commands.runOnce(shooter::stop, shooter));

    // Shooter Hood - Position A
    controller
        .povUp()
        .onTrue(Commands.runOnce(() -> shooterHood.setPositionRotations(0.8), shooterHood));

    // Shooter Hood - Position B
    controller
        .povDown()
        .onTrue(Commands.runOnce(() -> shooterHood.setPositionRotations(0.4), shooterHood));

    // DEBUG - tell hood to go outside of position range
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterHood.setPositionRotations(0.99);
                },
                shooterHood));
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> shooterHood.setPositionRotations(0.22), shooterHood));
        */
    /*
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  collectorExteriorRight.setVoltage(-9);
                  collectorExteriorLeft.setVoltage(-9);
                },
                collectorExteriorRight,
                collectorExteriorLeft))
        .onFalse(
            Commands.runOnce(
                () -> {
                  collectorExteriorRight.stop();
                  collectorExteriorLeft.stop();
                },
                collectorExteriorRight,
                collectorExteriorLeft));

    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  collectorExteriorRight.setVoltage(9);
                  collectorExteriorLeft.setVoltage(9);
                },
                collectorExteriorRight,
                collectorExteriorLeft))
        .onFalse(
            Commands.runOnce(
                () -> {
                  collectorExteriorRight.stop();
                  collectorExteriorLeft.stop();
                },
                collectorExteriorRight,
                collectorExteriorLeft));
                */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
