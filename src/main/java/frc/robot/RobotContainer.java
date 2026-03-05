// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.RealSubsystemFactory.*;
import static frc.robot.subsystems.ReplaySubsystemFactory.*;
import static frc.robot.subsystems.SimulatedSubsystemFactory.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CollectorCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.limelight.LimelightLogger;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollerintake.RollerIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.util.KeyboardController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  /// Shooter
  private final Shooter shooter;
  private final ShooterHood shooterHood;
  private final RollerIntake shooterIntake;
  /// Left Collector
  private final CollectorDeployer leftCollectorDeployer;
  private final RollerIntake leftCollectorExteriorRoller;
  private final RollerIntake leftCollectorInteriorRoller;
  /// Right Collector
  private final CollectorDeployer rightCollectorDeployer;
  private final RollerIntake rightCollectorExteriorRoller;
  private final RollerIntake rightCollectorInteriorRoller;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final KeyboardController operatorController = new KeyboardController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Limelight logging
  private final LimelightLogger rearLimelightLogger = new LimelightLogger("limelight-rear");
  private final LimelightLogger fronLimelightLogger = new LimelightLogger("limelight-front");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive = createRealDrive();
        shooter = createRealShooter();
        shooterHood = createRealShooterHood();
        shooterIntake = createRealShooterIntake();
        leftCollectorDeployer = createRealLeftCollectorDeployer();
        leftCollectorExteriorRoller = createRealLeftCollectorExteriorRoller();
        leftCollectorInteriorRoller = createRealLeftCollectorInteriorRoller();
        rightCollectorDeployer = createRealRightCollectorDeployer();
        rightCollectorExteriorRoller = createRealRightCollectorExteriorRoller();
        rightCollectorInteriorRoller = createRealRightCollectorInteriorRoller();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = createSimulatedDrive();
        shooter = createSimulatedShooter();
        shooterHood = createSimulatedShooterHood();
        shooterIntake = createSimulatedShooterIntake();
        leftCollectorDeployer = createSimulatedLeftCollectorDeployer();
        leftCollectorExteriorRoller = createSimulatedLeftCollectorExteriorRoller();
        leftCollectorInteriorRoller = createSimulatedLeftCollectorInteriorRoller();
        rightCollectorDeployer = createSimulatedRightCollectorDeployer();
        rightCollectorExteriorRoller = createSimulatedRightCollectorExteriorRoller();
        rightCollectorInteriorRoller = createSimulatedRightCollectorInteriorRoller();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = createReplayDrive();
        shooter = createReplayShooter();
        shooterHood = createReplayShooterHood();
        shooterIntake = createReplayShooterIntake();
        leftCollectorDeployer = createRealLeftCollectorDeployer();
        leftCollectorExteriorRoller = createRealLeftCollectorExteriorRoller();
        leftCollectorInteriorRoller = createRealLeftCollectorInteriorRoller();
        rightCollectorDeployer = createRealRightCollectorDeployer();
        rightCollectorExteriorRoller = createRealRightCollectorExteriorRoller();
        rightCollectorInteriorRoller = createRealRightCollectorInteriorRoller();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    /*
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
        */

    autoChooser.addOption(
        "Do Nothing",
        Commands.runOnce(
            () -> {
              if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
              } else {
                drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
              }
            }));

    configureDriverBindings();
    configureOperatorBindings();
    // configureDeveloperBindings();
  }

  public void logLimelights() {
    rearLimelightLogger.log();
    fronLimelightLogger.log();
  }

  private void configureDriverBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Set robot pose to 0, 0 which is the origin position on the Field 3D map.
    driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
                  } else {
                    drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
                  }
                }));
    // Useful for determining robot model config offsets.
    if (Constants.DEBUG) {
      driverController
          .y()
          .onTrue(
              Commands.runOnce(() -> drive.setPose(new Pose2d(0, 0, Rotation2d.kZero)), drive)
                  .ignoringDisable(true));
    }
  }

  private void configureOperatorBindings() {
    // Shooter
    operatorController
        .shooter()
        .hoodToUpPosition()
        .onTrue(ShooterCommands.hoodUp(shooterHood)); // TODO - verify
    operatorController
        .shooter()
        .manualMoveHoodUp()
        .onTrue(ShooterCommands.hoodUpManual(shooterHood)); // TODO - verify
    operatorController
        .shooter()
        .shooterHighSpeed()
        .whileTrue(ShooterCommands.shooterHighSpeed(shooter, shooterIntake))
        .onFalse(ShooterCommands.shooterStop(shooter)); // TODO - verify

    operatorController
        .shooter()
        .hoodToDownPosition()
        .onTrue(ShooterCommands.hoodDown(shooterHood)); // TODO - verify
    operatorController
        .shooter()
        .manualMoveHoodDown()
        .onTrue(ShooterCommands.hoodDownManual(shooterHood)); // TODO - verify
    operatorController
        .shooter()
        .shooterLowSpeed()
        .whileTrue(ShooterCommands.shooterLowSpeed(shooter, shooterIntake))
        .onFalse(ShooterCommands.shooterStop(shooter)); // TODO - verify

    // Right Collector
    operatorController
        .rightCollector()
        .deploy()
        .onTrue(CollectorCommands.deploy(rightCollectorDeployer)); // TODO - verify
    operatorController
        .rightCollector()
        .retract()
        .onTrue(CollectorCommands.retract(rightCollectorDeployer)); // TODO - verif

    operatorController // verified
        .rightCollector()
        .intake()
        .whileTrue(
            CollectorCommands.intake(rightCollectorInteriorRoller, rightCollectorExteriorRoller));

    operatorController // verified
        .rightCollector()
        .outtake()
        .whileTrue(
            CollectorCommands.outtake(rightCollectorInteriorRoller, rightCollectorExteriorRoller));

    operatorController
        .rightCollector()
        .manualDeploy()
        .onTrue(CollectorCommands.manualDeploy(rightCollectorDeployer)); // TODO - verify
    operatorController
        .rightCollector()
        .manualRetract()
        .onTrue(CollectorCommands.manualRetract(rightCollectorDeployer)); // TODO - verify

    // Left Collector
    operatorController
        .leftCollector()
        .deploy()
        .onTrue(CollectorCommands.deploy(leftCollectorDeployer)); // TODO - verify

    operatorController
        .leftCollector()
        .retract()
        .onTrue(CollectorCommands.retract(leftCollectorDeployer)); // TODO - verify

    operatorController // verified
        .leftCollector()
        .intake()
        .whileTrue(
            CollectorCommands.intake(leftCollectorInteriorRoller, leftCollectorExteriorRoller));

    operatorController // verified
        .leftCollector()
        .outtake()
        .whileTrue(
            CollectorCommands.outtake(leftCollectorInteriorRoller, leftCollectorExteriorRoller));

    operatorController
        .leftCollector()
        .manualDeploy()
        .onTrue(CollectorCommands.manualDeploy(leftCollectorDeployer)); // TODO - verify

    operatorController
        .leftCollector()
        .manualRetract()
        .onTrue(CollectorCommands.manualRetract(leftCollectorDeployer)); // TODO - verify
  }

  private void configureDeveloperBindings() {
    // driverController.a().whileTrue(ShooterCommands.shooterIntake(shooterIntake));
    // driverController.b().whileTrue(ShooterCommands.shooterOuttake(shooterIntake));

    /*
    driverController
        .a()
        .whileTrue(
            new StartEndCommand(
                () -> leftCollectorDeployer.setVoltage(6),
                leftCollectorDeployer::stop,
                leftCollectorDeployer));

    driverController
        .b()
        .whileTrue(
            new StartEndCommand(
                () -> leftCollectorDeployer.setVoltage(-6),
                leftCollectorDeployer::stop,
                leftCollectorDeployer));
                */
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
