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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RightCollectorCommands;
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
        .onTrue(RightCollectorCommands.deploy(rightCollectorDeployer)); // TODO - verify
    operatorController
        .rightCollector()
        .retract()
        .onTrue(RightCollectorCommands.retract(rightCollectorDeployer)); // TODO - verify
    operatorController
        .rightCollector()
        .intake()
        .whileTrue(
            RightCollectorCommands.intake(
                rightCollectorInteriorRoller, rightCollectorExteriorRoller))
        .onFalse(
            RightCollectorCommands.stopCollection(
                rightCollectorInteriorRoller, rightCollectorExteriorRoller)); // TODO - verify
    operatorController
        .rightCollector()
        .outtake()
        .whileTrue(
            RightCollectorCommands.outtake(
                rightCollectorInteriorRoller, rightCollectorExteriorRoller))
        .onFalse(
            RightCollectorCommands.stopCollection(
                rightCollectorInteriorRoller, rightCollectorExteriorRoller)); // TODO - verify
    operatorController
        .rightCollector()
        .manualDeploy()
        .onTrue(RightCollectorCommands.manualDeploy(rightCollectorDeployer)); // TODO - verify
    operatorController
        .rightCollector()
        .manualRetract()
        .onTrue(RightCollectorCommands.manualRetract(rightCollectorDeployer)); // TODO - verify

    // Left Collector
    operatorController.leftCollector().deploy(); // TODO
    operatorController.leftCollector().retract(); // TODO
    operatorController.leftCollector().intake(); // TODO
    operatorController.leftCollector().outtake(); // TODO
    operatorController.leftCollector().manualDeploy(); // TODO
    operatorController.leftCollector().manualRetract(); // TODO
  }

  private void configureDeveloperBindings() {
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

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
