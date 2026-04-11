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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CloseCollector;
import frc.robot.commands.CollectorCommands;
import frc.robot.commands.DeployCollector;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SidsSmartCommand;
import frc.robot.limelight.LimelightLogger;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
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
  private final DualRollerIntake shooterIntake;
  /// Collector
  private final CollectorDeployer collectorDeployer;
  private final DualRollerIntake collectorExteriorRoller;

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
        shooter = createRealShooter();
        shooterHood = createRealShooterHood();
        shooterIntake = createRealShooterIntake();
        collectorDeployer = createRealCollectorDeployer();
        collectorExteriorRoller = createRealCollectorExteriorRoller();
        drive = createRealDrive();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = createSimulatedDrive();
        shooter = createSimulatedShooter();
        shooterHood = createSimulatedShooterHood();
        shooterIntake = createSimulatedShooterIntake();
        collectorDeployer = createSimulatedCollectorDeployer();
        collectorExteriorRoller = createSimulatedCollectorExteriorRoller();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = createReplayDrive();
        shooter = createReplayShooter();
        shooterHood = createReplayShooterHood();
        shooterIntake = createReplayShooterIntake();
        collectorDeployer = createRealCollectorDeployer();
        collectorExteriorRoller = createRealCollectorExteriorRoller();
        break;
    }

    /*
     *
     * hood 0.41 and speed 3500
     */
    // Register comands for PathPlanner
    NamedCommands.registerCommand("shooter hood up", ShooterCommands.hoodUp(shooterHood));
    NamedCommands.registerCommand("shooter hood down", ShooterCommands.hoodDown(shooterHood));
    NamedCommands.registerCommand(
        "shoot high", ShooterCommands.shootHighSpeed(shooter, shooterIntake));
    NamedCommands.registerCommand(
        "collect", CollectorCommands.intake(collectorExteriorRoller).withTimeout(20));
    NamedCommands.registerCommand(
        "shoot preloaded",
        AutoCommands.shootPreloaded(
                drive, shooter, shooterIntake, collectorDeployer, collectorExteriorRoller)
            .withTimeout(5));
    NamedCommands.registerCommand(
        "shoot at depot",
        AutoCommands.shootAtDepot(shooter, shooterIntake, shooterHood).withTimeout(5));
    NamedCommands.registerCommand("deploy", new DeployCollector(collectorDeployer));

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

    autoChooser.addOption("Do Nothing", DriveCommands.resetPoseForAlliance(drive));
    autoChooser.addOption(
        "Shoot Preloaded",
        AutoCommands.shootPreloaded(
            drive, shooter, shooterIntake, collectorDeployer, collectorExteriorRoller));
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
    /*
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
             */

    // driverController.rightTrigger().whileTrue(
    //     DriveCommands.joystickDiveWithSidShoot(drive, () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(), shooter, shooterIntake, shooterHood)
    // );

    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithAutoAlign(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> {
              Trigger rightBumper = driverController.rightBumper();
              Trigger rightTrigger = driverController.rightTrigger();

              if (rightBumper.getAsBoolean()) {
                // todo verify
                if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                  if (drive.inTopHalfOfField()) {
                    return -2;
                  } else {
                    return 2;
                  }
                } else {
                  if (drive.inTopHalfOfField()) {
                    return -178;
                  } else {
                    return 178;
                  }
                }
              } else if (rightTrigger.getAsBoolean()) {
                if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                  double hoodAngle =
                      SidsSmartCommand.getHoodAngle(
                          drive.getXPosition(),
                          4.67,
                          drive.getYPosition(),
                          4.07,
                          1.85,
                          0.322,
                          50,
                          9.81);
                  return SidsSmartCommand.getTargetAngle(
                      drive.getXPosition(), 4.67, drive.getYPosition(), 4.07);

                } else {
                  return SidsSmartCommand.getTargetAngle(
                      drive.getXPosition(), 12.03, drive.getYPosition(), 4.07);
                }

              } else {
                return -1;
              }

              /*

              int pov = driverController.getHID().getPOV();

              // POV returns -1 when not pressed
              if (pov == -1) {
                return -1; // disables align mode
              }

              return pov; // 0, 90, 180, 270
              */
            }));

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
    // driverController.leftBumper().onTrue(DriveCommands.resetPoseForAlliance(drive));
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
    ///// Shooter
    operatorController.shooter().hoodToUpPosition().onTrue(ShooterCommands.hoodUp(shooterHood));

    operatorController.shooter().hoodToDownPosition().onTrue(ShooterCommands.hoodDown(shooterHood));

    operatorController
        .shooter()
        .shooterRollerIn()
        .whileTrue(ShooterCommands.shooterIntake(shooterIntake));

    operatorController
        .shooter()
        .shooterRollerOut()
        .whileTrue(ShooterCommands.shooterOuttake(shooterIntake));

    operatorController
        .shooter()
        .shooterHighSpeed()
        .whileTrue(ShooterCommands.shootHighSpeed(shooter, shooterIntake));

    operatorController
        .shooter()
        .manualMoveHoodUp()
        .onTrue(ShooterCommands.hoodUpManual(shooterHood));

    operatorController
        .shooter()
        .manualMoveHoodDown()
        .onTrue(ShooterCommands.hoodDownManual(shooterHood));

    operatorController
        .shooter()
        .shooterLowSpeed()
        .whileTrue(ShooterCommands.shootLowSpeed(shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot5000()
        .whileTrue(ShooterCommands.shootAtVelocity(5000, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot4500()
        .whileTrue(ShooterCommands.shootAtVelocity(4500, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot4000()
        .whileTrue(ShooterCommands.shootAtVelocity(4000, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot3500()
        .whileTrue(ShooterCommands.shootAtVelocity(3500, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot3000()
        .whileTrue(ShooterCommands.shootAtVelocity(3000, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot2500()
        .whileTrue(ShooterCommands.shootAtVelocity(2500, shooter, shooterIntake));

    operatorController
        .shooter()
        .shoot2000()
        .whileTrue(ShooterCommands.shootAtVelocity(5600, shooter, shooterIntake));

    // Collector
    operatorController
        .collector()
        .deploy()
        .onTrue(CollectorCommands.deploy(collectorDeployer)); // TODO - verify

    operatorController
        .collector()
        .close()
        .onTrue(CollectorCommands.close(collectorDeployer)); // TODO - verify

    operatorController
        .collector()
        .intake()
        .whileTrue(CollectorCommands.intake(collectorExteriorRoller));

    operatorController
        .collector()
        .outtake()
        .whileTrue(CollectorCommands.outtake(collectorExteriorRoller));

    operatorController.collector().manualDeploy().whileTrue(new DeployCollector(collectorDeployer));
    // .whileTrue(CollectorCommands.manualDeploy(collectorDeployer)); // TODO - verify

    operatorController.collector().manualClose().whileTrue(new CloseCollector(collectorDeployer));
    // .whileTrue(CollectorCommands.manualClose(collectorDeployer)); // TODO - verify

    /*
     * put hood down
     * shoot at 5600 and run collector intake at the same time
     *
     */
    operatorController
        .combo()
        .passFar()
        .whileTrue(
            Commands.sequence(
                ShooterCommands.hoodDown(shooterHood),
                new WaitUntilCommand(() -> shooterHood.atTarget()),
                Commands.parallel(
                    ShooterCommands.shootAtVelocity(5600, shooter, shooterIntake),
                    CollectorCommands.intake(collectorExteriorRoller))));

    operatorController
        .combo()
        .passShort()
        .whileTrue(
            Commands.sequence(
                ShooterCommands.hoodDown(shooterHood),
                new WaitUntilCommand(() -> shooterHood.atTarget()),
                Commands.parallel(
                    ShooterCommands.shootAtVelocity(3500, shooter, shooterIntake),
                    CollectorCommands.intake(collectorExteriorRoller)))); // verify

    operatorController
        .combo()
        .reverse()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  shooter.shootAtVelocity(-5000);
                  shooterIntake.out();
                  collectorExteriorRoller.out();
                },
                () -> {
                  shooter.stop();
                  shooterIntake.stop();
                  collectorExteriorRoller.stop();
                },
                shooter,
                shooterIntake,
                collectorExteriorRoller));
  }

  private void configureDeveloperBindings() {
    // verify direction of encoder / motor for collector
    // we want positive output to move the enocoder towards the deployed state
    driverController
        .a()
        .whileTrue(
            new StartEndCommand(
                () -> collectorDeployer.setVoltage(1),
                () -> collectorDeployer.stop(),
                collectorDeployer));
    driverController
        .b()
        .whileTrue(
            new StartEndCommand(
                () -> collectorDeployer.setVoltage(-1),
                () -> collectorDeployer.stop(),
                collectorDeployer));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
