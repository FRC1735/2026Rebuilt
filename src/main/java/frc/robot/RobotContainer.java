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
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.elastic.OperatorToggles;
import frc.robot.limelight.LimelightLogger;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO.CollectorState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.util.KeyboardController;
import java.util.Set;
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
  private final SendableChooser<Integer> autoWaitChooser;
  private final SendableChooser<String> dotChooser;

  // Limelight logging
  private final LimelightLogger rearLimelightLogger = new LimelightLogger("limelight-rear");
  private final LimelightLogger fronLimelightLogger = new LimelightLogger("limelight-front");

  private final Lighting lighting = new Lighting();

  private final Pose2d outpostDotPose;
  private final Pose2d depotDotPose;
  private final Pose2d hubDotPose;

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
    autoWaitChooser = new SendableChooser<Integer>();
    dotChooser = new SendableChooser<String>();

    /*
     *
     * hood 0.41 and speed 3500
     */
    // Register comands for PathPlanner
    NamedCommands.registerCommand("shooter hood up", ShooterCommands.hoodUp(shooterHood));
    NamedCommands.registerCommand("shooter hood down", ShooterCommands.hoodDown(shooterHood));
    NamedCommands.registerCommand(
        "shoot high", ShooterCommands.shootHighSpeed(shooter, shooterIntake));
    NamedCommands.registerCommand("collect", CollectorCommands.intake(collectorExteriorRoller));
    NamedCommands.registerCommand(
        "shoot preloaded",
        AutoCommands.shootPreloaded(
            drive, shooter, shooterIntake, collectorDeployer, collectorExteriorRoller));
    NamedCommands.registerCommand(
        "shoot at depot",
        AutoCommands.shootAtDepot(shooter, shooterIntake, shooterHood).withTimeout(4));

    NamedCommands.registerCommand("Deploy Collector", new DeployCollector(collectorDeployer));
    NamedCommands.registerCommand("Close Collector", new CloseCollector(collectorDeployer));

    NamedCommands.registerCommand("hood to depot", ShooterCommands.hoodDepot(shooterHood));
    NamedCommands.registerCommand("wheels to X", Commands.runOnce(drive::stopWithX, drive));
    NamedCommands.registerCommand(
        "Auto Wait",
        Commands.defer(
            () -> {
              int waitTime =
                  autoWaitChooser.getSelected() != null ? autoWaitChooser.getSelected() : 8;
              return new WaitCommand(waitTime);
            },
            Set.of()));

        // depot dot
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      depotDotPose = new Pose2d(8.8, 7.548, Rotation2d.fromDegrees(180));
      outpostDotPose = new Pose2d(8.8, 0.570, Rotation2d.fromDegrees(180));
    } else {
      outpostDotPose = new Pose2d(8.2, 7.548, Rotation2d.fromDegrees(180));
      depotDotPose = new Pose2d(8.2, 0.570, Rotation2d.fromDegrees(180));
    }
    // center dot
    hubDotPose = new Pose2d(8.487, 4.059, Rotation2d.fromDegrees(180));

    // outpost dot

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720), 12);
    Command goToDepotDot =
        AutoBuilder.pathfindToPose(
            depotDotPose, constraints, 0.0 // Goal end velocity in meters/sec
            );
    Command goToCenterDot =
        AutoBuilder.pathfindToPose(
            hubDotPose, constraints, 0.0 // Goal end velocity in meters/sec
            );
    Command gotOutpostDot =
        AutoBuilder.pathfindToPose(
            outpostDotPose, constraints, 0.0 // Goal end velocity in meters/sec
<<<<<<< Updated upstream
            );     
    
=======
            );

>>>>>>> Stashed changes
    NamedCommands.registerCommand(
        "go to dot based on chooser",
        Commands.defer(
            () -> {
<<<<<<< Updated upstream
                String selectedDot = dotChooser.getSelected();
                if (selectedDot == null || selectedDot == "") {
                    return Commands.none();
                } else if (selectedDot == "depot") {
                    return goToDepotDot;
                } else if (selectedDot == "hub") {
                    return goToCenterDot;
                } else if (selectedDot == "outpost")
              return new WaitCommand(0);
            },
            Set.of()));


=======
              String selectedDot = dotChooser.getSelected();
              if (selectedDot == null || selectedDot == "") {
                return Commands.none();
              } else if (selectedDot == "depot") {
                return goToDepotDot;
              } else if (selectedDot == "hub") {
                return goToCenterDot;
              } else if (selectedDot == "outpost") return new WaitCommand(0);
            },
            Set.of()));

>>>>>>> Stashed changes
    // NamedCommands.registerCommand("align hub",
    // Commands.runOnce(DriveCommands.joystickDriveWithAutoAlign(drive));

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
<<<<<<< Updated upstream
    autoChooser.addOption("Test path to Depot Dot", goToDeplotDot);
=======
    autoChooser.addOption("Test path to Depot Dot", goToDepotDot);
>>>>>>> Stashed changes
    autoChooser.addOption(
        "Shoot Preloaded",
        Commands.sequence(
            DriveCommands.resetPoseForAlliance(drive),
            AutoCommands.shootPreloaded(
                drive, shooter, shooterIntake, collectorDeployer, collectorExteriorRoller)));

    autoWaitChooser.addOption("1", 1);
    autoWaitChooser.addOption("2", 2);
    autoWaitChooser.addOption("3", 3);
    autoWaitChooser.addOption("4", 4);
    autoWaitChooser.addOption("5", 5);
    autoWaitChooser.addOption("6", 6);
    autoWaitChooser.addOption("7", 7);
    autoWaitChooser.addOption("8", 8);
    autoWaitChooser.addOption("9", 9);
    autoWaitChooser.addOption("10", 10);
    autoWaitChooser.addOption("11", 11);
    autoWaitChooser.addOption("12", 12);
    autoWaitChooser.addOption("13", 13);
    autoWaitChooser.addOption("14", 14);

    SmartDashboard.putData("Auto Delay", autoWaitChooser);

    dotChooser.addOption("depot", "depot");
    dotChooser.addOption("center", "center");
    dotChooser.addOption("outpost", "outpost");

    SmartDashboard.putData("Dot Chooser", dotChooser);

    collectorDeployer.setDefaultCommand(new MaintainCollectorPositionCommand(collectorDeployer));

    configureDriverBindings();
    configureOperatorBindings();
    // configureDeveloperBindings();

    lighting.green();
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

                  return SidsSmartCommand.getTargetAngle(
                          drive.getXPosition(), 11.915, drive.getYPosition(), 4.034)
                      * 180
                      / Math.PI;

                } else {
                  return SidsSmartCommand.getTargetAngle(
                          drive.getXPosition(), 4.626, drive.getYPosition(), 4.034)
                      * 180
                      / Math.PI;
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

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driverController.leftBumper().onTrue(DriveCommands.resetPoseForAlliance(drive));
    driverController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> OperatorToggles.toggleVisionEnabled()));
  }

  private void configureOperatorBindings() {
    // Combo Commands
    operatorController
        .combo()
        .shootIntoHubAndCollect()
        .whileTrue(
            ShooterCommands.hoodUp(shooterHood)
                .andThen(
                    Commands.parallel(
                        ShooterCommands.shootAtVelocity(3000, shooter, shooterIntake),
                        CollectorCommands.intake(collectorExteriorRoller))));

    operatorController
        .combo()
        .shootIntoHub()
        .whileTrue(
            ShooterCommands.hoodUp(shooterHood)
                .andThen(ShooterCommands.shootAtVelocity(3000, shooter, shooterIntake)));

    operatorController // TODO - verify
        .combo()
        .passAndCollect()
        .whileTrue(
            new DeployCollector(collectorDeployer)
                .andThen(
                    Commands.parallel(
                        ShooterCommands.hoodDown(shooterHood),
                        CollectorCommands.intake(collectorExteriorRoller)
                        // TODO - restore
                        /* ,
                        Commands.sequence(
                            Commands.waitSeconds(1),
                            ShooterCommands.shootAtVelocity(
                                () -> ShooterCommands.velocityForPose(drive.getXPosition()),
                                shooter,
                                shooterIntake))))*/ )));

    operatorController
        .combo()
        .collect()
        .whileTrue(
            Commands.runOnce(() -> collectorDeployer.setState(CollectorState.DEPLOYED))
                .andThen(CollectorCommands.intake(collectorExteriorRoller)));

    operatorController // TODO - verify
        .combo()
        .storage()
        .onTrue(new CloseCollector(collectorDeployer));

    // Hood
    operatorController.hood().hoodToUpPosition().onTrue(ShooterCommands.hoodUp(shooterHood));
    operatorController.hood().hoodToDownPosition().onTrue(ShooterCommands.hoodDown(shooterHood));

    operatorController
        .hood()
        .hoodToAutoPosition()
        .whileTrue(
            Commands.parallel(
                new AutoTargetCommand(
                    drive,
                    shooterHood,
                    DriverStation.getAlliance().isPresent(),
                    DriverStation.getAlliance().get(),
                    3500),
                ShooterCommands.shootAtVelocity(3500, shooter, shooterIntake)));
    operatorController
        .hood()
        .hoodToAutoPositionLow()
        .whileTrue(
            Commands.parallel(
                new AutoTargetCommand(
                    drive,
                    shooterHood,
                    DriverStation.getAlliance().isPresent(),
                    DriverStation.getAlliance().get(),
                    2500),
                ShooterCommands.shootAtVelocity(2500, shooter, shooterIntake)));

    // Hood Override
    operatorController.hood().manualMoveHoodUp().onTrue(ShooterCommands.hoodUpManual(shooterHood));
    operatorController
        .hood()
        .manualMoveHoodDown()
        .onTrue(ShooterCommands.hoodDownManual(shooterHood));

    // Collector
    operatorController
        .collector()
        .in()
        .whileTrue(CollectorCommands.intake(collectorExteriorRoller));
    operatorController
        .collector()
        .out()
        .whileTrue(CollectorCommands.outtake(collectorExteriorRoller));
    operatorController.collector().up().onTrue(new CloseCollector(collectorDeployer));
    operatorController.collector().down().onTrue(new DeployCollector(collectorDeployer));
    operatorController.collector().hardClose().onTrue(new HardClose(collectorDeployer));

    // Collector Override
    operatorController
        .collector()
        .manualUp()
        .whileTrue(new ManualCloseCollector(collectorDeployer));
    operatorController
        .collector()
        .manualDown()
        .whileTrue(new ManualDeployCollector(collectorDeployer));

    // Shooter Override
    operatorController
        .shooter()
        .shoot5600()
        .whileTrue(ShooterCommands.shootAtVelocity(5600, shooter, shooterIntake));
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
        .shootAtDepot()
        .whileTrue(
            Commands.parallel(
                ShooterCommands.hoodDepot(shooterHood),
                AutoCommands.shootAtDepot(shooter, shooterIntake, shooterHood),
                CollectorCommands.intake(collectorExteriorRoller)));

    operatorController
        .shooter()
        .reverseShooterAndShooterIntake()
        .whileTrue(
            Commands.parallel(
                ShooterCommands.shooterOuttake(shooterIntake), ShooterCommands.reverse(shooter)));
    operatorController
        .shooter()
        .reverseShooterIntake()
        .whileTrue(ShooterCommands.shooterOuttake(shooterIntake));
  }

  private void configureDeveloperBindings() {
    // verify direction of encoder / motor for collector
    // we want positive output to move the enocoder towards the deployed state
    /*
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
                */

    // increment x position to test pose velocity
    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      new Pose2d(drive.getXPosition() + 1, 0, Rotation2d.fromDegrees(180)));
                }));

    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
