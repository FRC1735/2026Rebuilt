package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.dualrollerintake.DualRollerIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterhood.ShooterHood;

public class ShooterCommands {
  private ShooterCommands() {}

  // Move hood to 'up' position
  public static Command hoodUp(ShooterHood hood) {
    return Commands.runOnce(hood::setPositionUp, hood);
  }

  // Move hood to 'down' position
  public static Command hoodDown(ShooterHood hood) {
    return Commands.runOnce(hood::setPositionDown, hood);
  }

  public static Command hoodDepot(ShooterHood hood) {
    return Commands.runOnce(hood::setPositionDepot, hood);
  }

  public static Command adjustHood(ShooterHood hood, double position) {
    return Commands.runOnce(
        () -> {
          hood.setPosition(position);
        },
        hood);
    // return Commands.runOnce(hood::setPositionDepot, hood);
  }

  // Manually move hood up
  public static Command hoodUpManual(ShooterHood hood) {
    return Commands.runOnce(hood::manualUp, hood);
  }

  // Manually move hood down
  public static Command hoodDownManual(ShooterHood hood) {
    return Commands.runOnce(hood::manualDown, hood);
  }

  // Shoot at High Speed
  public static Command shootHighSpeed(Shooter shooter, DualRollerIntake shooterIntake) {
    return shootAtVelocity(5600, shooter, shooterIntake);
  }

  // Shoot at Low Speed
  public static Command shootLowSpeed(Shooter shooter, DualRollerIntake shooterIntake) {
    return shootAtVelocity(3500, shooter, shooterIntake);
  }

  public static Command shootAtVelocity(
      Drive drive, Shooter shooter, DualRollerIntake shooterIntake) {
    return shootAtVelocity(drive.getXPosition(), shooter, shooterIntake);
  }

  // Shoot at a specified speed
  public static Command shootAtVelocity(
      double velocity, Shooter shooter, DualRollerIntake shooterIntake) {
    return Commands.parallel(
            Commands.run(() -> shooter.shootAtVelocity(velocity), shooter),
            Commands.waitUntil(shooter::atTargetVelocity)
                .andThen(Commands.run(shooterIntake::in, shooterIntake)))
        .finallyDo(
            () -> {
              shooter.stop();
              shooterIntake.stop();
            });
  }

  public static Command shooterIntake(DualRollerIntake shooterIntake) {
    return new StartEndCommand(shooterIntake::in, shooterIntake::stop, shooterIntake);
  }

  public static Command shooterOuttake(DualRollerIntake shooterIntake) {
    return new StartEndCommand(shooterIntake::out, shooterIntake::stop, shooterIntake);
  }

  public static double velocityForPose(double xPosition) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if (xPosition > 0 && xPosition < 2) {
        return 5600;
      } else if (xPosition > 2 && xPosition < 4) {
        return 4500;
      } else if (xPosition > 4 && xPosition < 6) {
        return 4000;
      } else if (xPosition > 6 && xPosition < 8) {
        return 3500;
      } else {
        return 3000;
      }
    } else {
      if (xPosition < 17 && xPosition > 15) {
        return 5600;
      } else if (xPosition < 15 && xPosition > 13) {
        return 4500;
      } else if (xPosition < 13 && xPosition > 11) {
        return 4000;
      } else if (xPosition < 11 && xPosition > 9) {
        return 3500;
      } else {
        return 3000;
      }
    }
  }
}
