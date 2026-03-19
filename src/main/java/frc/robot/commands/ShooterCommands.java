package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.rollerintake.RollerIntake;
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

  // Manually move hood up
  public static Command hoodUpManual(ShooterHood hood) {
    return Commands.runOnce(hood::manualUp, hood);
  }

  // Manually move hood down
  public static Command hoodDownManual(ShooterHood hood) {
    return Commands.runOnce(hood::manualDown, hood);
  }

  // Shoot at High Speed
  public static Command shootHighSpeed(Shooter shooter, RollerIntake shooterIntake) {
    return shootAtVelocity(5000, shooter, shooterIntake);
  }

  // Shoot at Low Speed
  public static Command shootLowSpeed(Shooter shooter, RollerIntake shooterIntake) {
    return shootAtVelocity(3500, shooter, shooterIntake);
  }

  // Shoot at a specified speed
  public static Command shootAtVelocity(
      double velocity, Shooter shooter, RollerIntake shooterIntake) {
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

  public static Command shooterIntake(RollerIntake shooterIntake) {
    return new StartEndCommand(shooterIntake::in, shooterIntake::stop, shooterIntake);
  }

  public static Command shooterOuttake(RollerIntake shooterIntake) {
    return new StartEndCommand(shooterIntake::out, shooterIntake::stop, shooterIntake);
  }
}
