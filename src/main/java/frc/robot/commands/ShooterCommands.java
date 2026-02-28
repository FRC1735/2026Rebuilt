package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return Commands.runOnce(hood::incrementTargetPosition, hood);
  }

  // Manually move hood down
  public static Command hoodDownManual(ShooterHood hood) {
    return Commands.runOnce(hood::decrementTargetPosition, hood);
  }

  public static Command shooterHighSpeed(Shooter shooter, RollerIntake shooterIntake) {
    return Commands.runOnce(shooter::shootHigh, shooter)
        .until(() -> shooter.atTargetVelocity())
        .andThen(Commands.run(shooterIntake::in));
  }

  public static Command shooterLowSpeed(Shooter shooter, RollerIntake shooterIntake) {
    return Commands.runOnce(shooter::shootLow, shooter)
        .until(() -> shooter.atTargetVelocity())
        .andThen(Commands.run(shooterIntake::in));
  }

  public static Command shooterStop(Shooter shooter) {
    return Commands.runOnce(shooter::stop, shooter);
  }
}
