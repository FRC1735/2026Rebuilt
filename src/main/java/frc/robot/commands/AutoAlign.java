package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class AutoAlign extends Command {
  private final Drive drive;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier targetAngleDegSupplier;

  private final PIDController thetaController =
      new PIDController(0.0, 0.0, 0.0); // will be set from dashboard

  public AutoAlign(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier targetAngleDegSupplier) {

    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetAngleDegSupplier = targetAngleDegSupplier;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(2));

    addRequirements(drive);

    // Initialize dashboard values (only once ideally)
    SmartDashboard.putNumber("TurnPID/P", 4.0);
    SmartDashboard.putNumber("TurnPID/I", 0.0);
    SmartDashboard.putNumber("TurnPID/D", 0.2);
  }

  @Override
  public void initialize() {
    thetaController.reset();
  }

  @Override
  public void execute() {
    // Update PID from SmartDashboard (live tuning)
    double kP = SmartDashboard.getNumber("TurnPID/P", 4.0);
    double kI = SmartDashboard.getNumber("TurnPID/I", 0.0);
    double kD = SmartDashboard.getNumber("TurnPID/D", 0.2);
    thetaController.setPID(kP, kI, kD);

    double targetRad = Math.toRadians(targetAngleDegSupplier.getAsDouble());
    double currentRad = drive.getRotation().getRadians();

    double omega = thetaController.calculate(currentRad, targetRad);

    // Clamp omega
    omega =
        Math.max(
            Math.min(omega, drive.getMaxAngularSpeedRadPerSec()),
            -drive.getMaxAngularSpeedRadPerSec());

    // Driver translation
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();

    // Field-relative driving
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, drive.getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted (driver command)
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
