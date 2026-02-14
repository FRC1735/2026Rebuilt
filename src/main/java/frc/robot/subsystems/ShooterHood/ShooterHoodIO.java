package frc.robot.subsystems.shooterhood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {

  @AutoLog
  class ShooterHoodIOInputs {
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double targetRotations = 0.0;
    public boolean atTarget = false;

    // Tunables (logged for replay)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double cruiseVelocity = 0.0;
    public double acceleration = 0.0;
    public double allowedProfileError = 0.0;
  }

  default void updateInputs(ShooterHoodIOInputs inputs) {}

  default void setPositionRotations(double rotations) {}

  default void stop() {}
}
