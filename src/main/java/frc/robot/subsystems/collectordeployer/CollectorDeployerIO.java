package frc.robot.subsystems.collectordeployer;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorDeployerIO {

  @AutoLog
  class CollectorDeployerIOInputs {
    public double encoderPosition = 0.0;
    public double velocityRPM = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double targetRotations = 0.0;
    public boolean atTarget = false;

    public double delta = 0;

    // Tunables (logged for replay)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kG = 0.0;
    public double cruiseVelocity = 0.0;
    public double acceleration = 0.0;
    public double allowedProfileError = 0.0;
  }

  default void updateInputs(CollectorDeployerIOInputs inputs) {}

  default void setTarget(double target) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
