package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double encoderVelocity = 0;
    public double targetVelocity = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(double velocity) {}

  default boolean atTargetVelocity() {
    return false;
  }

  default void stop() {}
}
