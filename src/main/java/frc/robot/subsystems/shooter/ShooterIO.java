package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double encoderVelocity = 0;
    public double targetVelocity = 0;
    public double outputCurrent = 0;
    public boolean atTargetVelocity = false;
    public boolean isOn = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(double velocity) {}

  default boolean atTargetVelocity() {
    return false;
  }

  default void stop() {}
}
