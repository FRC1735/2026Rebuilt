package frc.robot.subsystems.rollerintake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIntakeIO {
  @AutoLog
  class RollerIntakeIOInputs {
    public double velocity = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public boolean connected = true;
  }

  default void updateInputs(RollerIntakeIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
