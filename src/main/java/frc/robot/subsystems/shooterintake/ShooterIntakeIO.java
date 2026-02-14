package frc.robot.subsystems.shooterintake;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIntakeIO {

  @AutoLog
  class ShooterIntakeIOInputs {
    public double velocity = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public boolean connected = true;
  }

  default void updateInputs(ShooterIntakeIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
