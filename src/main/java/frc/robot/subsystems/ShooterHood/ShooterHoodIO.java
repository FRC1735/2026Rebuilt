package frc.robot.subsystems.ShooterHood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {

  @AutoLog
  class ShooterHoodIOInputs {
    public double velocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public boolean connected = true;
  }

  default void updateInputs(ShooterHoodIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
