package frc.robot.subsystems.collectorexterior;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorExteriorIO {
  @AutoLog
  class CollectorExteriorIOInputs {
    public double velocity = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public boolean connected = true;
  }

  default void updateInputs(CollectorExteriorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
