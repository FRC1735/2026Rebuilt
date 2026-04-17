package frc.robot.subsystems.dualrollerintake;

import org.littletonrobotics.junction.AutoLog;

public interface DualRollerIntakeIO {
  @AutoLog
  class DualRollerIntakeIOInputs {
    public double velocity = 0.0;

    public double appliedVolts = 0.0;

    public double currentAmps = 0.0;

    public boolean connected = true;

    public boolean isOn = false;

    public boolean requestedOn = false;
  }

  default void updateInputs(DualRollerIntakeIOInputs inputs) {}

  default void setSpeed(double speed) {}

  default void stop() {
    setSpeed(0.0);
  }
}
