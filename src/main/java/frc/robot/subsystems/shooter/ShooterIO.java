package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double leftVelocityRadPerSec = 0.0;
    public double rightVelocityRadPerSec = 0.0;

    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;

    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;

    public boolean leftConnected = true;
    public boolean rightConnected = true;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double leftVolts, double rightVolts) {}

  default void stop() {
    setVoltage(0.0, 0.0);
  }
}
