package frc.robot.subsystems.shooterintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterIntake extends SubsystemBase {
  private final ShooterIntakeIO io;
  private final ShooterIntakeIOInputsAutoLogged inputs = new ShooterIntakeIOInputsAutoLogged();

  public ShooterIntake(ShooterIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterIntake", inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void in() {
    io.setVoltage(6);
  }

  public void out() {
    io.setVoltage(-6);
  }

  public void stop() {
    io.stop();
  }
}
