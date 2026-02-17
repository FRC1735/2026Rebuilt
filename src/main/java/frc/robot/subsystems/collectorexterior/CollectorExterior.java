package frc.robot.subsystems.collectorexterior;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CollectorExterior extends SubsystemBase {
  private final CollectorExteriorIO io;
  private final CollectorExteriorIOInputsAutoLogged inputs =
      new CollectorExteriorIOInputsAutoLogged();

  public CollectorExterior(CollectorExteriorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Collector Exterior", inputs);
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
