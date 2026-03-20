package frc.robot.subsystems.rollerintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerIntake extends SubsystemBase {
  private final RollerIntakeIO io;
  private final RollerIntakeIOInputsAutoLogged inputs = new RollerIntakeIOInputsAutoLogged();
  private final String name;

  public RollerIntake(RollerIntakeIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void in() {
    io.setVoltage(9);
  }

  public void out() {
    io.setVoltage(-9);
  }

  public void stop() {
    io.stop();
  }
}
