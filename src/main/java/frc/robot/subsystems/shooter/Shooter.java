package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void stop() {
    io.stop();
  }

  public double getLeftVelocityRadPerSec() {
    return inputs.leftVelocityRadPerSec;
  }

  public double getRightVelocityRadPerSec() {
    return inputs.rightAppliedVolts;
  }

  public boolean isConnected() {
    return inputs.leftConnected && inputs.rightConnected;
  }
}
