package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // TODO - invert so shooting is positive if needed
  private final int SHOOT_HIGH_VOLTAGE = 12;
  private final int SHOOT_LOW_VOLTAGE = 2;

  private final int SHOOT_HIGH_RPM = 6000;
  private final int SHOOT_LOW_RPM = 1500;

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

  public void shootAtHub() {
    io.setTargetVelocity(600);
    // io.setVoltage(3, 3);
  }

  public void shootHigh() {
    io.setTargetVelocity(450);
    // io.setVoltage(3, 3);
  }

  public void shootAcross() {
    io.setTargetVelocity(700);
    // io.setVoltage(3, 3);
  }

  public void shootLow() {
    io.setTargetVelocity(SHOOT_LOW_RPM);
  }

  public boolean atTargetVelocity() {
    return io.atTargetVelocity();
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
