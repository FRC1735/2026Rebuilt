package frc.robot.subsystems.shooterhood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {

  private final ShooterHoodIO io;
  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  private static final double TARGET_UP = 0.13; // 0.33; // REAL VALUE: 0.13;
  private static final double TARGET_DOWN = 0.85; // 0.7; // REAL VALUE: 0.9;
  private static final double TARGET_DEPOT = 0.41; // 0.7; // REAL VALUE: 0.9;

  public ShooterHood(ShooterHoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterHood", inputs);
  }

  public void setPositionRotations(double rotations) {
    double clamped = Math.max(TARGET_UP, Math.min(TARGET_DOWN, rotations));
    io.setTarget(clamped);
  }

  public void setPositionUp() {
    setPositionRotations(TARGET_UP);
  }

  public void setPositionDepot() {
    setPositionRotations(TARGET_DEPOT);
  }

  public void setPositionDown() {
    setPositionRotations(TARGET_DOWN);
  }

  public void setPosition(double position) {
    setPositionRotations(position);
  }

  public void manualDown() {
    double position = inputs.targetRotations;
    double incremented = position + 0.01;
    if (incremented > 1) {
      incremented = 1;
    }
    io.setTarget(incremented);
  }

  public void manualUp() {
    double position = inputs.targetRotations;
    double decremented = position - 0.01;

    if (decremented < 0) {
      decremented = 0;
    }
    io.setTarget(decremented);
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }
}
