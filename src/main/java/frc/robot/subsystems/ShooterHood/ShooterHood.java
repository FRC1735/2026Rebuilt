package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {

  private final ShooterHoodIO io;
  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  private static final double MIN_ROT = 0; // TODO
  private static final double MAX_ROT = 1; // TODO

  public ShooterHood(ShooterHoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterHood", inputs);
  }

  public void setPositionRotations(double rotations) {
    double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
    io.setPositionRotations(clamped);
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }
}
