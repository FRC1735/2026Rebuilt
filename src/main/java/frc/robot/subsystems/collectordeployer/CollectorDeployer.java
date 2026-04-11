package frc.robot.subsystems.collectordeployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO - there is a ton of stuff in here that doesn't apply to current setup
// TODO - the entry point methods are doing the bang-bang approach from old collector
public class CollectorDeployer extends SubsystemBase {

  private final CollectorDeployerIO io;
  private final CollectorDeployerIOInputsAutoLogged inputs =
      new CollectorDeployerIOInputsAutoLogged();

  private static final double MIN_ROT = 0.2; // TODO
  private static final double MAX_ROT = 0.7; // TODO

  public static final double DEPLOYED_TARGET = 0.565; // TODO
  public static final double CLOSED_TARGET = 0.26; // TODO
  private static final double RANGE = 0.02; // TODO

  public enum CollectorState {
    DEPLOYED,
    CLOSED,
    MOVING
  }

  private final String name;

  public CollectorDeployer(CollectorDeployerIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public CollectorState getCollectorState() {
    if (inputs.encoderPosition > DEPLOYED_TARGET - 0.01) {
      return CollectorState.DEPLOYED;
    } else if (inputs.encoderPosition < CLOSED_TARGET + 0.01) {
      return CollectorState.CLOSED;
    } else {
      return CollectorState.MOVING;
    }
  }

  public void setTarget(double rotations) {
    double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
    io.setTarget(clamped);
  }

  public void stop() {
    io.stop();
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public void deploy() {
    setTarget(DEPLOYED_TARGET);
  }

  public void close() {
    setTarget(CLOSED_TARGET);
  }

  double MANUAL_VOLTAGE = 7.5;
  double SLOW_VOLTAGE = 1;

  public void manualDeploy() {
    if (getPosition() < (DEPLOYED_TARGET - 0.5)) {
      setVoltage(SLOW_VOLTAGE);
    } else if (getPosition() < DEPLOYED_TARGET) {
      setVoltage(MANUAL_VOLTAGE);
    }
  }

  public void manualClose() {
    if (getPosition() < (CLOSED_TARGET + 0.5)) {
      setVoltage(-SLOW_VOLTAGE);
    } else if (getPosition() > CLOSED_TARGET) {
      setVoltage(-MANUAL_VOLTAGE);
    }
  }
}
