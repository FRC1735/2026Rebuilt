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

  public static final double DEPLOYED_TARGET = 0.545; // TODO
  public static final double CLOSED_TARGET = 0.28; // TODO
  private static final double RANGE = 0.02; // TODO

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

  public CollectorDeployerIO.CollectorState getCollectorState() {
    return inputs.state;
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
  /*
  public boolean atTarget() {
    return inputs.atTarget;
  }*/

  public void setState(CollectorDeployerIO.CollectorState state) {
    io.setState(state);
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public void deploy() {
    setState(CollectorDeployerIO.CollectorState.DEPLOYED);
  }

  public void close() {
    setState(CollectorDeployerIO.CollectorState.CLOSED);
  }

  public void scaledDeploy() {
    double position = getPosition();

    if (position > DEPLOYED_TARGET) {
      setVoltage(0);
      return;
    }

    // slow
    boolean goSlow =
        position < CollectorDeployer.DEPLOYED_TARGET
            && position > CollectorDeployer.DEPLOYED_TARGET - 0.2; // 0.125;
    if (goSlow) {
      setVoltage(1);
    } else if (position < CollectorDeployer.DEPLOYED_TARGET) {
      setVoltage(5);
    } else {
      setVoltage(0);
    }
  }

  public void scaleClose() {
    // this seems to be good
    double position = getPosition();

    if (position < CLOSED_TARGET + 0.02) {
      setVoltage(0);
      return;
    }

    // go slow
    boolean goSlow =
        position > CollectorDeployer.CLOSED_TARGET
            && position < CollectorDeployer.CLOSED_TARGET + 0.2; // 0.15;
    if (goSlow) {
      setVoltage(-1);
    } else if (position > CollectorDeployer.CLOSED_TARGET) {
      setVoltage(-6.5);
    } else {
      setVoltage(0);
    }
  }

  public void hardClose() {
    // this seems to be good
    double position = getPosition();

    if (position < CLOSED_TARGET) {
      setVoltage(0);
      return;
    }

    // go slow
    setVoltage(-1);
  }

  public void keepClosed() {
    // this seems to be good
    double position = getPosition();

    if (position < CLOSED_TARGET + 0.04) {
      setVoltage(0);
      return;
    }

    // go slow
    setVoltage(-1);
  }

  public void keepOpen() {

    double position = getPosition();

    if (position > DEPLOYED_TARGET - 0.02) {
      setVoltage(0);
      return;
    }

    setVoltage(1);
  }
}
