package frc.robot.subsystems.collectordeployer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.DetachedEncoder.Model;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.TunableNumber;

public class CollectorDeployerIOSparkFlex implements CollectorDeployerIO {

  private final SparkFlex spark;
  private final SparkClosedLoopController controller;
  private final DetachedEncoder encoder;
  private final int detachedEncoderCanId;

  private double targetRotations = 0.5;

  private static final double FORWARD_LIMIT = 0.7;
  private static final double REVERSE_LIMIT = 0.3;

  // Tunables
  private final TunableNumber kP;
  private final TunableNumber kI;
  private final TunableNumber kD;
  private final TunableNumber cruiseVelocity;
  private final TunableNumber acceleration;
  private final TunableNumber allowedProfileError;
  private final TunableNumber target;
  private final double delta;

  public CollectorDeployerIOSparkFlex(int motorCanId, int detachedEncoderCanId, String name) {

    this.detachedEncoderCanId = detachedEncoderCanId;

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);
    controller = spark.getClosedLoopController();
    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    kP = new TunableNumber(table, "kP", 5);
    kI = new TunableNumber(table, "kI", 0.0);
    kD = new TunableNumber(table, "kD", 0.0);

    cruiseVelocity = new TunableNumber(table, "cruiseVelocity", 1.8);
    acceleration = new TunableNumber(table, "acceleration", 4.0);
    allowedProfileError = new TunableNumber(table, "allowedProfileError", 0.02);
    target = new TunableNumber(table, "targetRotations", 0.0);
    delta = target.get() - encoder.getAngle();

    configureSpark();
  }

  private void configureSpark() {

    var config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30);
    config.inverted(true);

    config
        .softLimit
        .forwardSoftLimit(FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(REVERSE_LIMIT)
        .reverseSoftLimitEnabled(true);

    DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();
    encoderConfig.dutyCycleOffset(0.2f);
    encoder.configure(encoderConfig, ResetMode.kNoResetSafeParameters);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder, detachedEncoderCanId)
        .positionWrappingEnabled(false)
        .p(kP.get())
        .i(kI.get())
        .d(kD.get())
        .feedForward
        .kS(0.25)
        .kV(0)
        .kA(0)
        .kG(0);

    var motionConfig = new MAXMotionConfig();

    motionConfig
        .cruiseVelocity(cruiseVelocity.get())
        .maxAcceleration(acceleration.get())
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedProfileError(allowedProfileError.get());

    config.closedLoop.apply(motionConfig);


    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void updateTunables() {

    boolean pidChanged =
        kP.hasChanged(1e-6)
            | kI.hasChanged(1e-6)
            | kD.hasChanged(1e-6)
            | cruiseVelocity.hasChanged(1e-3)
            | acceleration.hasChanged(1e-3)
            | allowedProfileError.hasChanged(1e-4);

    if (pidChanged) {
      configureSpark();
    }

    if (target.hasChanged(1e-6)) {
      targetRotations = target.get();
      /*
      controller.setSetpoint(
          targetRotations,
          SparkBase.ControlType.kMAXMotionPositionControl);
          */
    }
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    updateTunables();

    double position = encoder.getAngle();

    inputs.encoderPosition = position;
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = Math.abs(position - targetRotations) < allowedProfileError.get();

    inputs.kP = kP.get();
    inputs.kI = kI.get();
    inputs.kD = kD.get();
    inputs.cruiseVelocity = cruiseVelocity.get();
    inputs.acceleration = acceleration.get();
    inputs.allowedProfileError = allowedProfileError.get();

    inputs.delta = position - targetRotations;
  }

  @Override
  public void setTarget(double rotations) {
    targetRotations = rotations;

    System.out.println("JTA - setTarget: " + rotations);

    controller.setSetpoint(rotations, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
