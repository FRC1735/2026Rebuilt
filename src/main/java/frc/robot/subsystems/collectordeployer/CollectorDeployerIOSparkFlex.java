package frc.robot.subsystems.collectordeployer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.DetachedEncoder.Model;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CollectorDeployerIOSparkFlex implements CollectorDeployerIO {
  private final SparkFlex spark;
  private final SparkClosedLoopController controller;
  private final DetachedEncoder encoder;
  private final int detachedEncoderCanId;

  private double targetRotations = 0.0;

  private final double FORWARD_LIMIT = 0.9;
  private final double REVERSE_LIMIT = 0.3;

  // Tunables
  private double kP = 4.0;
  private double kI = 0.0;
  private double kD = 5.0;
  private double cruiseVelocity = 500.0;
  private double acceleration = 200.0;
  private double allowedProfileError = 0.01;

  private double lastP, lastI, lastD;
  private double lastCruise, lastAccel, lastError;

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry cruiseEntry;
  private final DoubleEntry accelEntry;
  private final DoubleEntry errorEntry;

  public CollectorDeployerIOSparkFlex(int motorCanId, int detachedEncoderCanId, String name) {
    this.detachedEncoderCanId = detachedEncoderCanId;

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);
    controller = spark.getClosedLoopController();
    // encoder = spark.getAbsoluteEncoder();

    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    kPEntry = table.getDoubleTopic("kP").getEntry(kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(kD);
    cruiseEntry = table.getDoubleTopic("cruiseVelocity").getEntry(cruiseVelocity);
    accelEntry = table.getDoubleTopic("acceleration").getEntry(acceleration);
    errorEntry = table.getDoubleTopic("allowedProfileError").getEntry(allowedProfileError);

    kPEntry.set(kP);
    kIEntry.set(kI);
    kDEntry.set(kD);
    cruiseEntry.set(cruiseVelocity);
    accelEntry.set(acceleration);
    errorEntry.set(allowedProfileError);

    configureSpark();
  }

  private void configureSpark() {

    var config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30);

    config
        .softLimit
        .forwardSoftLimit(FORWARD_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(REVERSE_LIMIT)
        .reverseSoftLimitEnabled(true);

    config
        .absoluteEncoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0)
        .inverted(false);

    DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();
    // TODO - expose as argument
    encoderConfig.dutyCycleOffset(0.2f);
    encoder.configure(encoderConfig, ResetMode.kNoResetSafeParameters);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder, detachedEncoderCanId)
        .outputRange(-1, 1)
        .p(kP)
        .i(kI)
        .d(kD);

    var motionConfig = new MAXMotionConfig();

    motionConfig
        .cruiseVelocity(cruiseVelocity)
        .maxAcceleration(acceleration)
        .allowedProfileError(allowedProfileError);

    config.closedLoop.apply(motionConfig);

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    lastP = kP;
    lastI = kI;
    lastD = kD;
    lastCruise = cruiseVelocity;
    lastAccel = acceleration;
    lastError = allowedProfileError;
  }

  private void checkForChanges() {

    kP = kPEntry.get();
    kI = kIEntry.get();
    kD = kDEntry.get();
    cruiseVelocity = cruiseEntry.get();
    acceleration = accelEntry.get();
    allowedProfileError = errorEntry.get();

    if (kP != lastP
        || kI != lastI
        || kD != lastD
        || cruiseVelocity != lastCruise
        || acceleration != lastAccel
        || allowedProfileError != lastError) {

      configureSpark();
    }
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    checkForChanges();

    inputs.encoderPosition = encoder.getAngle();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = Math.abs(inputs.encoderPosition - targetRotations) < allowedProfileError;

    inputs.kP = kP;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.cruiseVelocity = cruiseVelocity;
    inputs.acceleration = acceleration;
    inputs.allowedProfileError = allowedProfileError;
  }

  @Override
  public void setTarget(double rotations) {
    targetRotations = rotations;

    controller.setSetpoint(rotations, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
