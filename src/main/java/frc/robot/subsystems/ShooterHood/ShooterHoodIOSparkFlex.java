package frc.robot.subsystems.ShooterHood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.*;

public class ShooterHoodIOSparkFlex implements ShooterHoodIO {

  private final SparkFlex spark;
  private final SparkClosedLoopController controller;
  private final AbsoluteEncoder encoder;

  private double targetRotations = 0.0;

  private final double FORWARD_LIMIT = 0.9;
  private final double REVERSE_LIMIT = 0.3;

  // Tunables
  private double kP = 3.0;
  private double kI = 0.0;
  private double kD = 0.1;
  private double cruiseVelocity = 2.0;
  private double acceleration = 4.0;
  private double allowedProfileError = 0.01;

  private double lastP, lastI, lastD;
  private double lastCruise, lastAccel, lastError;

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry cruiseEntry;
  private final DoubleEntry accelEntry;
  private final DoubleEntry errorEntry;

  public ShooterHoodIOSparkFlex(int canId) {

    spark = new SparkFlex(canId, MotorType.kBrushless);
    controller = spark.getClosedLoopController();
    encoder = spark.getAbsoluteEncoder();

    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("ShooterHood");

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

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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
  public void updateInputs(ShooterHoodIOInputs inputs) {

    checkForChanges();

    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = Math.abs(inputs.positionRotations - targetRotations) < allowedProfileError;

    inputs.kP = kP;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.cruiseVelocity = cruiseVelocity;
    inputs.acceleration = acceleration;
    inputs.allowedProfileError = allowedProfileError;
  }

  @Override
  public void setPositionRotations(double rotations) {
    targetRotations = rotations;

    controller.setSetpoint(rotations, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
