package frc.robot.subsystems.collectordeployer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.DetachedEncoder.Model;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CollectorDeployerIOSparkFlex implements CollectorDeployerIO {

  private final SparkFlex spark;
  private final DetachedEncoder encoder;
  private final int detachedEncoderCanId;

  private final PIDController pidDeploy = null;
  private final PIDController pidRetract = null;

  private double targetRotations = 0.5;

  private static final double FORWARD_LIMIT = 0.6;
  private static final double REVERSE_LIMIT = 0.42;

  // Tunables
  private final double pDeploy = 0;
  private final double iDeploy = 0;
  private final double dDeploy = 0;
  private final double pRetract = 0;
  private final double iRetract = 0;
  private final double dRetract = 0;
  private final double allowedProfileError = 0;
  private final double target = 0;
  private boolean deploying = true;

  private boolean motorInverted;
  private final double kS;
  private final double kG;

  public CollectorDeployerIOSparkFlex(
      int motorCanId,
      int detachedEncoderCanId,
      String name,
      double pDeploy,
      double iDeploy,
      double dDeploy,
      double pRetract,
      double iRetract,
      double dRetract,
      double kS,
      double kG,
      boolean motorInverted) {

    this.detachedEncoderCanId = detachedEncoderCanId;
    this.motorInverted = motorInverted;
    this.kS = kS;
    this.kG = kG;

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);

    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    configureSpark();

    // make the current target whatever the encoder is when subsystem initialized
    targetRotations = encoder.getAngle();
  }

  private void configureSpark() {

    var config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(motorInverted);

    // NOTE: leaving this here, but these do not get applied because the motor is not aware of the
    // encoder
    config
        .softLimit
        .forwardSoftLimit(FORWARD_LIMIT)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(REVERSE_LIMIT)
        .reverseSoftLimitEnabled(false);

    DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();
    encoderConfig.dutyCycleOffset(0);

    encoder.configure(encoderConfig, ResetMode.kNoResetSafeParameters);

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    double position = encoder.getAngle();

    inputs.encoderPosition = position;
    inputs.velocityRPM = encoder.getVelocity();

    if (position > FORWARD_LIMIT && encoder.getVelocity() > 0) {
      stop();
    }

    if (position < REVERSE_LIMIT && encoder.getVelocity() < 0) {
      stop();
    }

    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = false; // deploying ? pidDeploy.atSetpoint() : pidRetract.atSetpoint();

    inputs.kP = 0; // deploying ? pDeploy : pRetract;
    inputs.kI = 0; // deploying ? iDeploy : iRetract;
    inputs.kD = 0; // deploying ? dDeploy : dRetract;

    inputs.allowedProfileError = 0;

    inputs.delta = position - targetRotations;
  }

  @Override
  public void setTarget(double rotations) {}

  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
