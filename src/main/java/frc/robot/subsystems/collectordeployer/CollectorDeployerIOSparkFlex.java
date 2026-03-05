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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.TunableNumber;

public class CollectorDeployerIOSparkFlex implements CollectorDeployerIO {

  private final SparkFlex spark;
  private final DetachedEncoder encoder;
  private final int detachedEncoderCanId;

  private final PIDController pid;

  private double targetRotations = 0.5;

  private static final double FORWARD_LIMIT = 0.7;
  private static final double REVERSE_LIMIT = 0.3;

  // Tunables
  private final TunableNumber kP;
  private final TunableNumber kI;
  private final TunableNumber kD;
  private final TunableNumber allowedProfileError;
  private final TunableNumber target;

  public CollectorDeployerIOSparkFlex(int motorCanId, int detachedEncoderCanId, String name) {

    this.detachedEncoderCanId = detachedEncoderCanId;

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);

    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    kP = new TunableNumber(table, "kP", 5);
    kI = new TunableNumber(table, "kI", 0.0);
    kD = new TunableNumber(table, "kD", 0.0);

    allowedProfileError = new TunableNumber(table, "allowedError", 0.02);
    target = new TunableNumber(table, "targetRotations", 0.0);

    pid = new PIDController(kP.get(), kI.get(), kD.get());
    pid.setTolerance(allowedProfileError.get());

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

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void updateTunables() {

    if (kP.hasChanged(1e-6) || kI.hasChanged(1e-6) || kD.hasChanged(1e-6)) {

      pid.setPID(kP.get(), kI.get(), kD.get());
    }

    if (allowedProfileError.hasChanged(1e-6)) {
      pid.setTolerance(allowedProfileError.get());
    }

    if (target.hasChanged(1e-6)) {
      targetRotations = target.get();
    }
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    updateTunables();

    double position = encoder.getAngle();

    double output = pid.calculate(position, targetRotations);

    double volts = MathUtil.clamp(output, -12.0, 12.0);

    // TODO - I guess this is fine, but it might make more sense to use setSetpoint
    spark.setVoltage(volts);

    inputs.encoderPosition = position;
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = pid.atSetpoint();

    inputs.kP = kP.get();
    inputs.kI = kI.get();
    inputs.kD = kD.get();

    inputs.allowedProfileError = allowedProfileError.get();

    inputs.delta = position - targetRotations;
  }

  @Override
  public void setTarget(double rotations) {
    targetRotations = rotations;
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
