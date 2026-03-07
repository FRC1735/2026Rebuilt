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

  private final PIDController pidDeploy;
  private final PIDController pidRetract;

  private double targetRotations = 0.5;

  private static final double FORWARD_LIMIT = 0.7;
  private static final double REVERSE_LIMIT = 0.3;

  // Tunables
  private final double pDeploy;
  private final double iDeploy;
  private final double dDeploy;
  private final double pRetract;
  private final double iRetract;
  private final double dRetract;
  private final TunableNumber allowedProfileError;
  private final TunableNumber target;
  private boolean deploying = true;

  private boolean motorInverted;
  private float encoderOffset;
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
      boolean motorInverted,
      float encoderOffset) {

    this.detachedEncoderCanId = detachedEncoderCanId;
    this.motorInverted = motorInverted;
    this.encoderOffset = encoderOffset;
    this.kS = kS;
    this.kG = kG;

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);

    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    this.pDeploy = pDeploy;
    this.iDeploy = iDeploy;
    this.dDeploy = dDeploy;
    this.pRetract = pRetract;
    this.iRetract = iRetract;
    this.dRetract = dRetract;

    allowedProfileError = new TunableNumber(table, "allowedError", 0.02);
    target = new TunableNumber(table, "targetRotations", 0.0);

    pidDeploy = new PIDController(pDeploy, iDeploy, dDeploy);
    pidRetract = new PIDController(pRetract, iRetract, dRetract);
    pidDeploy.setTolerance(allowedProfileError.get());
    pidRetract.setTolerance(allowedProfileError.get());

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
    encoderConfig.dutyCycleOffset(encoderOffset);

    encoder.configure(encoderConfig, ResetMode.kNoResetSafeParameters);

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void updateTunables() {
    /*
    if (pDeploy.hasChanged(1e-6) || iDeploy.hasChanged(1e-6) || dDeploy.hasChanged(1e-6)) {

      pid.setPID(pDeploy.get(), iDeploy.get(), dDeploy.get());
    }

    if (allowedProfileError.hasChanged(1e-6)) {
      pid.setTolerance(allowedProfileError.get());
    }

    if (target.hasChanged(1e-6)) {
      targetRotations = target.get();
    }
      */
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    updateTunables();

    double position = encoder.getAngle();

    double angleRadians = position * 2 * Math.PI;

    // TODO - I guess this is fine, but it might make more sense to use setSetpoint
    if (deploying) {
      // double output = pid.calculate(position, targetRotations);
      double pidOutput = pidDeploy.calculate(position, targetRotations);
      double output = pidOutput + Math.copySign(kS, pidOutput) + (-kG * Math.cos(angleRadians));

      double volts = MathUtil.clamp(output, -12.0, 12.0);
      if (volts > -0.25) {
        volts = -0.25;
      }
      if (!pidDeploy.atSetpoint()) {
        spark.setVoltage(volts);
      }
    } else {

      // double output = pid.calculate(position, targetRotations);
      double pidOutput = pidRetract.calculate(position, targetRotations);
      double output = pidOutput + Math.copySign(kS, pidOutput) + (-kG * Math.cos(angleRadians));

      double volts = MathUtil.clamp(output, -12.0, 12.0);
      if (volts > 1) {
        volts = 1;
      }
      if (!pidRetract.atSetpoint()) {
        spark.setVoltage(volts);
      }
    }

    inputs.encoderPosition = position;
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = deploying ? pidDeploy.atSetpoint() : pidRetract.atSetpoint();

    inputs.kP = deploying ? pDeploy : pRetract;
    inputs.kI = deploying ? iDeploy : iRetract;
    inputs.kD = deploying ? dDeploy : dRetract;

    inputs.allowedProfileError = allowedProfileError.get();

    inputs.delta = position - targetRotations;
  }

  @Override
  public void setTarget(double rotations) {
    targetRotations = rotations;
    if (targetRotations < encoder.getAngle()) {
      deploying = true;
    } else {
      deploying = false;
    }
  }

  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
