package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class ShooterIOSparkFlex implements ShooterIO {

  private final SparkFlex leader;
  private final SparkFlex follower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double targetVelocity = 0;

  // Dashboard keys
  private static final String kPrefix = "Shooter/";

  public ShooterIOSparkFlex(int leaderCanId, int followerCanId) {
    leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
    follower = new SparkFlex(followerCanId, MotorType.kBrushless);
    encoder = leader.getEncoder();
    pid = leader.getClosedLoopController();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(kPrefix);

    configureMotors();
  }

  private void configureMotors() {
    var leadConfig = new SparkFlexConfig();

    leadConfig.idleMode(IdleMode.kCoast);
    leadConfig.smartCurrentLimit(60);

    leadConfig.encoder.velocityConversionFactor(1);

    leadConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .feedForward
        .kV((0.5 / 2722) * 10);

    var followerConfig = new SparkFlexConfig();
    followerConfig.follow(Constants.SHOOTER_LEADER_CAN_ID, true);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leader.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.encoderVelocity = encoder.getVelocity();
    inputs.targetVelocity = targetVelocity;

    // update pid?
    if (targetVelocity == 0) {
      leader.set(0);
    } else {
      pid.setSetpoint(targetVelocity, ControlType.kVelocity);
    }
  }

  @Override
  public void setTargetVelocity(double targetVelocity) {
    this.targetVelocity = targetVelocity;
    // leader.set(0.5);
  }

  @Override
  public void stop() {
    this.targetVelocity = 0;
    leader.stopMotor();
  }
}
