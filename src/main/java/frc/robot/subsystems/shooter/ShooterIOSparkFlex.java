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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class ShooterIOSparkFlex implements ShooterIO {

  private final SparkFlex leader;
  private final SparkFlex follower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double targetVelocity = 0;

  // Tunables
  private double p = 0;
  private double i = 0;
  private double d = 0;

  private final DoubleEntry pEntry;
  private final DoubleEntry iEntry;
  private final DoubleEntry dEntry;

  private double lastP, lastI, lastD;

  // Dashboard keys
  private static final String kPrefix = "Shooter/";

  public ShooterIOSparkFlex(int leaderCanId, int followerCanId) {
    leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
    follower = new SparkFlex(followerCanId, MotorType.kBrushless);
    encoder = leader.getEncoder();
    pid = leader.getClosedLoopController();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(kPrefix);

    pEntry = table.getDoubleTopic("P").getEntry(p);
    iEntry = table.getDoubleTopic("I").getEntry(i);
    dEntry = table.getDoubleTopic("D").getEntry(d);

    pEntry.set(p);
    iEntry.set(i);
    dEntry.set(d);

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
        .p(p)
        .i(i)
        .d(d)
        .outputRange(-1, 1)
        .feedForward
        .kV((0.5 / 2722) * 10);

    var followerConfig = new SparkFlexConfig();
    followerConfig.follow(Constants.SHOOTER_LEADER_CAN_ID, true);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leader.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lastP = p;
    lastI = i;
    lastD = d;
  }

  private void checkForChanges() {

    p = pEntry.get();
    i = iEntry.get();
    d = dEntry.get();

    if (p != lastP || i != lastI || d != lastD) {
      configureMotors();
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    checkForChanges();

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
