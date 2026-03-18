// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean DEBUG = true;

  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  // Enable / Disable Subsystems
  /// Shooter
  public static final boolean SHOOTER_ENABLED = true;
  public static final boolean SHOOTER_HOOD_ENABLED = true;
  public static final boolean SHOOTER_INTAKE_ENABLED = true;
  /// Collector
  public static final boolean COLLECTOR_DEPLOYER_ENABLED = true;
  public static final boolean COLLECTOR_ROLLER_EXTERIOR_ENABLED = false;

  // CAN ID's
  /// Shooter
  public static final int SHOOTER_LEADER_CAN_ID = 5;
  public static final int SHOOTER_FOLLOWER_CAN_ID = 6;
  public static final int SHOOTER_INTAKE_CAN_ID = 7;
  public static final int SHOOTER_HOOD_CAN_ID = 8;
  /// COLLECTOR
  public static final int COLLECTOR_DEPLOYER_CAN_ID = 27;
  public static final int COLLECTOR_DEPLOYER_ENCODER_CAN_ID = 19;
  public static final int COLLECTOR_ROLLER_EXTERIOR_CAN_ID = 25;
}
