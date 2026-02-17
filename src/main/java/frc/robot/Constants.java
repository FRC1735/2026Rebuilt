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
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* CAN IDs */
  // SHOOTER
  public static final int SHOOTER_LEFT_CAN_ID = 10; // TODO
  public static final int SHOOTER_RIGHT_CAN_ID = 15; // TODO

  // SHOOTER HOOD
  public static final int SHOOTER_HOOD_CAN_ID = 8; // TODO

  // SHOOTER INTAKE
  public static final int SHOOTER_INTAKE_CAN_ID = 9; // TODO

  // COLLECTOR DEPLOYER
  public static final int COLLECTOR_DEPLOYER_CAN_ID = 99; // TODO
  public static final int COLLECTOR_DEPLOYER_DETACHED_ENCODER_CAN_ID = 18; // TODO

  // COLLECTOR EXTERIOR
  public static final int COLLECTOR_EXTERIOR_CAN_ID = 7; // TODO
}
