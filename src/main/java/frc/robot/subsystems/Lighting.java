// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class Lighting extends SubsystemBase {
  // private AddressableLED ledLeft;
  // private AddressableLEDBuffer bufferLeft;

  private AddressableLED ledRight;
  private AddressableLEDBuffer bufferRight;

  int selectedR = 0;
  int selectedG = 0;
  int selectedB = 0;

  private int LED_COUNT = 72;

  /** Creates a new Lighting. */
  public Lighting() {
    // ledLeft = new AddressableLED(0);
    ledRight = new AddressableLED(1);

    // bufferLeft = new AddressableLEDBuffer(LED_COUNT);f
    bufferRight = new AddressableLEDBuffer(LED_COUNT);
    // ledLeft.setLength(bufferLeft.getLength());
    ledRight.setLength(bufferRight.getLength());

    // ledLeft.setData(bufferLeft);
    ledRight.setData(bufferRight);
    // ledLeft.start();
    ledRight.start();

    setColor(255, 255, 0);
  }

  double lastMatchTime = 30;
  boolean flashed = false;

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();

    if (!DriverStation.isAutonomous()
        && (matchTime < 30 && matchTime != -1)
        && DriverStation.isEnabled()) {
      if (lastMatchTime - matchTime > .5) {
        flashed = !flashed;
        lastMatchTime = matchTime;
      }
      if (flashed) {
        setColor(255, 255, 255);
      } else {
        setColor(selectedR, selectedG, selectedB);
      }
    } else if (!isHubActive()) {
      red();
    } else {
      green();
    }
  }

  public void on() {
    setColor(0, 255, 0);
    // ledLeft.start();
    // ledRight.start();
  }

  public void off() {
    setColor(0, 0, 0);
    // ledLeft.stop();
    ledRight.stop();
  }

  public void coral() {
    setColor(252, 251, 244);
  }

  public void algae() {
    setColor(0, 0, 255);
  }

  public void blank() {
    setColor(0, 0, 0);
  }

  public void green() {
    setColor(0, 255, 0);
  }

  public void red() {
    setColor(255, 0, 0);
  }

  public void purple() {
    setColor(128, 0, 128);
  }

  public void yellow() {
    setColor(255, 255, 0);
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public void setColor(int r, int g, int b) {
    if (!(r == 255 && g == 0 && b == 0)) {
      selectedR = r;
      selectedB = b;
      selectedG = g;
    }

    for (int i = 0; i < bufferRight.getLength(); i++) {
      // bufferLeft.setRGB(i, r, g, b);
      bufferRight.setRGB(i, r, g, b);
    }
    // ledLeft.setData(bufferLeft);
    ledRight.setData(bufferRight);
  }
}
