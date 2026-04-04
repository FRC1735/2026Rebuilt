package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SidsSmartCommandTest {
  @Test
  void testDistanceCalculation() {
    // Simple example: 2 + 2 = 4
    // double test1 = SidsSmartCommand.getTargetDistance(0, -3, 0, -4);
    // double test2 = SidsSmartCommand.getTargetAngle(0, -3, 0, -4);
    // assertEquals(5, test1, "The math should be 5");
    // assertEquals(Math.atan2(-4, -3), test2, "The math should be atan(4/3)");

    double test1 = SidsSmartCommand.getSpeedFromHoodAngle(30, 0.332, Math.PI / 4, 9.81);
    assertEquals(1, test1, "The math should be 5");
  }
}
