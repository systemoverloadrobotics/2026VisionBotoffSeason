package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ProgressWatchdogTest {

  @Test
  void notStalledWhileImproving() {
    var w = new ProgressWatchdog(1.0, 0.02);
    w.reset(0.0);
    assertFalse(w.update(0.5, 5.0));
    assertFalse(w.update(1.5, 4.0));
    assertFalse(w.update(2.5, 3.0));
  }

  @Test
  void stallsWhenNoMetricImprovesWithinTimeout() {
    var w = new ProgressWatchdog(1.0, 0.02);
    w.reset(0.0);
    assertFalse(w.update(0.5, 5.0));
    assertTrue(w.update(2.0, 5.0));
  }

  @Test
  void progressOnAnyMetricResetsTheTimer() {
    // Turning toward a target can make no distance progress for a while.
    // That is normal, not a jam - only both being flat means stuck.
    var w = new ProgressWatchdog(1.0, 0.02, 0.02);
    w.reset(0.0);
    assertFalse(w.update(0.5, 5.0, 1.0));
    assertFalse(w.update(2.0, 5.0, 0.5));
    assertFalse(w.update(2.5, 5.0, 0.4));
  }

  @Test
  void improvementSmallerThanThresholdDoesNotCount() {
    var w = new ProgressWatchdog(1.0, 0.5);
    w.reset(0.0);
    assertFalse(w.update(0.5, 5.0));
    assertTrue(w.update(2.0, 4.9));
  }
}
