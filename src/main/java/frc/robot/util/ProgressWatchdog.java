package frc.robot.util;

import java.util.Arrays;

/**
 * Gives up when the robot stops making progress toward its goal.
 *
 * A physically stuck robot - wedged against something, or slipping - looks
 * exactly like "arrived" to a PID: both errors stop changing. But no tolerance
 * is ever met, so without this the command would drive the motors forever and
 * hold the drivetrain, costing the driver control entirely.
 *
 * Tracks several metrics at once, and progress on ANY of them restarts the
 * timer. That distinction matters: while the robot is still turning to face a
 * target it can go a while with no distance progress, and that is normal, not
 * a jam. Only when every metric is flat is the robot actually stuck.
 */
public class ProgressWatchdog {

  private final double m_timeoutSeconds;
  private final double[] m_minImprovements;
  private final double[] m_bestSeen;
  private double m_lastProgressTimestamp;

  public ProgressWatchdog(double timeoutSeconds, double... minImprovements) {
    m_timeoutSeconds = timeoutSeconds;
    m_minImprovements = minImprovements.clone();
    m_bestSeen = new double[minImprovements.length];
  }

  public void reset(double nowSeconds) {
    Arrays.fill(m_bestSeen, Double.POSITIVE_INFINITY);
    m_lastProgressTimestamp = nowSeconds;
  }

  /**
   * @param metrics one value per configured threshold; smaller is better
   * @return true if no metric has improved within the timeout window
   */
  public boolean update(double nowSeconds, double... metrics) {
    boolean madeProgress = false;
    for (int i = 0; i < metrics.length && i < m_bestSeen.length; i++) {
      if (metrics[i] < m_bestSeen[i] - m_minImprovements[i]) {
        madeProgress = true;
      }
      m_bestSeen[i] = Math.min(m_bestSeen[i], metrics[i]);
    }
    if (madeProgress) {
      m_lastProgressTimestamp = nowSeconds;
      return false;
    }
    return nowSeconds - m_lastProgressTimestamp >= m_timeoutSeconds;
  }

  /** Closest this metric ever got. Report it when giving up. */
  public double bestSeen(int index) {
    return m_bestSeen[index];
  }
}
