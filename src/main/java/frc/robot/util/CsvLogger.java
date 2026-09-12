package frc.robot.util;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * One CSV file per command run, at full loop rate, under logs/.
 *
 * Console output stays throttled to about 1 Hz so a human can read it; this
 * file keeps every loop. On the XRP prototype nearly every root cause was
 * found by reading these files, and several were invisible at 1 Hz.
 *
 * DIFFERENT FROM THE XRP VERSION: that one flushed on every row, which was
 * fine on a laptop but is a synchronous flash write 50 times a second on the
 * roboRIO - a real source of loop overruns. This buffers and flushes on close.
 */
public class CsvLogger {

  private static final DateTimeFormatter TIMESTAMP_FORMAT =
      DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");
  private static final Path LOG_DIR = Paths.get("logs");

  private final BufferedWriter m_writer;
  private final Path m_path;
  private boolean m_closed;

  public CsvLogger(String prefix, String... headers) {
    try {
      Files.createDirectories(LOG_DIR);
      String filename =
          prefix + "-" + LocalDateTime.now().format(TIMESTAMP_FORMAT) + ".csv";
      m_path = LOG_DIR.resolve(filename);
      m_writer = Files.newBufferedWriter(m_path);
      m_writer.write(String.join(",", headers));
      m_writer.newLine();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** Appends one row. Buffered - not visible in the file until close(). */
  public void logRow(Object... values) {
    if (m_closed) {
      return;
    }
    try {
      StringBuilder line = new StringBuilder();
      for (int i = 0; i < values.length; i++) {
        if (i > 0) {
          line.append(',');
        }
        line.append(values[i]);
      }
      m_writer.write(line.toString());
      m_writer.newLine();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** Safe to call more than once - a command's end() can run twice. */
  public void close() {
    if (m_closed) {
      return;
    }
    m_closed = true;
    try {
      m_writer.close();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public Path getPath() {
    return m_path;
  }
}
