package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.Test;

class CsvLoggerTest {

  @Test
  void writesHeaderAndRows() throws IOException {
    CsvLogger logger = new CsvLogger("unit-test", "a", "b", "c");
    logger.logRow(1, 2.5, true);
    logger.logRow(3, 4.5, false);
    logger.close();

    List<String> lines = Files.readAllLines(logger.getPath());
    assertEquals("a,b,c", lines.get(0));
    assertEquals("1,2.5,true", lines.get(1));
    assertEquals("3,4.5,false", lines.get(2));

    Files.deleteIfExists(logger.getPath());
  }

  @Test
  void fileNameContainsPrefix() throws IOException {
    CsvLogger logger = new CsvLogger("aim", "x");
    logger.close();
    assertTrue(logger.getPath().getFileName().toString().startsWith("aim-"));
    Files.deleteIfExists(logger.getPath());
  }

  @Test
  void closeIsIdempotent() throws IOException {
    CsvLogger logger = new CsvLogger("idem", "x");
    logger.logRow(1);
    logger.close();
    logger.close();
    assertEquals(2, Files.readAllLines(logger.getPath()).size());
    Files.deleteIfExists(logger.getPath());
  }
}
