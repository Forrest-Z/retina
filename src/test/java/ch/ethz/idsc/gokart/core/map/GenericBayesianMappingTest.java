// code by jph
package ch.ethz.idsc.gokart.core.map;

import junit.framework.TestCase;

public class GenericBayesianMappingTest extends TestCase {
  public void testSimple() {
    GenericBayesianMapping gokartMappingModule = GenericBayesianMapping.createObstacleMapping();
    gokartMappingModule.start();
    gokartMappingModule.stop();
  }
}
