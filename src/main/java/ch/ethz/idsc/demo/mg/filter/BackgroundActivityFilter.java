// code by mg
package ch.ethz.idsc.demo.mg.filter;

import ch.ethz.idsc.demo.mg.DavisConfig;
import ch.ethz.idsc.retina.dev.davis._240c.DavisDvsEvent;

/** Implementation of background activity filter as presented in TODO MG find reference paper.
 * Events on the image boarders are always filtered. Smaller {@link filterConstant} results in more aggressive filtering */
public class BackgroundActivityFilter extends AbstractFilterHandler implements DavisDvsEventFilter {
  private final int width;
  private final int height;
  private final int[][] timestamps;
  private final double filterConstant;

  public BackgroundActivityFilter(DavisConfig davisConfig) {
    width = davisConfig.width.number().intValue();
    height = davisConfig.height.number().intValue();
    timestamps = new int[width][height];
    filterConstant = davisConfig.filterConstant.number().doubleValue();
  }

  @Override // from DavisDvsEventFilter
  public boolean filter(DavisDvsEvent davisDvsEvent) {
    updateNeighboursTimestamps(davisDvsEvent.x, davisDvsEvent.y, davisDvsEvent.time);
    return davisDvsEvent.time - timestamps[davisDvsEvent.x][davisDvsEvent.y] <= filterConstant;
  }

  /** updates all neighboring cells with the time stamp of the incoming event
   * 
   * @param x left-right pixel location, x=0 corresponds to far left
   * @param y up-down pixel location, y=0 corresponds to far up
   * @param time [us] */
  private void updateNeighboursTimestamps(int x, int y, int time) {
    // check if we are not on an edge and then update all 8 neighbors
    if (x != 0 && x != width - 1 && y != 0 && y != height - 1) {
      timestamps[x - 1][y] = time;
      timestamps[x + 1][y] = time;
      timestamps[x - 1][y - 1] = time;
      timestamps[x][y - 1] = time;
      timestamps[x + 1][y - 1] = time;
      timestamps[x - 1][y + 1] = time;
      timestamps[x][y + 1] = time;
      timestamps[x + 1][y + 1] = time;
    }
  }
}
