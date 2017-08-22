// code by jph
package ch.ethz.idsc.retina.lcm.davis;

import ch.ethz.idsc.retina.dev.davis.DavisApsType;
import ch.ethz.idsc.retina.dev.davis.DavisDecoder;
import ch.ethz.idsc.retina.dev.davis._240c.Davis240c;
import ch.ethz.idsc.retina.dev.davis.data.DavisApsBlockCollector;
import ch.ethz.idsc.retina.dev.davis.data.DavisApsBlockListener;
import ch.ethz.idsc.retina.dev.davis.data.DavisApsColumnCompiler;
import ch.ethz.idsc.retina.dev.davis.data.DavisApsDatagramServer;
import ch.ethz.idsc.retina.dev.davis.data.DavisDvsBlockCollector;
import ch.ethz.idsc.retina.dev.davis.data.DavisDvsBlockListener;
import ch.ethz.idsc.retina.dev.davis.data.DavisDvsDatagramServer;
import ch.ethz.idsc.retina.dev.davis.data.DavisImuFrameCollector;
import ch.ethz.idsc.retina.dev.davis.data.RawDavisApsColumnCompiler;
import idsc.DavisImu;

/** collection of functionality that filters raw data for aps content
 * the aps content is encoded in timed column blocks and sent via {@link DavisApsDatagramServer}
 * the dvs content is encoded in packets with at most 300 events and sent via {@link DavisDvsDatagramServer}
 * the imu content is encoded as {@link DavisImu}
 * 
 * <p>tested on cameras:
 * <pre>
 * DAVIS FX2 02460045
 * </pre> */
public class DavisLcmServer {
  // ---
  public final DavisDecoder davisDecoder;

  /** @param serial for instance "FX2_02460045"
   * @param cameraId */
  public DavisLcmServer(String serial, String cameraId) {
    davisDecoder = Davis240c.INSTANCE.createDecoder();
    {
      DavisDvsBlockCollector davisDvsBlockCollector = new DavisDvsBlockCollector();
      DavisDvsBlockListener davisDvsBlockListener = new DavisDvsBlockPublisher(cameraId);
      davisDvsBlockCollector.setListener(davisDvsBlockListener);
      davisDecoder.addDvsListener(davisDvsBlockCollector);
    }
    {
      DavisApsBlockListener davisApsBlockListener = new DavisApsBlockPublisher(cameraId, DavisApsType.RST);
      DavisApsBlockCollector davisApsBlockCollector = new DavisApsBlockCollector();
      davisApsBlockCollector.setListener(davisApsBlockListener);
      // DavisApsCorrection davisApsCorrection = new DavisApsCorrection(serial);
      DavisApsColumnCompiler davisApsColumnCompiler = //
          new RawDavisApsColumnCompiler(davisApsBlockCollector);
      davisDecoder.addRstListener(davisApsColumnCompiler);
    }
    {
      DavisApsBlockListener davisApsBlockListener = new DavisApsBlockPublisher(cameraId, DavisApsType.SIG);
      DavisApsBlockCollector davisApsBlockCollector = new DavisApsBlockCollector();
      davisApsBlockCollector.setListener(davisApsBlockListener);
      // DavisApsCorrection davisApsCorrection = new DavisApsCorrection(serial);
      DavisApsColumnCompiler davisApsColumnCompiler = new RawDavisApsColumnCompiler(davisApsBlockCollector);
      davisDecoder.addSigListener(davisApsColumnCompiler);
    }
    {
      DavisImuFramePublisher davisImuFramePublisher = new DavisImuFramePublisher(cameraId);
      DavisImuFrameCollector davisImuFrameCollector = new DavisImuFrameCollector();
      davisImuFrameCollector.addListener(davisImuFramePublisher);
      davisDecoder.addImuListener(davisImuFrameCollector);
    }
  }

  /** @param length
   * @param data
   * @param time */
  public void append(int length, int[] data, int[] time) {
    for (int index = 0; index < length; ++index)
      davisDecoder.read(data[index], time[index]);
  }
}
