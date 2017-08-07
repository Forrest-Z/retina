// code by jph
package ch.ethz.idsc.retina.dev.davis._240c;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.retina.dev.DimensionInterface;
import ch.ethz.idsc.retina.dev.davis.ApsDavisEventListener;
import ch.ethz.idsc.retina.dev.davis.ColumnTimedImageListener;
import ch.ethz.idsc.retina.util.GlobalAssert;

public class DavisImageProvider implements ApsDavisEventListener {
  private final int width;
  private final int height;
  private final int lastX;
  private final int lastY;
  // ---
  private final List<ColumnTimedImageListener> timedImageListeners = new LinkedList<>();
  private final BufferedImage bufferedImage;
  private final byte[] bytes;
  private final int[] time;

  public DavisImageProvider(DimensionInterface dimensionInterface) {
    width = dimensionInterface.getWidth();
    height = dimensionInterface.getHeight();
    lastX = width - 1;
    lastY = height - 1;
    bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
    DataBufferByte dataBufferByte = (DataBufferByte) bufferedImage.getRaster().getDataBuffer();
    bytes = dataBufferByte.getData();
    GlobalAssert.that(bytes.length == width * height);
    time = new int[width];
  }

  public void addListener(ColumnTimedImageListener timedImageListener) {
    timedImageListeners.add(timedImageListener);
  }

  @Override
  public void aps(ApsDavisEvent apsDavisEvent) {
    int intensity = apsDavisEvent.grayscale();
    int index = apsDavisEvent.x + (apsDavisEvent.y * width); // TODO should precompute?
    bytes[index] = (byte) intensity;
    if (apsDavisEvent.y == lastY) {
      time[apsDavisEvent.x] = apsDavisEvent.time;
      if (apsDavisEvent.x == lastX)
        // System.out.println(DeleteDuplicates.of(Differences.of(Tensors.vectorInt(time))));
        timedImageListeners.forEach(listener -> listener.image(time, bufferedImage));
    }
  }
}
