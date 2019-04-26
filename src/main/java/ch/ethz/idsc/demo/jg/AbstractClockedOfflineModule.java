package ch.ethz.idsc.demo.jg;

import java.nio.ByteBuffer;

import ch.ethz.idsc.gokart.lcm.OfflineLogListener;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Mod;

/* package */ abstract class AbstractClockedOfflineModule implements OfflineLogListener {
  private final Scalar updatePeriod;
  private final Mod MOD_FUNCTION;
  private Scalar nextUpdateUpdate = Quantity.of(0, SI.SECOND);

  public AbstractClockedOfflineModule(Scalar updatePeriod) {
    this.updatePeriod = updatePeriod;
    MOD_FUNCTION = Mod.function(updatePeriod);
  }

  @Override // from OfflineLogListener
  public void event(Scalar time, String channel, ByteBuffer byteBuffer) {
    process(time, channel, byteBuffer);
    Scalar update = time.subtract(MOD_FUNCTION.apply(time));
    if (Scalars.lessEquals(nextUpdateUpdate, update)) {
      nextUpdateUpdate = update.add(updatePeriod);
      runAlgo();
    }
  }

  abstract void process(Scalar time, String channel, ByteBuffer byteBuffer);

  abstract void runAlgo();
}
