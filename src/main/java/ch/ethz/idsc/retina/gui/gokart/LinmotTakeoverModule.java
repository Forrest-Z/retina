// code by rvmoss and jph
package ch.ethz.idsc.retina.gui.gokart;

import java.util.Optional;

import ch.ethz.idsc.retina.dev.linmot.LinmotGetEvent;
import ch.ethz.idsc.retina.dev.linmot.LinmotGetListener;
import ch.ethz.idsc.retina.dev.linmot.LinmotPutConfiguration;
import ch.ethz.idsc.retina.dev.linmot.LinmotPutEvent;
import ch.ethz.idsc.retina.dev.linmot.LinmotPutListener;
import ch.ethz.idsc.retina.dev.linmot.LinmotPutProvider;
import ch.ethz.idsc.retina.dev.linmot.LinmotSocket;
import ch.ethz.idsc.retina.dev.zhkart.ProviderRank;
import ch.ethz.idsc.retina.sys.AbstractModule;
import ch.ethz.idsc.retina.util.data.TimedFuse;

public class LinmotTakeoverModule extends AbstractModule implements LinmotGetListener, LinmotPutListener, LinmotPutProvider {
  private static final double DURATION_S = 0.04;
  private static final double THRESHOLD_POS_DELTA = 20000;

  @Override
  protected void first() throws Exception {
    LinmotSocket.INSTANCE.addAll(this);
  }

  private boolean isActive = false;
  private final TimedFuse timedFuse = new TimedFuse(DURATION_S);

  @Override
  protected void last() {
    LinmotSocket.INSTANCE.removeAll(this);
  }

  @Override
  public void getEvent(LinmotGetEvent getEvent) {
    int difference = getEvent.demand_position - getEvent.actual_position;
    timedFuse.register(isActive && difference >= THRESHOLD_POS_DELTA);
  }

  @Override
  public ProviderRank getProviderRank() {
    return ProviderRank.EMERGENCY;
  }

  @Override
  public Optional<LinmotPutEvent> putEvent() {
    if (timedFuse.isBlown()) {
      LinmotPutEvent takeOverEvent = new LinmotPutEvent(LinmotPutConfiguration.CMD_OFF_MODE, LinmotPutConfiguration.MC_ZEROS);
      return Optional.of(takeOverEvent);
    }
    return Optional.empty();
  }

  @Override
  public void putEvent(LinmotPutEvent putEvent) {
    isActive = putEvent.control_word == LinmotPutConfiguration.CMD_OPERATION.getShort();
  }
}
