// code by jph
package ch.ethz.idsc.retina.dev.linmot;

import java.util.Optional;

import ch.ethz.idsc.retina.dev.zhkart.ProviderRank;
import ch.ethz.idsc.retina.util.data.PenaltyCards;
import ch.ethz.idsc.tensor.Scalar;

/**  */
public enum LinmotFireFighter implements LinmotGetListener, LinmotPutProvider {
  INSTANCE;
  // ---
  private final PenaltyCards penaltyCards = new PenaltyCards();

  @Override // from LinmotGetListener
  public void getEvent(LinmotGetEvent linmotGetEvent) {
    Scalar temperature = linmotGetEvent.getWindingTemperatureMax();
    penaltyCards.evaluate( //
        !LinmotConfig.GLOBAL.isTemperatureOperationSafe(temperature), // issue yellow card ?
        !LinmotConfig.GLOBAL.isTemperatureHardwareSafe(temperature)); // issue red card ?
  }

  /***************************************************/
  @Override // from LinmotPutProvider
  public ProviderRank getProviderRank() {
    return ProviderRank.HARDWARE;
  }

  @Override // from LinmotPutProvider
  public Optional<LinmotPutEvent> putEvent() {
    return Optional.ofNullable(penaltyCards.isPenalty() ? LinmotPutHelper.FALLBACK_OPERATION : null);
  }
}
