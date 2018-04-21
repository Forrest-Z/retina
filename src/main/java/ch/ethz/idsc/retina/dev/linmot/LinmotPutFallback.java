// code by jph
package ch.ethz.idsc.retina.dev.linmot;

import java.util.Optional;

import ch.ethz.idsc.owl.math.state.ProviderRank;

/** when no other {@link LinmotPutProvider} controls the break, then
 * the break is commanded to be in operation mode with non-breaking position */
/* package */ enum LinmotPutFallback implements LinmotPutProvider {
  INSTANCE;
  // ---
  @Override // from LinmotPutProvider
  public ProviderRank getProviderRank() {
    return ProviderRank.FALLBACK;
  }

  @Override // from LinmotPutProvider
  public Optional<LinmotPutEvent> putEvent() {
    return Optional.of(LinmotBitCycler.INSTANCE.FALLBACK_OPERATION());
  }
}
