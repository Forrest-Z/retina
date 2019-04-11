// code by jph
package ch.ethz.idsc.gokart.core.pursuit.pure;

import java.util.Optional;

import ch.ethz.idsc.gokart.core.pursuit.CurvePursuitModule;
import ch.ethz.idsc.gokart.core.pursuit.PursuitConfig;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class CurvePurePursuitModule extends CurvePursuitModule {
  public CurvePurePursuitModule(PursuitConfig pursuitConfig) {
    super(pursuitConfig);
  }

  @Override // from CurvePursuitModule
  protected synchronized Optional<Scalar> getRatio(Tensor pose) {
    Optional<Tensor> optionalCurve = this.optionalCurve; // copy reference instead of synchronize
    if (optionalCurve.isPresent())
      return CurvePurePursuitHelper.getRatio( //
          pose, //
          optionalCurve.get(), //
          closed, //
          isForward, //
          pursuitConfig.lookAheadMeter());
    System.err.println("no curve in pursuit pursuit");
    return Optional.empty();
  }
}
