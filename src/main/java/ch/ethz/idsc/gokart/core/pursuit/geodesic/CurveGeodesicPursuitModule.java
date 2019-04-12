// code by jph, gjoel
package ch.ethz.idsc.gokart.core.pursuit.geodesic;

import ch.ethz.idsc.gokart.core.pursuit.CurvePursuitModule;
import ch.ethz.idsc.gokart.core.pursuit.PursuitConfig;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

import java.util.Optional;

public class CurveGeodesicPursuitModule extends CurvePursuitModule {
  public CurveGeodesicPursuitModule(PursuitConfig pursuitConfig) {
    super(pursuitConfig);
  }

  @Override // from CurvePursuitModule
  protected synchronized Optional<Scalar> getRatio(Tensor pose) {
    Optional<Tensor> optionalCurve = this.optionalCurve; // copy reference instead of synchronize
    if (optionalCurve.isPresent())
      return CurveGeodesicPursuitHelper.getRatio( //
          pose, //
          speed, //
          optionalCurve.get(), //
          isForward, //
          pursuitConfig.geodesic, //
          pursuitConfig.entryFinder, //
          pursuitConfig.ratioLimits);
    System.err.println("no curve in geodesic pursuit");
    return Optional.empty();
  }
}
