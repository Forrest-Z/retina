// code by jph
package ch.ethz.idsc.gokart.core.pursuit.pure;

import ch.ethz.idsc.gokart.core.pursuit.DubendorfCurve;
import ch.ethz.idsc.gokart.core.pursuit.FigureBaseModule;
import ch.ethz.idsc.gokart.core.pursuit.PursuitConfig;
import ch.ethz.idsc.gokart.core.pursuit.pure.CurvePurePursuitModule;

public class FigureDucttapeModule extends FigureBaseModule {
  public FigureDucttapeModule() {
    super(new CurvePurePursuitModule(PursuitConfig.GLOBAL), DubendorfCurve.HYPERLOOP_DUCTTAPE);
  }
}
