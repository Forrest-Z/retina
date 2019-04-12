// code by jph
package ch.ethz.idsc.gokart.core.pursuit.pure;

import ch.ethz.idsc.gokart.core.pursuit.DubendorfCurve;
import ch.ethz.idsc.gokart.core.pursuit.FigureBaseModule;
import ch.ethz.idsc.gokart.core.pursuit.PursuitConfig;
import ch.ethz.idsc.gokart.core.pursuit.pure.CurvePurePursuitModule;

public class FigureTiresAModule extends FigureBaseModule {
  /** until 20180226 the curve for trajectory pursuit was
   * DubendorfCurve.OVAL
   * 
   * due to new safety structure, the curve made a bit smaller and shifted slightly
   * in the direction away from the container. the new curve is
   * DubendorfCurve.OVAL_SHIFTED
   * 
   * then the hyperloop project was introduced to the hanger which further reduced
   * the operating domain for the gokart. the trajectory is now
   * DubendorfCurve.EIGHT_HYPERLOOP */
  public FigureTiresAModule() {
    super(new CurvePurePursuitModule(PursuitConfig.GLOBAL), DubendorfCurve.TIRES_TRACK_A);
  }
}
