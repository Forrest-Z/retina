// code by mh
package ch.ethz.idsc.gokart.core.map;

import ch.ethz.idsc.gokart.core.fuse.SafetyConfig;
import ch.ethz.idsc.gokart.core.perc.SimpleSpacialObstaclePredicate;
import ch.ethz.idsc.gokart.core.perc.SpacialXZObstaclePredicate;
import ch.ethz.idsc.gokart.gui.top.SensorsConfig;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.AppResources;
import ch.ethz.idsc.sophus.filter.Regularization2Step;
import ch.ethz.idsc.sophus.group.RnGeodesic;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Clip;

/** @see SafetyConfig */
public class TrackReconConfig {
  public static final TrackReconConfig GLOBAL = AppResources.load(new TrackReconConfig());
  /***************************************************/
  /** .
   * 20180226: changed from -1.0[m] to -0.90[m] because the sensor rack was lowered by ~8[cm]
   * 20181206: changed from -0.9[m] to -1.05[m] to detect a car tire flat on the ground as an obstacle */
  public Scalar vlp16_ZLo = Quantity.of(-1.05, SI.METER);
  /** 20181206: changed from +0.1[m] to -0.1[m] because artifacts were observed when driving fast
   * cause probably by the arrangement of the lidar on the sensor rack */
  public Scalar vlp16_ZHi = Quantity.of(-0.10, SI.METER);
  /** how fast should the refinement be */
  public Scalar trackRefinementLambda = Quantity.of(3, SI.ONE);
  /** how strong is the regularizer */
  public Scalar trackRefinementReg = Quantity.of(1, SI.ONE);
  private static final Scalar gdRadiusGrowth = Quantity.of(0.007, SI.METER);
  private static final Scalar gdRegularizer = RealScalar.of(0.0007);
  private static final Scalar gdLimits = RealScalar.of(0.4);
  private static final Scalar gdRadius = RealScalar.of(0.8);

  public Scalar getGdRadius() {
    return gdRadius.multiply(trackRefinementLambda);
  }

  public Scalar getGdLimits() {
    return gdLimits.multiply(trackRefinementLambda);
  }

  public Scalar getGdRegularizer() {
    return gdRegularizer.multiply(trackRefinementLambda).multiply(trackRefinementReg);
  }

  public Scalar getGdRadiusGrowth() {
    return gdRadiusGrowth.multiply(trackRefinementLambda);
  }

  public TensorUnaryOperator getRegularizationCyclic() {
    return Regularization2Step.cyclic(RnGeodesic.INSTANCE, getGdRegularizer());
  }

  public TensorUnaryOperator getRegularizationString() {
    return Regularization2Step.string(RnGeodesic.INSTANCE, getGdRegularizer());
  }

  /***************************************************/
  /** @return */
  private Clip vlp16_ZClip() {
    return Clip.function( //
        Magnitude.METER.apply(vlp16_ZLo), //
        Magnitude.METER.apply(vlp16_ZHi));
  }

  /** convenient way for the application layer to obtain an instance
   * without having to specify the geometric configuration
   * 
   * @return */
  public SpacialXZObstaclePredicate createSpacialXZObstaclePredicate() {
    return new SimpleSpacialObstaclePredicate(vlp16_ZClip(), SensorsConfig.GLOBAL.vlp16_incline);
  }
}
