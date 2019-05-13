// code by jph
package ch.ethz.idsc.gokart.core.pure;

import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.ref.FieldSubdivide;

public abstract class PursuitConfig {
  public final Scalar updatePeriod;
  /** gokart velocity speed for curve follower module
   * 20180531 the rate was increased to 75[s^-1]
   * 20180604 the rate was decreased to 50[s^-1] because of the presence of the tents */
  @FieldSubdivide(start = "30[s^-1]", end = "70[s^-1]", intervals = 4)
  public Scalar rateFollower = Quantity.of(50.0, SI.PER_SECOND);

  protected PursuitConfig(Scalar updatePeriod) {
    this.updatePeriod = updatePeriod;
  }
}
