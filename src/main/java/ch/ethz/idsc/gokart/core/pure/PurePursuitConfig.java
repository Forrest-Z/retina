// code by jph
package ch.ethz.idsc.gokart.core.pure;

import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.AppResources;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.ref.FieldSubdivide;

/** parameters for PID controller of steering
 * 
 * there are 2 special units related to the manufacturer of the steering column:
 * "SCE" steer-column encoder
 * "SCT" steer-column torque */
public class PurePursuitConfig extends PursuitConfig {
  public static final PurePursuitConfig GLOBAL = AppResources.load(new PurePursuitConfig());
  /***************************************************/
  /** look ahead distance for pure pursuit controller
   * 20171218: changed from 2.8[m] to 3.5[m] otherwise tracked angle is out of range too frequently
   * 20180304: changed from 3.5[m] to 3.9[m] to match with value used many times before
   * 20180929: changed from 3.9[m] to 3.5[m]
   * TODO as look ahead as decreased -> increase pure pursuit update rate also */
  @FieldSubdivide(start = "2.5[m]", end = "4[m]", intervals = 6)
  public Scalar lookAhead = Quantity.of(3.5, SI.METER);

  public PurePursuitConfig() {
    super(Quantity.of(0.1, SI.SECOND));
  }
}
