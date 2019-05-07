// code by jph
package ch.ethz.idsc.gokart.calib.steer;

import ch.ethz.idsc.gokart.dev.steer.SteerColumnInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;

public abstract class AbstractSteerMapping implements SteerMapping {
  private final ScalarUnaryOperator column2steer;
  private final ScalarUnaryOperator steer2column;

  protected AbstractSteerMapping(ScalarUnaryOperator column2steer, ScalarUnaryOperator steer2column) {
    this.column2steer = column2steer;
    this.steer2column = steer2column;
  }

  @Override // from SteerMapping
  public final Scalar getAngleFromSCE(SteerColumnInterface steerColumnInterface) {
    return getAngleFromSCE(steerColumnInterface.getSteerColumnEncoderCentered());
  }

  @Override // from SteerMapping
  public final Scalar getAngleFromSCE(Scalar scalar) {
    return column2steer.apply(scalar);
  }

  @Override // from SteerMapping
  public final Scalar getSCEfromAngle(Scalar angle) {
    return steer2column.apply(angle);
  }
}
