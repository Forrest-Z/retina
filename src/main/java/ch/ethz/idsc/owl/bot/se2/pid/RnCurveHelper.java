// code by mcp
package ch.ethz.idsc.owl.bot.se2.pid;

import ch.ethz.idsc.sophus.planar.ArcTan2D;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum RnCurveHelper {
  ;
  /** @param optionalCurve
   * @return if enough elements in curve */
  // TODO MCP function would be obsolete if addAngleToCurve would return Optional<Tensor>
  public static boolean bigEnough(Tensor optionalCurve) {
    return 1 < optionalCurve.length();
  }

  /** @param curve
   * @return appends angle between two following points on the curve */
  public static Tensor addAngleToCurve(Tensor curve) {
    Tensor value = Tensors.empty();
    for (int index = 0; index < curve.length(); ++index) {
      int nextIndex = (index + 1) % curve.length();
      Tensor prev = curve.get(index);
      value.append(prev.append(ArcTan2D.of(curve.get(nextIndex).subtract(prev))));
    }
    return value;
  }

  public static boolean isSe2Curve(Tensor curve) {
    // System.out.println(Pretty.of(curve.get()));
    curve.iterator().next();
    // TODO expect that curve has proper units (?)
    // TODO either demand that se2 curve is provided or append angles ...
    // TODO if invalid -> optionalCurve = Optional.empty() return false;
    return true;
  }
}