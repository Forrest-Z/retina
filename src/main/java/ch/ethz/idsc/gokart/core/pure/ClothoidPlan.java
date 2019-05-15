// code by gjoel
package ch.ethz.idsc.gokart.core.pure;

import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owl.math.MinMax;
import ch.ethz.idsc.owl.math.planar.ClothoidPursuit;
import ch.ethz.idsc.sophus.group.Se2GroupElement;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.LinearInterpolation;
import ch.ethz.idsc.tensor.sca.Clips;

public class ClothoidPlan {
  private static final int REFINEMENT = 2;

  /** @param lookAhead {x[m], y[m], angle}
   * @param pose of vehicle {x[m], y[m], angle}
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @return ClothoidPlan */
  public static Optional<ClothoidPlan> from(Tensor lookAhead, Tensor pose, boolean isForward) {
    ClothoidPursuit clothoidPursuit = new ClothoidPursuit(lookAhead);
    Tensor curveSE2 = ClothoidPursuit.curve(lookAhead, REFINEMENT);
    if (!isForward)
      CurveClothoidPursuitHelper.mirrorAndReverse(curveSE2);
    Tensor curve = Tensor.of(curveSE2.stream().map(new Se2GroupElement(pose)::combine));
    return Optional.of(new ClothoidPlan(clothoidPursuit.ratios(), curve));
  }

  // ---
  private final Tensor ratios;
  private final Tensor curve;
  private Scalar length;

  /** @param ratios [m^-1] start and end, used to derive future heading in good precision
   * @param curve sparse planned to be followed */
  private ClothoidPlan(Tensor ratios, Tensor curve) {
    this.ratios = ratios;
    this.curve = curve;
  }

  public Tensor ratios() {
    return ratios.unmodifiable();
  }

  public Scalar ratio() {
    return ratio(RealScalar.ZERO);
  }

  /** @param progress [0, 1]
   * @return ratio [m^-1] */
  public Scalar ratio(Scalar progress) {
    return LinearInterpolation.of(ratios).At(Clips.interval(0, 1).requireInside(progress));
  }

  public Tensor curve() {
    return curve;
  }

  public Scalar length() {
    if (Objects.isNull(length))
      length = CurveClothoidPursuitHelper.curveLength(curve);
    return length;
  }
}