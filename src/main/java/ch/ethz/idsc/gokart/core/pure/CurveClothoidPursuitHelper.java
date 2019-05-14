// code by gjoel
package ch.ethz.idsc.gokart.core.pure;

import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

import ch.ethz.idsc.owl.bot.se2.glc.DynamicRatioLimit;
import ch.ethz.idsc.owl.math.planar.ClothoidPursuit;
import ch.ethz.idsc.owl.math.planar.Extract2D;
import ch.ethz.idsc.owl.math.planar.GeodesicPursuitInterface;
import ch.ethz.idsc.owl.math.planar.PseudoSe2CurveIntersection;
import ch.ethz.idsc.owl.math.planar.TrajectoryEntryFinder;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.sophus.group.Se2GroupElement;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.opt.LinearInterpolation;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.red.ArgMin;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Min;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.red.Times;
import ch.ethz.idsc.tensor.sca.Abs;

// TODO JPH rename
// TODO GJOEL split into helper (enum) and planner (class)
public enum CurveClothoidPursuitHelper {
  ;
  private static final int REFINEMENT = 2;
  // ---
  private static Optional<ClothoidPlan> plan = Optional.empty(); // TODO curve is very rough -> approximated deviation is way above actual value

  /** @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @param curve in world coordinates
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @param trajectoryEntryFinder strategy to find best re-entry point
   * @param ratioLimits depending on pose and speed
   * @return geodesic plan */
  public static Optional<ClothoidPlan> getPlan( //
      Tensor pose, Scalar speed, Tensor curve, boolean isForward, //
      TrajectoryEntryFinder trajectoryEntryFinder, //
      List<DynamicRatioLimit> ratioLimits) {
    if (!plan.isPresent())
      replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
    if (plan.isPresent()) {
      if (Scalars.lessEquals(ClothoidPursuitConfig.GLOBAL.minDistance, distanceDriven(pose))) // TODO maybe introduce some margin
        replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
      else {
        Scalar deviation = plan.get().curve().stream().map(pose::subtract).map(Extract2D.FUNCTION).map(Norm._2::ofVector).reduce(Min::of).get();
        if (Scalars.lessEquals(ClothoidPursuitConfig.GLOBAL.deviationLimit, deviation))
          replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
      }
      plan.ifPresent(p -> p.setRatio(LinearInterpolation.of(plan.get().ratios()).At(distanceDriven(pose).divide(plan.get().length()))));
    }
    return plan;
  }

  /** @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @param curve in world coordinates
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @param trajectoryEntryFinder strategy to find best re-entry point
   * @param ratioLimits depending on pose and speed */
  public static void replanning( //
      Tensor pose, Scalar speed, Tensor curve, boolean isForward, //
      TrajectoryEntryFinder trajectoryEntryFinder, //
      List<DynamicRatioLimit> ratioLimits) {
    TensorUnaryOperator tensorUnaryOperator = new Se2GroupElement(pose).inverse()::combine;
    Tensor tensor = Tensor.of(curve.stream().map(tensorUnaryOperator));
    if (!isForward)
      mirrorAndReverse(tensor);
    /* Predicate<Scalar> isCompliant = isCompliant(ratioLimits, pose, speed);
     * TensorScalarFunction mapping = vector -> dragonNightKingKnife(vector, isCompliant, speed);
     * Scalar var = ArgMinVariable.using(trajectoryEntryFinder, mapping, ClothoidPursuitConfig.GLOBAL.getOptimizationSteps()).apply(tensor);
     * Optional<Tensor> lookAhead = trajectoryEntryFinder.on(tensor).apply(var).point; */
    Optional<Tensor> lookAhead = new PseudoSe2CurveIntersection(ClothoidPursuitConfig.GLOBAL.minDistance).string(tensor);
    plan = lookAhead.map(vector -> ClothoidPlan.from(vector, pose, isForward).orElse(null));
  }

  /** @param vector
   * @param isCompliant
   * @param speed
   * @return quantity with unit [m] */
  public static Scalar dragonNightKingKnife(Tensor vector, Predicate<Scalar> isCompliant, Scalar speed) {
    if (Scalars.lessThan(ClothoidPursuitConfig.GLOBAL.minDistance, Norm._2.ofVector(Extract2D.FUNCTION.apply(vector)))) {
      GeodesicPursuitInterface geodesicPursuit = new ClothoidPursuit(vector);
      Tensor ratios = geodesicPursuit.ratios();
      if (ratios.stream().map(Tensor::Get).allMatch(isCompliant)) {
        Scalar length = curveLength(ClothoidPursuit.curve(vector, REFINEMENT)); // [m]
        // System.out.println("length=" + length);
        Scalar max = Abs.of(geodesicPursuit.ratios().stream().reduce(Max::of).get()).Get(); // [m^-1]
        // System.out.println("max=" + max);
        Scalar virtual = Times.of(ClothoidPursuitConfig.GLOBAL.scale, speed, max);
        // System.out.println("virtual=" + virtual);
        return length.add(virtual);
      }
    }
    return Quantity.of(DoubleScalar.POSITIVE_INFINITY, SI.METER);
  }

  /** mirror the points along the y axis and invert their orientation
   * @param se2points curve given by points {x,y,a} */
  /* package */ static void mirrorAndReverse(Tensor se2points) {
    se2points.set(Scalar::negate, Tensor.ALL, 0);
    se2points.set(Scalar::negate, Tensor.ALL, 2);
  }

  /** @param ratioLimits depending on pose and speed
   * @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @return predicate to determine whether ratio is compliant with all posed turning ratio limits */
  private static Predicate<Scalar> isCompliant(List<DynamicRatioLimit> ratioLimits, Tensor pose, Scalar speed) {
    return ratio -> ratioLimits.stream() //
        .map(dynamicRatioLimit -> dynamicRatioLimit.at(pose, speed)) //
        .allMatch(dynamicRatioLimit -> dynamicRatioLimit.isInside(ratio));
  }

  /** @param curve geodesic
   * @return approximated length of curve */
  /* package */ static Scalar curveLength(Tensor curve) {
    Tensor curve_ = Tensor.of(curve.stream().map(Extract2D.FUNCTION));
    int n = curve_.length();
    return curve_.extract(1, n).subtract(curve_.extract(0, n - 1)).stream() //
        .map(Norm._2::ofVector) //
        .reduce(Scalar::add).get();
  }

  /** @param pose of vehicle {x[m], y[m], angle}
   * @return [m] */
  private static Scalar distanceDriven(Tensor pose) {
    Tensor trajectory = plan.get().curve();
    int idx = ArgMin.of(Tensor.of(trajectory.stream().map(pose::subtract).map(Extract2D.FUNCTION).map(Norm._2::ofVector)));
    return idx > 0 ? curveLength(trajectory.extract(0, idx + 1)) : Quantity.of(0, SI.METER);
  }
}