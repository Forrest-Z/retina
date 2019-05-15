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
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Min;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.red.Times;
import ch.ethz.idsc.tensor.sca.Abs;

// TODO JPH rename
public class CurveClothoidPursuitPlanner {
  private static final int REFINEMENT = 2;
  // ---
  private Optional<ClothoidPlan> plan = Optional.empty(); // TODO curve is very rough -> approximated deviation is way above actual value

  /** @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @param curve in world coordinates
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @param trajectoryEntryFinder strategy to find best re-entry point
   * @param ratioLimits depending on pose and speed
   * @return geodesic plan */
  public Optional<ClothoidPlan> getPlan( //
      Tensor pose, Scalar speed, Tensor curve, boolean isForward, //
      TrajectoryEntryFinder trajectoryEntryFinder, //
      List<DynamicRatioLimit> ratioLimits) {
    if (plan.isPresent()) {
      if (Scalars.lessEquals(ClothoidPursuitConfig.GLOBAL.minDistance, CurveClothoidPursuitHelper.distanceDriven(plan.get(), pose))) // TODO maybe introduce some margin
        replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
      else {
        Scalar deviation = plan.get().curve().stream().map(pose::subtract).map(Extract2D.FUNCTION).map(Norm._2::ofVector).reduce(Min::of).get();
        if (Scalars.lessEquals(ClothoidPursuitConfig.GLOBAL.deviationLimit, deviation))
          replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
      }
    } else
      replanning(pose, speed, curve, isForward, trajectoryEntryFinder, ratioLimits);
    if (plan.isPresent()) {
      Scalar progress = CurveClothoidPursuitHelper.distanceDriven(plan.get(), pose).divide(plan.get().length());
      plan.get().ratio(progress);
    }
    return plan;
  }

  /** @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @param curve in world coordinates
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @param trajectoryEntryFinder strategy to find best re-entry point
   * @param ratioLimits depending on pose and speed */
  private void replanning( //
      Tensor pose, Scalar speed, Tensor curve, boolean isForward, //
      TrajectoryEntryFinder trajectoryEntryFinder, //
      List<DynamicRatioLimit> ratioLimits) {
    TensorUnaryOperator tensorUnaryOperator = new Se2GroupElement(pose).inverse()::combine;
    Tensor tensor = Tensor.of(curve.stream().map(tensorUnaryOperator));
    if (!isForward)
      CurveClothoidPursuitHelper.mirrorAndReverse(tensor);
    /* Predicate<Scalar> isCompliant = CurveClothoidPursuitHelper.isCompliant(ratioLimits, pose, speed);
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
        Scalar length = CurveClothoidPursuitHelper.curveLength(ClothoidPursuit.curve(vector, REFINEMENT)); // [m]
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
}