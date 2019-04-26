// code by gjoel
package ch.ethz.idsc.demo.jg;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

import ch.ethz.idsc.gokart.core.pos.GokartPoseEvent;
import ch.ethz.idsc.gokart.core.pos.GokartPoseHelper;
import ch.ethz.idsc.gokart.core.pure.DubendorfCurve;
import ch.ethz.idsc.gokart.core.pure.GeodesicPursuitParams;
import ch.ethz.idsc.gokart.core.pure.PursuitConfig;
import ch.ethz.idsc.gokart.core.pure.TrajectoryEvents;
import ch.ethz.idsc.gokart.gui.GokartLcmChannel;
import ch.ethz.idsc.gokart.lcm.OfflineLogPlayer;
import ch.ethz.idsc.owl.bot.se2.glc.DynamicRatioLimit;
import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.math.planar.ArgMinVariable;
import ch.ethz.idsc.owl.math.planar.Extract2D;
import ch.ethz.idsc.owl.math.planar.GeodesicPursuit;
import ch.ethz.idsc.owl.math.planar.GeodesicPursuitInterface;
import ch.ethz.idsc.owl.math.planar.TrajectoryEntryFinder;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.sophus.group.Se2GroupElement;
import ch.ethz.idsc.sophus.math.GeodesicInterface;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.opt.TensorScalarFunction;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.red.Norm;

/* package */ class OfflineGeodesicPursuitModule extends AbstractClockedOfflineModule {
  private final FakeCurveGeodesicPursuitHelper pursuit = new FakeCurveGeodesicPursuitHelper();
  private final PursuitConfig pursuitConfig;
  // ---
  private Scalar time;
  private Tensor pose = Tensors.empty();
  private Tensor referenceCurve = Tensors.empty();

  public OfflineGeodesicPursuitModule(PursuitConfig pursuitConfig) {
    super(PursuitConfig.GLOBAL.updatePeriod);
    this.pursuitConfig = pursuitConfig;
  }

  public OfflineGeodesicPursuitModule(PursuitConfig pursuitConfig, Tensor curve) {
    super(PursuitConfig.GLOBAL.updatePeriod);
    this.pursuitConfig = pursuitConfig;
    referenceCurve = curve;
  }

  @Override // from AbstractClockedOfflineModule
  public void process(Scalar time, String channel, ByteBuffer byteBuffer) {
    this.time = time;
    switch (channel) {
      case GokartLcmChannel.POSE_LIDAR:
        GokartPoseEvent gokartPoseEvent = GokartPoseEvent.of(byteBuffer);
        pose = gokartPoseEvent.getPose();
        break;
      case GokartLcmChannel.TRAJECTORY_XYAT_STATETIME:
        Tensor trajectory = Tensor.of(TrajectoryEvents.trajectory(byteBuffer).stream().map(TrajectorySample::stateTime).map(StateTime::state));
        setReferenceCurve(trajectory);
        break;
    }
  }

  @Override // from AbstractClockedOfflineModule
  public void runAlgo() {
    if (Tensors.nonEmpty(pose)) {
      Scalar speed = RealScalar.ONE; // TODO implement
      boolean isForward = true; // TODO implement
      Optional<Scalar> ratio = pursuit.getRatio(pose, speed, referenceCurve, isForward, //
          pursuitConfig.geodesicInterface, //
          pursuitConfig.trajectoryEntryFinder, //
          PursuitConfig.ratioLimits());
      System.out.println("time: " + time.number() + "\n\tpose: " + pose + "\n\tratio: " + ratio + "\n\tcurve: " + pursuit.curve);
    }
  }

  public void setReferenceCurve(Tensor curve) {
    referenceCurve = curve;
  }

  public static void main(String[] args) throws Exception {
    File file = new File(args[0]);
    System.out.println("processing " + file.getAbsolutePath());
    GlobalAssert.that(file.isFile() && file.getName().endsWith(".lcm.00"));
    // ---
    OfflineLogPlayer.process(file, new OfflineGeodesicPursuitModule(PursuitConfig.GLOBAL, DubendorfCurve.TRACK_OVAL_SE2));
  }
}


/* package */ class FakeCurveGeodesicPursuitHelper {
  public Tensor curve = Tensors.empty();

  /** @param pose of vehicle {x[m], y[m], angle}
   * @param speed of vehicle [m*s^-1]
   * @param curve in world coordinates
   * @param isForward driving direction, true when forward or stopped, false when driving backwards
   * @param geodesicInterface type of planned curve
   * @param trajectoryEntryFinder strategy to find best re-entry point
   * @param ratioLimits depending on pose and speed
   * @return ratio rate [m^-1] */
  public Optional<Scalar> getRatio( //
      Tensor pose, Scalar speed, Tensor curve, boolean isForward, //
      GeodesicInterface geodesicInterface, //
      TrajectoryEntryFinder trajectoryEntryFinder, //
      List<DynamicRatioLimit> ratioLimits) {
    TensorUnaryOperator tensorUnaryOperator = new Se2GroupElement(GokartPoseHelper.toUnitless(pose)).inverse()::combine;
    Tensor tensor = Tensor.of(curve.stream().map(tensorUnaryOperator));
    if (!isForward)
      mirrorAndReverse(tensor);
    Predicate<Scalar> isCompliant = isCompliant(ratioLimits, pose, speed);
    TensorScalarFunction mapping = vector -> { //
      if (Scalars.lessThan(Magnitude.METER.apply(GeodesicPursuitParams.GLOBAL.minDistance), Norm._2.ofVector(Extract2D.FUNCTION.apply(vector)))) {
        GeodesicPursuitInterface geodesicPursuit = new GeodesicPursuit(geodesicInterface, vector);
        Tensor ratios = geodesicPursuit.ratios().map(scalar -> Quantity.of(scalar, SI.PER_METER));
        if (ratios.stream().map(Tensor::Get).allMatch(isCompliant))
          return curveLength(geodesicPursuit.curve()); // Norm._2.ofVector(Extract2D.FUNCTION.apply(vector));
      }
      return DoubleScalar.POSITIVE_INFINITY; // TODO GJOEL unitless?
    };
    Scalar var = ArgMinVariable.using(trajectoryEntryFinder, mapping, 25).apply(tensor);
    Optional<Tensor> lookAhead = trajectoryEntryFinder.on(tensor).apply(var).point;
    return lookAhead.map(vector -> {
      GeodesicPursuitInterface geo = new GeodesicPursuit(geodesicInterface, vector);
      TensorUnaryOperator unaryOperator = new Se2GroupElement(GokartPoseHelper.toUnitless(pose))::combine;
      this.curve = Tensor.of(geo.curve().stream().map(unaryOperator));
      return geo.firstRatio().map(r -> Quantity.of(r, SI.PER_METER)).orElse(null);
    });
  }

  /** mirror the points along the y axis and invert their orientation
   * @param se2points curve given by points {x,y,a} */
  private static void mirrorAndReverse(Tensor se2points) {
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
  private static Scalar curveLength(Tensor curve) {
    Tensor curve_ = Tensor.of(curve.stream().map(Extract2D.FUNCTION));
    int n = curve_.length();
    return curve_.extract(1, n).subtract(curve_.extract(0, n - 1)).stream() //
        .map(Norm._2::ofVector) //
        .reduce(Scalar::add).get();
  }
}
