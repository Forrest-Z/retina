// code by jph
package ch.ethz.idsc.gokart.core.pursuit;

import ch.ethz.idsc.gokart.core.pos.GokartPoseEvent;
import ch.ethz.idsc.gokart.core.pos.GokartPoseLcmClient;
import ch.ethz.idsc.gokart.core.pos.GokartPoseListener;
import ch.ethz.idsc.gokart.dev.rimo.RimoConfig;
import ch.ethz.idsc.gokart.dev.rimo.RimoGetEvent;
import ch.ethz.idsc.gokart.dev.rimo.RimoGetListener;
import ch.ethz.idsc.gokart.dev.rimo.RimoSocket;
import ch.ethz.idsc.gokart.gui.top.ChassisGeometry;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Sign;

import java.util.Objects;
import java.util.Optional;

/** class is the default choice for pursuit when driving along a curve in global
 * coordinates while the pose is updated periodically from a localization method. */
public abstract class CurvePursuitModule extends PursuitModule implements GokartPoseListener {
  private final Chop speedChop = RimoConfig.GLOBAL.speedChop();
  private final GokartPoseLcmClient gokartPoseLcmClient = new GokartPoseLcmClient();
  /** forward motion is determined by odometry:
   * noise in the measurements around zero are also mapped to "forward" */
  protected boolean isForward = true;
  protected Scalar speed = RealScalar.ZERO;
  public final RimoGetListener rimoGetListener = new RimoGetListener() {
    @Override
    public void getEvent(RimoGetEvent rimoGetEvent) {
      speed = speedChop.apply(ChassisGeometry.GLOBAL.odometryTangentSpeed(rimoGetEvent));
      isForward = Sign.isPositiveOrZero(speed);
    }
  };
  // ---
  protected Optional<Tensor> optionalCurve = Optional.empty();
  protected boolean closed = true;
  protected GokartPoseEvent gokartPoseEvent = null;

  public CurvePursuitModule(PursuitConfig pursuitConfig) {
    super(pursuitConfig);
  }

  @Override // from AbstractModule
  protected final void protected_first() {
    gokartPoseLcmClient.addListener(this);
    gokartPoseLcmClient.startSubscriptions();
    RimoSocket.INSTANCE.addGetListener(rimoGetListener);
  }

  @Override // from AbstractModule
  protected final void protected_last() {
    RimoSocket.INSTANCE.removeGetListener(rimoGetListener);
    gokartPoseLcmClient.stopSubscriptions();
  }

  @Override // from PursuitModule
  protected final Optional<Scalar> deriveHeading() {
    GokartPoseEvent gokartPoseEvent = this.gokartPoseEvent; // copy reference instead of synchronize
    // System.err.println("check isOperational");
    if (Objects.nonNull(gokartPoseEvent)) { // is localization pose available?
      final Scalar quality = gokartPoseEvent.getQuality();
      if (pursuitConfig.isQualitySufficient(quality)) { // is localization quality sufficient?
        Tensor pose = gokartPoseEvent.getPose(); // latest pose
        Optional<Scalar> ratio = getRatio(pose);
        if (ratio.isPresent()) { // is look ahead beacon available?
          Scalar angle = ChassisGeometry.GLOBAL.steerAngleForTurningRatio(ratio.get());
          if (angleClip.isInside(angle)) // is look ahead beacon within steering range?
            return Optional.of(angle);
          System.err.println("beacon outside steering range");
        }
      } else
        System.err.println("pose quality insufficient");
    }
    return Optional.empty(); // autonomous operation denied
  }

  // TODO JPH function should return a scalar with unit "rad*m^-1"...
  // right now, "curve" does not have "m" as unit but entries are unitless.
  /** @param pose
   * @return */
  protected abstract Optional<Scalar> getRatio(Tensor pose);

  /** @param curve world frame coordinates */
  public synchronized final void setCurve(Optional<Tensor> curve) {
    setCurve(curve, true);
  }

  public synchronized final void setCurve(Optional<Tensor> curve, boolean closed) {
    optionalCurve = curve;
    this.closed = closed;
  }

  @Override // from GokartPoseListener
  public final void getEvent(GokartPoseEvent gokartPoseEvent) { // arrives at 20[Hz]
    this.gokartPoseEvent = gokartPoseEvent;
  }

  /***************************************************/
  /** @return curve world frame coordinates */
  public final Optional<Tensor> getCurve() {
    return optionalCurve;
  }

  /** @return true if gokart is stationary or moving forwards */
  public final boolean isForward() {
    return isForward;
  }
}
