// code by mcp
package ch.ethz.idsc.owl.bot.se2.pid;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.ani.adapter.StateTrajectoryControl;
import ch.ethz.idsc.owl.bot.se2.Se2Wrap;
import ch.ethz.idsc.owl.bot.se2.glc.CarHelper;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.red.Norm2Squared;
import ch.ethz.idsc.tensor.sca.Clip;
import ch.ethz.idsc.tensor.sca.Clips;

/** PID control */
public class PIDTrajectoryControl extends StateTrajectoryControl {
  private final Clip clip;
  private final PIDGains pidGains;
  private PIDTrajectory pidTrajectory;
  private int pidIndex;

  public PIDTrajectoryControl(Scalar maxTurningRate, PIDGains pidGains) {
    this.clip = Clips.interval(maxTurningRate.negate(), maxTurningRate);
    this.pidGains = pidGains;
  }

  @Override // from StateTrajectoryControl
  protected Scalar pseudoDistance(Tensor x, Tensor y) {
    return Norm2Squared.ofVector(Se2Wrap.INSTANCE.difference(x, y));
  }

  @Override // from StateTrajectoryControl
  protected Optional<Tensor> customControl(StateTime stateTime, List<TrajectorySample> trailAhead) {
    Scalar speed = trailAhead.get(0).getFlow().get().getU().Get(0);
    Tensor traj = Tensor.of(trailAhead.stream() //
        .map(TrajectorySample::stateTime) //
        .map(StateTime::state));
    StateTime unitST = new StateTime(stateTime.state(), Quantity.of(stateTime.time(), SI.SECOND));
    PIDTrajectory pidTrajectory = new PIDTrajectory(pidIndex, this.pidTrajectory, pidGains, traj, unitST);
    Scalar ratePerMeter = pidTrajectory.ratioOut();
    if (clip.isInside(ratePerMeter))
      return Optional.of(CarHelper.singleton(speed, ratePerMeter).getU());
    this.pidTrajectory = pidTrajectory;
    ++pidIndex;
    return Optional.empty();
  }
}
