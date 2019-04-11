// code by jph
package ch.ethz.idsc.gokart.core.pursuit;

import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.gokart.core.pursuit.pure.CurvePurePursuitModule;
import ch.ethz.idsc.gokart.gui.top.GlobalViewLcmModule;
import ch.ethz.idsc.retina.util.sys.AbstractModule;
import ch.ethz.idsc.retina.util.sys.ModuleAuto;
import ch.ethz.idsc.tensor.Tensor;

abstract class FigureBaseModule extends AbstractModule {
  private final CurvePursuitModule curvePursuitModule = new CurvePurePursuitModule(PursuitConfig.GLOBAL);
  private final GlobalViewLcmModule globalViewLcmModule = ModuleAuto.INSTANCE.getInstance(GlobalViewLcmModule.class);

  protected FigureBaseModule(Tensor curve) {
    curvePursuitModule.setCurve(Optional.of(curve));
    if (Objects.nonNull(globalViewLcmModule))
      globalViewLcmModule.setCurve(curve);
  }

  @Override // from AbstractModule
  protected final void first() {
    curvePursuitModule.launch();
  }

  @Override // from AbstractModule
  protected final void last() {
    curvePursuitModule.terminate();
    if (Objects.nonNull(globalViewLcmModule))
      globalViewLcmModule.setCurve(null);
  }
}
