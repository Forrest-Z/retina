// code by jph
package ch.ethz.idsc.retina.dev.zhkart;

import ch.ethz.idsc.retina.dev.linmot.LinmotSocket;
import ch.ethz.idsc.retina.dev.misc.MiscSocket;
import ch.ethz.idsc.retina.dev.rimo.RimoSocket;
import ch.ethz.idsc.retina.dev.steer.SteerSocket;
import ch.ethz.idsc.retina.sys.AbstractModule;

public class AutoboxSocketModule extends AbstractModule {
  @Override
  protected void first() throws Exception {
    RimoSocket.INSTANCE.start();
    LinmotSocket.INSTANCE.start();
    SteerSocket.INSTANCE.start();
    MiscSocket.INSTANCE.start();
  }

  @Override
  protected void last() {
    RimoSocket.INSTANCE.stop();
    LinmotSocket.INSTANCE.stop();
    SteerSocket.INSTANCE.stop();
    MiscSocket.INSTANCE.stop();
  }
}
