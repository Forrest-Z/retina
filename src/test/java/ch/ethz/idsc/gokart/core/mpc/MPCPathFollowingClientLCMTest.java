// code by mh
package ch.ethz.idsc.gokart.core.mpc;

import junit.framework.TestCase;

public class MPCPathFollowingClientLCMTest extends TestCase {
  public void testSimple() throws Exception {
    // only sends a simple message
    // uncomment if you are able to compile the binary
    /* LcmMPCPathFollowingClient lcmMPCPathFollowingClient = new LcmMPCPathFollowingClient();
     * try {
     * // TODO jan had to add try here
     * lcmMPCPathFollowingClient.start();
     * for (int i = 0; i < 4; i++) {
     * System.out.println("i=" + i);
     * GokartState gokartState = new GokartState(//
     * 11, //
     * 12, //
     * 13, //
     * 14, //
     * 15, //
     * 16, //
     * 17, //
     * 18, //
     * 19, //
     * 20);
     * lcmMPCPathFollowingClient.publishGokartState(gokartState);
     * Thread.sleep(1000);
     * // System.out.print(lcmMPCPathFollowingClient.mpcNativeSession.getNativeOutput());
     * }
     * lcmMPCPathFollowingClient.stop();
     * } catch (Exception exception) {
     * exception.printStackTrace();
     * } */
  }
}
