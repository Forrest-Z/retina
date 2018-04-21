// code by jph
package ch.ethz.idsc.retina.dev.linmot;

import java.nio.ByteBuffer;

import ch.ethz.idsc.gokart.core.DataEvent;
import ch.ethz.idsc.retina.util.data.Word;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** information sent to micro-autobox to forward to the linear motor that
 * controls the break of the gokart */
public class LinmotPutEvent extends DataEvent {
  /** 12 bytes encoding length */
  private static final int LENGTH = 12;

  /** function generates messages for calibration and linmot de-activation.
   * the values that determine position control are all set to zero.
   * 
   * @param control
   * @param motion command header
   * @return */
  /* package */ static LinmotPutEvent configuration(Word control, Word motion) {
    return new LinmotPutEvent(control, motion.getShort(), (short) 0, (short) 0, (short) 0, (short) 0);
  }

  // ---
  public final short control_word;
  public final short motion_cmd_hdr;
  public final short target_position;
  public final short max_velocity;
  public final short acceleration;
  public final short deceleration;

  /** universal constructor of messages for linmot.
   * not all parameter combinations make sense.
   * the flexibility is required for testing.
   * 
   * TODO remark on interpretation
   * 
   * @param control
   * @param motion command header
   * @param target_position
   * @param max_velocity
   * @param acceleration
   * @param deceleration */
  public LinmotPutEvent(Word control, short motion_cmd_hdr, //
      short target_position, short max_velocity, short acceleration, short deceleration) {
    control_word = control.getShort();
    this.motion_cmd_hdr = motion_cmd_hdr;
    this.target_position = target_position;
    this.max_velocity = max_velocity;
    this.acceleration = acceleration;
    this.deceleration = deceleration;
  }

  public LinmotPutEvent(ByteBuffer byteBuffer) {
    control_word = byteBuffer.getShort();
    motion_cmd_hdr = byteBuffer.getShort();
    target_position = byteBuffer.getShort();
    max_velocity = byteBuffer.getShort();
    acceleration = byteBuffer.getShort();
    deceleration = byteBuffer.getShort();
  }

  /** @param byteBuffer with at least 12 bytes remaining */
  @Override // from DataEvent
  public void insert(ByteBuffer byteBuffer) {
    byteBuffer.putShort(control_word);
    byteBuffer.putShort(motion_cmd_hdr);
    byteBuffer.putShort(target_position);
    byteBuffer.putShort(max_velocity);
    byteBuffer.putShort(acceleration);
    byteBuffer.putShort(deceleration);
  }

  @Override // from DataEvent
  protected int length() {
    return LENGTH;
  }

  public boolean isOperational() {
    return control_word == LinmotPutHelper.CMD_OPERATION.getShort() //
        && motion_cmd_hdr == LinmotPutHelper.MC_POSITION.getShort();
  }

  /** function only used in post-processing
   * 
   * @return */
  public Tensor vector_raw() {
    return Tensors.vector(control_word, motion_cmd_hdr, target_position, max_velocity, acceleration, deceleration);
  }

  public String toInfoString() {
    return String.format("%d %d %d %d %d %d", //
        control_word, motion_cmd_hdr, //
        target_position, max_velocity, //
        acceleration, deceleration);
  }
}
