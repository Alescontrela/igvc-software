/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.24
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class player_blinkenlight_cmd_power_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected player_blinkenlight_cmd_power_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(player_blinkenlight_cmd_power_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_player_blinkenlight_cmd_power_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  protected static long[] cArrayUnwrap(player_blinkenlight_cmd_power_t[] arrayWrapper) {
      long[] cArray = new long[arrayWrapper.length];
      for (int i=0; i<arrayWrapper.length; i++)
        cArray[i] = player_blinkenlight_cmd_power_t.getCPtr(arrayWrapper[i]);
      return cArray;
  }

  protected static player_blinkenlight_cmd_power_t[] cArrayWrap(long[] cArray, boolean cMemoryOwn) {
    player_blinkenlight_cmd_power_t[] arrayWrapper = new player_blinkenlight_cmd_power_t[cArray.length];
    for (int i=0; i<cArray.length; i++)
      arrayWrapper[i] = new player_blinkenlight_cmd_power_t(cArray[i], cMemoryOwn);
    return arrayWrapper;
  }

  public void setEnable(short enable) {
    playercore_javaJNI.set_player_blinkenlight_cmd_power_t_enable(swigCPtr, enable);
  }

  public short getEnable() {
    return playercore_javaJNI.get_player_blinkenlight_cmd_power_t_enable(swigCPtr);
  }

  public player_blinkenlight_cmd_power_t() {
    this(playercore_javaJNI.new_player_blinkenlight_cmd_power_t(), true);
  }

}