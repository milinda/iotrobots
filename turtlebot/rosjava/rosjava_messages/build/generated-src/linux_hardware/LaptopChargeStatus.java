package linux_hardware;

public interface LaptopChargeStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "linux_hardware/LaptopChargeStatus";
  static final java.lang.String _DEFINITION = "uint8 DISCHARGING = 0\nuint8 CHARGING    = 1\nuint8 CHARGED     = 2\n\nHeader  header\nfloat32 voltage          # Voltage in Volts\nfloat32 rate             # Negative when discharging (A)\nfloat32 charge           # Current charge in Ah\nfloat32 capacity         # Capacity in Ah (last full capacity)\nfloat32 design_capacity  # Capacity in Ah (design capacity)\nint32   percentage       # Charge percentage\nuint8   charge_state     # Enum \nbool    present          # Should be an error if battery is not present";
  static final byte DISCHARGING = 0;
  static final byte CHARGING = 1;
  static final byte CHARGED = 2;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getVoltage();
  void setVoltage(float value);
  float getRate();
  void setRate(float value);
  float getCharge();
  void setCharge(float value);
  float getCapacity();
  void setCapacity(float value);
  float getDesignCapacity();
  void setDesignCapacity(float value);
  int getPercentage();
  void setPercentage(int value);
  byte getChargeState();
  void setChargeState(byte value);
  boolean getPresent();
  void setPresent(boolean value);
}
