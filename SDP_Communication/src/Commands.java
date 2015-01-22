
public class Commands {
	
	public static final String com0 = "00000000";
	public static final String com1 = "00000001";
	public static final String com2 = "00000010";
	public static final String com3 = "00000011";
	
	public static final String dataStop = "00000000";
	public static final String dataFull = "00000001";
	
	public static String getCommand(String command, String data) {
		return command + data;
	}
	
}
