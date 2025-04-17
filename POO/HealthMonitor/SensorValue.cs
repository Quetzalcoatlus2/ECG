public enum SensorType
{
    None,
    SkinTemperature,
    HeartRate,
    BloodGlucose    
}

public class SensorValue
{
    private SensorType type;
    private double value;
    private DateTime timeStamp;

    #region properties
    public SensorType Type
    {
        get { return type; }
        set { type = value; }
    }
    public double Value
    {
        get { return value; }
        set { this.value = value; }
    }
    public DateTime TimeStamp
    {
        get { return timeStamp; }
        set { timeStamp = value; }
    }
 

    public string TimeStampString
    {
        get { return timeStamp.ToString("dd-MMM-yy HH:mm:ss"); }
        set { timeStamp = DateTime.ParseExact(value, "dd-MMM-yy HH:mm:ss", CultureInfo.InvariantCulture); }
    }

    #endregion

    #region constructors
    public SensorValue()
    {
        typeof = SensorType.None;
    }
    public SensorValue(SensorType type, double value, DateTime timeStamp)
    {
        this.type = type;
        this.value = value;
        this.timeStamp = timeStamp;
    }  

    public SensorValue(SensorType type, double value, string timeStamp)
    {
        this.type = type;
        this.value = value;
        this.timeStamp = DateTime.Now;
    }
    #endregion
}