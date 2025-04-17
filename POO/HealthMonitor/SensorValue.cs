using System;
using System.Globalization; // Ensure this is present

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
        get { return this.value; } // Using this. for consistency
        set { this.value = value; }
    }
    public DateTime TimeStamp
    {
        get { return timeStamp; }
        set { timeStamp = value; }
    }


    public string TimeStampString
    {
        get
        {
            // Specify CultureInfo.InvariantCulture for consistent formatting without dots
            return timeStamp.ToString("dd-MMM-yy HH:mm:ss", CultureInfo.InvariantCulture);
        }
        set
        {
            // Keep using InvariantCulture for parsing
            timeStamp = DateTime.ParseExact(value, "dd-MMM-yy HH:mm:ss", CultureInfo.InvariantCulture);
        }
    }
    #endregion // Corrected placement

    #region constructors
    // Corrected default constructor
    public SensorValue()
    {
        this.type = SensorType.None; // Corrected assignment
        this.value = 0.0; // Initialize value
        this.timeStamp = DateTime.MinValue; // Initialize timestamp
    }

    public SensorValue(SensorType type, double value, DateTime timeStamp)
    {
        this.type = type;
        this.value = value;
        this.timeStamp = timeStamp;
    }

    // Constructor now uses the string parameter
    public SensorValue(SensorType type, double value, string timeStampString)
    {
        this.type = type;
        this.value = value;
        // Use the setter logic to parse the string
        this.TimeStampString = timeStampString;
    }
    #endregion

}