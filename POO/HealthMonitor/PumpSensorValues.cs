using System; // For Random, DateTime, Enum
using System.Timers; // For Timer, ElapsedEventArgs

// Assuming SensorValue and SensorType are in the same namespace or referenced
// namespace HealthMonitor; // If using a namespace

public class PumpSensorValues // Changed to public for broader accessibility
{
    System.Timers.Timer timerBase;
    Random myRandom;

    // Private constructor might be intended for singleton pattern, but seems unused here.
    // private PumpSensorValues() {}

    public PumpSensorValues(int periodSecondsBetweenValues)
    {
        // Start the random number generator
        myRandom = new Random();
        // Define the timer for pumping sensor values
        timerBase = new System.Timers.Timer();
        timerBase.Interval = periodSecondsBetweenValues * 1000; // Interval between ticks
        timerBase.Elapsed += new ElapsedEventHandler(timerBase_Elapsed); // Correctly references the method within the class
    }

    public void StartPumping()
    {
        Console.WriteLine("Starting sensor value generation..."); // Added feedback
        timerBase.Start();
    }

    public void StopPumping()
    {
        Console.WriteLine("Stopping sensor value generation..."); // Added feedback
        timerBase.Stop();
    }

    // Moved timerBase_Elapsed method INSIDE the PumpSensorValues class
    private void timerBase_Elapsed(object? sender, ElapsedEventArgs e)
    {
        int minNumber, maxNumber;
        double valueRandom;

        // 1. Get a random sensor type (excluding None)
        // 1.1. Find the boundaries for this random type
        // Get all values as SensorType[]
        SensorType[] sensorTypeValues = (SensorType[])Enum.GetValues(typeof(SensorType));
        int maxSensorIndex = sensorTypeValues.Length - 1; // Get the count of defined enum values

        // 1.2 Get a random number between 1 (skip None) and maxSensorIndex
        // Note: Enum.GetValues includes the underlying integer values.
        // If SensorType starts at 0 (None), index 0 is None.
        int typeRandomIndex = myRandom.Next(1, maxSensorIndex + 1); // Assuming None is 0 and we skip it
        SensorType sensorTypeRandom = sensorTypeValues[typeRandomIndex]; // Access directly by index

        // 2. Get a random value for the current sensorTypeRandom
        // 2.1. Find the boundaries for the current type
        switch (sensorTypeRandom)
        {
            case SensorType.SkinTemperature:
                minNumber = 36;
                maxNumber = 40;
                // Generate value with one decimal place
                valueRandom = myRandom.Next(minNumber * 10, maxNumber * 10 + 1) / 10.0;
                break;
            case SensorType.BloodGlucose:
                minNumber = 80;
                maxNumber = 300;
                valueRandom = myRandom.Next(minNumber, maxNumber + 1);
                break;
            case SensorType.HeartRate:
                minNumber = 30;
                maxNumber = 200;
                valueRandom = myRandom.Next(minNumber, maxNumber + 1);
                break;
            // Add cases for any other SensorType values if necessary
            default:
                // Handle unexpected sensor types or SensorType.None if it could be generated
                valueRandom = 0;
                Console.WriteLine($"Warning: Generated default value for unexpected SensorType: {sensorTypeRandom}");
                break;
        }

        // 3. Create a SensorValue object (NOT PumpSensorValues)
        SensorValue sensorRandom = new SensorValue(sensorTypeRandom, valueRandom, DateTime.Now);

        // 4. Display the generated value using the static method from Program
        // Ensure Program.DisplaySensorValues is public static or internal static
        Program.DisplaySensorValues("New sensor value arrived: ", sensorRandom);
    }
}