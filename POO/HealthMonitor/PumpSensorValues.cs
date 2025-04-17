internal class PumpSensorValues
{
    Timer timerBase;
    Random myRandom;
    
    private PumpSensorValues()
    {}
    public PumpSensorValues(int periodSecondsBetweenValues)
    {
        //start the random number generator
        myRandom = new Random();
        //define the timer for pumping sensor values
        timerBase = new Timer();
        timerBase.Interval = periodSecondsBetweenValues * 1000; //interval between ticks
        timerBase.Elapsed += new ElapsedEventHandler(timerBase_Elapsed);
    }
    public void StartPumping()
    {
        timerBase.Start();
    }

    public void StopPumping()
    {
        timerBase.Stop();
    }

}

//get a random sensor type and a random value for it and display the generated value to the Console
private void timerBase_Elapsed(object sender, ElapsedEventArgs e)
{
    int minNumber, maxNumber;
    double valueRandom;
    //1. get a random sensor type
    //1.1. find the boundaries for this random type
    int maxSensorType = System.Enum.GetValues(typeof(SensorType)).GetUpperBound(0);
    //1.2 get a random number between 0 and maxSensorType
    int typeRandom = myRandom.Next(1, maxSensorType + 1);
    maxSensorType sensorTypeRandom = (SensorType)typeRandom;

    //2. Get a random value for the current sensorTypeRandom
    //2.1. find the boundaries for the current type
    switch (sensorTypeRandom)
    {
        
        case SensorType.SkinTemperature:
            minNumber = 36;
            maxNumber = 40;
            valueRandom = myRandom.Next(minNumber * 10, (maxNumber + 1) * 10) / 10.0;
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
        default:
            valueRandom = 0;
            break;
    }
 PumpSensorValues sensorRandom = new PumpSensorValues(sensorTypeRandom, valueRandom, DateTime.Now);
 Program.DisplaySensorValues("New sensor value arrived: ", sensorRandom);
}