#define ECG_PIN A0 //each ECG kit will have the analog ECG output connected to pin A0 on the Arduino UNO Development Board
#define RR_BUFFER_SIZE 100 //the number of R-R intervals (ms) to collect for the HRV metrics (more data = more accurate HRV = longer collection time)

//defining state numbers for the finite state machine
#define INIT 0
#define R_WAVE_DETECTED 1
#define S_WAVE_DETECTED 2

int state = INIT; //state variable, initialize it with the INIT state

unsigned long previousRWaveTime = 0;
unsigned long currentRWaveTime = 0;
unsigned long timestamp = 0;
unsigned long rrIntervals[RR_BUFFER_SIZE]; //u long array to hold all r-r interval times (ms)
int rrIndex = 0;

const int rWaveThreshold = 410; // Adjust based on your personal ecg signal (use ecg_plot.ino to determine)
const int sWaveThreshold = 310;
const int ecgSignalOffset = 200;
unsigned long rrInterval = 0;

const int windowSize = 3; //this rolling window size needs to be 3 for determining local maxima
int dataWindow[windowSize] = {0, 0, 0}; //rolling data window initialized to 0s
int timestampWindow[windowSize] = {0,0,0}; //this size 3 array holds the timestamps of corresponding data in dataWindow

bool localMaxima = false; 
int currentEcgMagnitude = 0;

int rawSensorValue = 0;

float heartrate = 0.0;
float sdnn = 0.0;
float rmssd = 0.0;

void setup() {
  //serial baud rate must be 115200
  Serial.begin(115200); //this is the baud rate we need to set the Serial Monitor and Serial Plotter to
  //analogReadResolution(14); //set the analog read or ADC resolution to 14 bit for higher resolution data acquisiton
  pinMode(LED_BUILTIN, OUTPUT); //we're going to use the built in LED to indicate R Waves (LED will be on when an R wave is detected)
}

void loop() {
  // Read analog input of ECG connected to pin A0, store in variable called rawSensorValue
  rawSensorValue = analogRead(ECG_PIN);
  timestamp = millis(); //temporary variable to hold the timestamp of the data point in ms
  updateWindow(rawSensorValue,timestamp);  //update sliding window (shift and add new data acquired)

  localMaxima = (dataWindow[0] < dataWindow[1]) && (dataWindow[2] < dataWindow[1]); //boolean expression to check if there was a local maxima
  currentEcgMagnitude = dataWindow[1]; //we look at the second latest data value so we can detect local maxima
  
  // If statement to check if there's a local peak by comparing the first and last values in the sliding window with the second value (i.e. the second value in the window is a peak / local maximum)
  // AND that the peak is above the threshold that constitutes a R wave (determined by viewing the ECG wave with ecg_plot.ino)
  // AND that the state is not R_WAVE_DETECTED, meaning we are in the S_WAVE_DETECTED state and waiting for an R Wave
  // i.e. we are checking IF we have found an R wave peak
  if (localMaxima && (currentEcgMagnitude > rWaveThreshold) && (state != R_WAVE_DETECTED)) {
   
    //check if the state is the initialization state (INIT)
    if (state == INIT) {
      // If the state is INIT then this is the first R wave since boot, so we need to set the first R wave timestamp
      previousRWaveTime = timestampWindow[1];
      digitalWrite(LED_BUILTIN, HIGH); //turn the built in LED on so that it's glowing when an R wave is detected
    }
    else {
      //or else the state is not INIT so we update the rrIntervals array and continue the algorithm
      
      currentRWaveTime = timestampWindow[1]; //store the R wave peak timestamp
      rrInterval = (currentRWaveTime - previousRWaveTime); //calculate the difference between R wave peak timestamps i.e. the R-R interval
      rrIntervals[rrIndex] = rrInterval; //store the R-R interval time in the array
      rrIndex = (rrIndex + 1) % RR_BUFFER_SIZE; //update the index so the next rrInterval will be stored in the next index, which effectively makes the array a circular buffer

      previousRWaveTime = currentRWaveTime; //store the timestamp for the future to calculate the next difference when a new peak is found
      
      digitalWrite(LED_BUILTIN, HIGH); //turn the built in LED on so that it's glowing when an R wave is detected

      //if we have a new full set of data, update the HRV metrics
      if (rrIndex == 0) {
        sdnn = calculateSDNN(rrIntervals, RR_BUFFER_SIZE); //call our function to calculate the SDRR HRV metric
        rmssd = (float)calculateRMSSD(rrIntervals, RR_BUFFER_SIZE);  //call our function to calculate the RMSSD HRV metric
        
      }
      //delay(1);
    }
    state = R_WAVE_DETECTED; //we need to update the state variable since an R wave was detected, this is used in the algorithm so that consecutive peaks aren't considered until an S Wave is detected
  }

  // If statement to check if the ecg signal goes under the threshold that determines the R wave is transitioning to the S Wave
  // AND that the current state is R_WAVE_DETECTED so that we already had a R wave peak
  if (currentEcgMagnitude < sWaveThreshold && state == R_WAVE_DETECTED){
    state = S_WAVE_DETECTED; //need to update the state variable since the S wave was detected and we need to start checking for an R wave peak again
    digitalWrite(LED_BUILTIN, LOW); //turn off the built in LED so that it's only on when a R wave is detected
  }
  
  if (rrInterval != 0){
    heartrate = (1.0/rrInterval)*60.0*1000.0;
  }
  
  //before printing all the values, we're going to map the ECG sensor values from 14 bit to 8 bit resolution
  //mapping the ECG values will make the Serial Plotter graph more legible since the Y axis will be a smaller range
  
  rawSensorValue= map(rawSensorValue,0,16383,0,255); //You should use ecg_plot.ino for determining thresholds, but if you want to use this code instead, remove this line!!
  Serial.print("ECG:");
  Serial.print(rawSensorValue);
  Serial.print(",");
  Serial.print("HR(bpm):");
  Serial.print(heartrate);
  Serial.print(",");
  Serial.print("SDRR(ms):");
  Serial.print(sdnn);
  Serial.print(",");
  Serial.print("RMSSD(ms):");
  Serial.println(rmssd);
  delay(1);
}

//function for shifting the sliding window by 1 index and storing the latest sensor value in the last index
//i.e. the data shifts one index to the left and the current data point is stored in the final index
// dataWindow structure is: [2ndLastSensorValue lastSensorValue latestSensorValue]
void updateWindow(int value, int ms) {
  dataWindow[0] = dataWindow[1];
  timestampWindow[0] = timestampWindow[1];
  
  dataWindow[1] = dataWindow[2];
  timestampWindow[1] = timestampWindow[2];
  
  dataWindow[2] = value;
  timestampWindow[2] = ms;
}

//function which calculates Standard Deviation of R-R intervals (SDRR) HRV Metric
//the parameters passed are the array holding the R-R intervals (ms) and the size of the rrIntervals array
double calculateSDNN(unsigned long* rrIntervals, int numberOfIntervals) {
  // Calculate the mean RR interval
  unsigned long sum = 0;
  for (int i = 0; i < numberOfIntervals; i++) {
    sum += rrIntervals[i];
  }
  double meanRR = sum / numberOfIntervals;

  // Calculate the sum of squared differences from the mean
  double sumSquaredDifferences = 0;
  for (int i = 0; i < numberOfIntervals; i++) {
    double difference = rrIntervals[i] - meanRR;
    sumSquaredDifferences += difference * difference;
  }

  // Calculate the standard deviation to determine the Standard Deviation of R-R intervals (SDRR) HRV Metric
  double calculatedSDNN = sqrt(sumSquaredDifferences / numberOfIntervals - 1);
  return calculatedSDNN;
}

//function which calculates Root Mean Square of Successive Differences of R-R intervals (RMSSD) HRV Metric
//the parameters passed are the array holding the R-R intervals (ms) and the size of the rrIntervals array
double calculateRMSSD(unsigned long* nnIntervals, int numberOfIntervals) {

  double sumSquaredDifferences = 0.0;

  for (int i = 1; i < numberOfIntervals; i++) {
    // Using conditional expression to handle potential negative values which would underflow the unsigned long timestamps which came from using millis()
    unsigned long difference = (nnIntervals[i] >= nnIntervals[i - 1]) ?
                         (nnIntervals[i] - nnIntervals[i - 1]) :
                         (nnIntervals[i - 1] - nnIntervals[i]);
    double squaredDifference = sq(difference);
    sumSquaredDifferences += squaredDifference;
  }

  // Calculate the square root of the mean square difference in R-R intervals to determine the Root Mean Square Successive Difference of R-R intervals (RMSSD) HRV Metric
  double calculatedRMSSD = sqrt(sumSquaredDifferences / (double)(numberOfIntervals - 1));
  return calculatedRMSSD;
}
