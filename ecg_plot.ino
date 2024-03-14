void setup() {
  // initialize the serial communication:
  Serial.begin(115200); //for patient simulator, you can try 9600 instead of 115200 (serial plot looks better)

}

void loop() {
  
  // print the raw analog value (0 - 16383 bits since we've changed the ADC to 14 bit resolution) 
  //which is used by serial plotter (or view data stream on serial monitor)
  Serial.println(analogRead(A0));
  
  //Wait for a bit to keep serial data from saturating
  delay(5);
}
