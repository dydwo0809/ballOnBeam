#define PIN_LED 7
unsigned int count, toggle, answer;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  if (count <= 10)
    answer = 0;
  else if (count <= 20)
    answer = count % 2;
  else
    answer = 1;
  toggle = toggle_state(toggle); // toggle LED value.
  digitalWrite(PIN_LED, toggle); //update LED status.
  delay(100); // wait for 200 milliseconds
}

int toggle_state(int toggle) {
 
  return answer;
}
