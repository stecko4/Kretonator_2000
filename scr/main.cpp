#include <Arduino.h>

// Biblioteki potrzebne do przejścia w stan uspienia
//#include <coredecls.h>         // crc32()
//#include <PolledTimeout.h>
//#include <include/WiFiState.h> // WiFiState structure details




#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WString.h>
#include <DNSServer.h>
#include <WebSerial.h>
//const char* ssid = "SSID";
//const char* password = "Password";

const char* ssid = "KRETONATOR"; // WiFi AP SSID
const char* password = ""; // WiFi AP Password

static const char* dict = "AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz1234567890";
static uint32_t last = millis();
static uint32_t count = 0;

AsyncWebServer server(80);
unsigned long ota_progress_millis = 0;



#include <SimpleTimer.h>				          // https://github.com/jfturcot/SimpleTimer
#include <TimeLib.h>
SimpleTimer Timer;


//ZMIENNE GLOBALNE
boolean		 	ReadyToFire			= false;		// TRUE jeśli jest rotary switch poziom 1 / FALSE jeśli jest rotary switch poziom 2
boolean		 	MoleInPlace			= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 	ExplosionConfirmed	= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 	DeadMole			= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 	IgniterReady		= false;		// TRUE jeśli zapalnik jest sprawny (pomiar przewodności) / FALSE jeśli zapalnik jest przerwany (uszkodzony lub po odpaleniu petardy)
boolean		 	BatterySaveMode		= false;		// TRUE jeśli ESP32 jest w trybie oszczędzania baterii, potrzebne do tego aby nie wywoływaćAutoConnect
unsigned long	BatteryLevel		= 0;			// Surowy odczyt poziomu baterii na wejściu analogowym
unsigned long	BatteryPercent		= 0;			// Procent naładowania baterii
volatile int 	timerID				= -1;			//Przetrzymuje ID Timera

//STAŁE
#define CD42ActivateTrigger	  19				// (GPIO19) Pin na którym co 20s wystawiany jest stan LOW aby CD42 nie przechodził w stan uśpienia 
#define MoleDetectionPin	  26				// (GPIO33) Pin na którym jest kontaktron / AM312 PIR Sensor 
#define IgniterVoltage		  33				// (GPIO35, ADC1CH7) Pin pomiaru napięcia zapalnika w celu sprawdzenia czy sprawny
#define RELAY_01			  25				// (GPIO32) Pin do przekaźnika inicjującego wybuch petardy
#define LED_GPIO  			  32     		    // (GPIO32) Pin na którym jest adresowalna dioda RGB 
//#define G_LED			      26				// (GPIO26) Pin na którym jest kolor zielony diody RGB (62 or 56 Ohm resistor for 3.3V)
//#define B_LED			      25				// (GPIO25) Pin na którym jest kolor niebieski diody RGB (62 62Ohm resistor for 3.3V)


//##########################################################################################################
#include <LiteLED.h>							// https://github.com/Xylopyrographer/LiteLED

// Choose the LED type from the list below.
#define LED_TYPE        LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW 0   // if the LED is an RGBW type, change the 0 to 1
#define LED_BRIGHT 40   // sets how bright the LED is. O is off; 255 is burn your eyeballs out (not recommended)
#define TriggerTime 50	// Ustawia czas zwarcia do masy dla zasilania dla wybudzenia 

// pick the colour you want from the list here and change it in setup()
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_WHITE = 0xe0e0e0;
static const crgb_t L_YELLOW = 0xFFFF00;

LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );    // create the LiteLED object; we're calling it "myLED"
//##########################################################################################################




//---------------------------------------------------------------------------------------------------------------------------------------------

// put function declarations here:
void GatherInformation();				// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void BatterySaveMode_Modemsleep();		// Przejście w stan uśpienia (Forced Modem-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Lightsleep();		// Przejście w stan uśpienia (Forced Light-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void PrintData();
void BatteryIndicator();

// Zmienia sekundy na milisekundy
unsigned long secondsToMilliseconds(float seconds) {
    return seconds * 1000;
}

// Zmienia surowy odczyt poziomu baterii na wejściu analogowym na procent naładowania baterii
unsigned long BatteryLevelToPercent(unsigned long BatLev) {
    if (BatLev < 2230)
	{
		BatLev = 2230;
	}
	else if (BatLev > 3462)
	{
		BatLev = 3462;
	}
	return 	map(BatLev, 2230, 3462, 0, 100); 
}

void wakeupCallback() {  // unlike ISRs, you can do a print() from a callback function
#ifdef testPoint
  digitalWrite(testPoint, LOW);  // testPoint tracks latency from WAKE_UP_PIN LOW to testPoint LOW
#endif
  Serial.print(F("millis() = ")); // show that RTC / millis() is stopped in Forced Light Sleep
  Serial.println(millis());  // although the CPU may run for up to 1000 mS before fully stopping
  Serial.println(F("Woke from Forced Light Sleep - this is the callback"));
}

void TriggerCD42() {
	digitalWrite(CD42ActivateTrigger,HIGH);							  // Stan wysoki aby nie zanikało napięcie
	digitalWrite(22,LOW);
	delay(TriggerTime);
	digitalWrite(CD42ActivateTrigger,LOW);							  // Stan wysoki aby nie zanikało napięcie
	digitalWrite(22,HIGH);
	Serial.println("Trigger activated ");
  }

void onOTAStart() {
	// Log when OTA has started
	Serial.println("OTA update started!");
	// <Add your own code here>
  }
  
  void onOTAProgress(size_t current, size_t final) {
	// Log every 1 second
	if (millis() - ota_progress_millis > 1000) {
	  ota_progress_millis = millis();
	  Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
	}
  }
  
  void onOTAEnd(bool success) {
	// Log when OTA has finished
	if (success) {
	  Serial.println("OTA update finished successfully!");
	} else {
	  Serial.println("There was an error during OTA update!");
	}
	// <Add your own code here>
  }


void setup() {
	Serial.begin(115200);
	WiFi.softAP(ssid, password);
	// Once connected, print IP
	Serial.print("IP Address: ");
	Serial.println(WiFi.softAPIP().toString());

	WebSerial.onMessage([](const String& msg) { Serial.println(msg); });
	WebSerial.begin(&server);
  
	server.onNotFound([](AsyncWebServerRequest* request) { request->redirect("/webserial"); });


	ElegantOTA.begin(&server);    // Start ElegantOTA
	// ElegantOTA callbacks
	ElegantOTA.onStart(onOTAStart);
	ElegantOTA.onProgress(onOTAProgress);
	ElegantOTA.onEnd(onOTAEnd);

	server.begin();


	pinMode(MoleDetectionPin, INPUT_PULLUP);				  // Deklaracja pinu z kontaktronem
	//pinMode(LED_GPIO, OUTPUT);					              // Pin na którym jest adresowalna dioda RGB 

	pinMode(RELAY_01, OUTPUT);				                  // Deklaracja pinu z przekaźnikiem wyzwalającym eksplozję petardy
	pinMode(CD42ActivateTrigger, OUTPUT);				      // Deklaracja pinu na którym co 20s wystawiany jest stan LOW aby CD42 nie przechodził w stan uśpienia 
	digitalWrite(CD42ActivateTrigger,LOW);

	pinMode(22, OUTPUT);					          		  // Deklaracja pinu z wbudowaną diodą
	digitalWrite(RELAY_01,LOW);							  // Wyłącza przekaźnik z petardą
	digitalWrite(22,HIGH);							  		  // Wyłącza zintegrowaną diodę LED

    myLED.begin( LED_GPIO, 1 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
    myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level


	//digitalWrite(22, HIGH);		// Zapala diodę
	//delay(secondsToMilliseconds(2)); 


	// call function f once after d milliseconds
    //timerID = Timer.setTimeout(secondsToMilliseconds(30), BatterySaveMode_Modemsleep);	  // Przejdzie w stan uśpienia za 3min czyli 180sec
	//BatterySaveMode_Lightsleep();
	//Timer.setInterval(secondsToMilliseconds(3), PrintData);
	Timer.setInterval(secondsToMilliseconds(25), TriggerCD42);


}

void loop(){	
	Timer.run();

	if ( BatterySaveMode == false )	// Obsługuje Captive portal tylko jeśli nie jest w stanie oszczędzania energii
	{
		// Print every 2 seconds (non-blocking)
		if (millis() - last > 50) {
				count++;
			
				long r = random(10, 250) + 15;
				String buffer;
				buffer.reserve(r);
				buffer += count;
				while (buffer.length() < 10) {
				  buffer += " ";
				}
				buffer += "";
				for (int i = 0; i < r; i++) {
				  buffer += dict[random(0, 62)];
				}
			
			#ifdef WSL_HIGH_PERF
				// Using internal websocket buffer to improve memory consumption and avoid another internal copy when enqueueing the message
				AsyncWebSocketMessageBuffer* wsBuffer = WebSerial.makeBuffer(buffer.length());
				memmove(wsBuffer->get(), buffer.c_str(), buffer.length());
				WebSerial.send(wsBuffer);
			#else
				WebSerial.print(buffer);
			#endif
			
				last = millis();
			  }

		//###################################
		ElegantOTA.loop();
	}

	if ( DeadMole == true)
	{
		// Zapala niebieską diodę na znak żałoby po kreciku
		myLED.setPixel( 0, L_BLUE, 1 );    // set the LED colour and show it	
	}
	else if ( MoleInPlace == false &&  ReadyToFire == true )
	{
		// Zapala czerwoną diodę (Uzbrojone i gotowe do detonacji)
		myLED.setPixel( 0, L_RED, 1 );    // set the LED colour and show it
	}
	else if ( MoleInPlace == false &&  ReadyToFire == false )
	{
		// Zapala zieloną diodę (Prawidłowo uzbrojone)
		myLED.setPixel( 0, L_GREEN, 1 );    // set the LED colour and show it
	}
	else if ( MoleInPlace == true &&  ReadyToFire == false )
	{
		// Zapala żółtą diodę (Popraw wyzwalacz magnetyczny)
		myLED.setPixel( 0, L_YELLOW, 1 );    // set the LED colour and show it
	}
	else if ( MoleInPlace == true &&  ReadyToFire == true )
	{	
		//myLED.setPixel( 0, L_BLUE, 1 );    		// set the LED colour and show it
		digitalWrite(RELAY_01,HIGH);			// Odpala petardę
		delay(secondsToMilliseconds(0.2)); 		// Czeka 0.2s, tyle trwa ikkrzenie zapalnika
		digitalWrite(RELAY_01,LOW);				// Wyłącza przekaźnik z petardą
		DeadMole = true;						// Zmienia status kreta na "MARTWY"
	}
	GatherInformation();

}

// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void GatherInformation() {

	if ( analogRead(IgniterVoltage) < 2230 && BatterySaveMode == false)	// Sprawdza czy na zapalniku jest napięcie aby sprawdzić pozycję Rotary Switch oraz stan zapalnika
	{	
		ReadyToFire = false;
		BatterySaveMode = false;
	}
	else
	{
		ReadyToFire = true;
		if ( BatterySaveMode == false )	// Czy ESP jest już w stanie oszczędzania energii
		{	
			BatteryIndicator();
		}
	}

	if ( digitalRead(MoleDetectionPin) == HIGH )	// Sprawdza stan kontaktrony aby sprawdzić czy jest prawidłowo uzbrojony i czy kred podszedł pod petardę
	{	
		MoleInPlace = true;
	}
	else
	{
		MoleInPlace = false;
	}
}

// REF: optymizacja baterii https://mischianti.org/esp32-practical-power-saving-manage-wifi-and-cpu-1/
// REF: https://github.com/esp8266/Arduino/issues/6642

// Przejście w stan uśpienia (Forced Modem-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Modemsleep()
{
  //WiFi.forceSleepBegin();  // alternate method of Forced Modem Sleep without saving WiFi state
  //WiFi.setSleep(true);

    setCpuFrequencyMhz(80);
	delay(1000);
    Serial.print("CPU Freq: ");
    Serial.println(getCpuFrequencyMhz());
	Serial.println("");

    WiFi.disconnect(true);  		// Disconnect from the network
    WiFi.mode(WIFI_OFF);    		// Switch WiFi off
	delay(10);  					// it doesn't always go to sleep unless you delay(10); yield() wasn't reliable
	BatterySaveMode	= true;			// TRUE jeśli ESP32 jest w trybie oszczędzania baterii, potrzebne do tego aby nie wywoływaćAutoConnect
  	Serial.println("Forced Modem-sleep");
	Serial.println("");
}

// Przejście w stan uśpienia (Forced Light-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Lightsleep()
{
  //WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
  //delay(10);
  //wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  // gpio_pin_wakeup_enable(GPIO_ID_PIN(MoleDetectionPin), GPIO_PIN_INTR_LOLEVEL);
  Serial.println("Forced Light-sleep");
	esp_light_sleep_start();
}

void PrintData()
{
	
	BatteryLevel = analogRead(IgniterVoltage);
	WebSerial.print("BatteryLevel");
	WebSerial.print(BatteryLevel);
	BatteryPercent = BatteryLevelToPercent(BatteryLevel);
	WebSerial.print("BatteryPercent");
	WebSerial.print(BatteryPercent);
/*
	Serial.print("Battery, Analog Read = ");
	Serial.println(BatteryLevel);
	WebSerial.print("Battery, Analog Read = ");
	WebSerial.print(BatteryLevel);

	Serial.print("ReadyToFire = ");
	Serial.println(ReadyToFire);
	WebSerial.print("ReadyToFire = ");
	WebSerial.print(ReadyToFire);

	Serial.print("MoleInPlace = ");
	Serial.println(MoleInPlace);
	WebSerial.print("MoleInPlace = ");
	WebSerial.print(MoleInPlace);

	Serial.print("DeadMole = ");
	Serial.println(DeadMole);
	WebSerial.print("DeadMole = ");
	WebSerial.print(DeadMole);

	Serial.println("");
	WebSerial.print("");
	*/
}

void BatteryIndicator(){

	BatteryLevel = 0;
	for(int j=0; j<5; j++) {
		delay( secondsToMilliseconds(0.05) ); 		// Czeka 0.8s
    	BatteryLevel = BatteryLevel + analogRead(IgniterVoltage);
    }
	BatteryLevel = BatteryLevel / 5; 	//Uśrednianie wartości 
	WebSerial.print(F("BatteryLevel :"));
	WebSerial.println(BatteryLevel);
	BatteryPercent = BatteryLevelToPercent(BatteryLevel);
	WebSerial.print(F("BatteryPercent: "));
	WebSerial.println(BatteryPercent);

	myLED.brightness( 0, 1 );           	// Gasi diodę na początku na 1s aby można było rozrużnić od poprawnego uzbrojenia
	delay( secondsToMilliseconds(1.0) ); 		// Czeka 1s

	for ( int i = 10; i <= 100; i = i + 10 )
	{
		if ( BatteryPercent > i - 5 )	// Poziom naładowania baterii > 20%, pierwsze mignięcie - zielona diona 
		{	
			WebSerial.print(F("i = "));
			WebSerial.println(i);
			myLED.setPixel( 0, L_GREEN, 1 );    			// set the LED colour and show it
			myLED.brightness( LED_BRIGHT, 1 );  		// turn the LED on
			delay( secondsToMilliseconds(0.8) ); 		// Czeka 0.8s
			myLED.brightness( 0, 1 );           		// turn the LED off
			delay( secondsToMilliseconds(0.2) ); 		// Czeka 0.2s
			Timer.run();
		}
		else
		{
			WebSerial.print(F("i = "));
			WebSerial.println(i);
			myLED.setPixel( 0, L_YELLOW, 1 );    			// set the LED colour and show it
			myLED.brightness( LED_BRIGHT, 1 );  		// turn the LED on
			delay( secondsToMilliseconds(0.8) ); 		// Czeka 0.8s
			myLED.brightness( 0, 1 );           		// turn the LED off
			delay( secondsToMilliseconds(0.2) ); 		// Czeka 0.2s
			Timer.run();
		}
	}

	delay( secondsToMilliseconds(1.3) ); 				// Czeka 1.3s
	myLED.setPixel( 0, L_RED, 1 );    					// set the LED colour and show it
	myLED.brightness( LED_BRIGHT, 1 );  				// turn the LED on

	BatterySaveMode_Modemsleep();
	BatterySaveMode = true;
}