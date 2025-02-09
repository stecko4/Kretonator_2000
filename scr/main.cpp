#include <Arduino.h>

// Biblioteki potrzebne do przejścia w stan uspienia
//#include <coredecls.h>         // crc32()
//#include <PolledTimeout.h>
//#include <include/WiFiState.h> // WiFiState structure details




#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
const char* ssid = "SSID";
const char* password = "Password";
AsyncWebServer server(80);
unsigned long ota_progress_millis = 0;










#include <SimpleTimer.h>				          // https://github.com/jfturcot/SimpleTimer
#include <TimeLib.h>
SimpleTimer Timer;


//##########################################################################################################
#include <LiteLED.h>	// https://github.com/Xylopyrographer/LiteLED

// Choose the LED type from the list below.
// Comment out all but one LED_TYPE.
#define LED_TYPE        LED_STRIP_WS2812
// #define LED_TYPE        LED_STRIP_SK6812
// #define LED_TYPE        LED_STRIP_APA106
// #define LED_TYPE        LED_STRIP_SM16703

#define LED_TYPE_IS_RGBW 0   	// if the LED is an RGBW type, change the 0 to 1

#define LED_BRIGHT 30   		// sets how bright the LED is. O is off; 255 is burn your eyeballs out (not recommended)

// pick the colour you want from the list here and change it in setup()
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_YELLOW = 0xFFFF00;

LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );    // create the LiteLED object; we're calling it "myLED"
//##########################################################################################################





//ZMIENNE GLOBALNE
boolean		 ReadyToFire		= false;		// TRUE jeśli jest rotary switch poziom 1 / FALSE jeśli jest rotary switch poziom 2
boolean		 MoleInPlace		= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 ExplosionConfirmed	= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 DeadMole			= false;		// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 IgniterReady		= false;		// TRUE jeśli zapalnik jest sprawny (pomiar przewodności) / FALSE jeśli zapalnik jest przerwany (uszkodzony lub po odpaleniu petardy)
boolean		 BatterySaveMode	= false;		// TRUE jeśli ESP32 jest w trybie oszczędzania baterii, potrzebne do tego aby nie wywoływaćAutoConnect
volatile int timerID			= -1;			//Przetrzymuje ID Timera

//STAŁE
#define CD42ActivateTrigger	  19				// (GPIO19) Pin na którym co 20s wystawiany jest stan LOW aby CD42 nie przechodził w stan uśpienia 
#define MoleDetectionPin	  26				// (GPIO33) Pin na którym jest kontaktron / AM312 PIR Sensor 
#define IgniterVoltage		  A0				// (GPIO35, ADC1CH7) Pin pomiaru napięcia zapalnika w celu sprawdzenia czy sprawny
#define RELAY_01			  32				// (GPIO32) Pin do przekaźnika inicjującego wybuch petardy
#define LED_GPIO 			  27     		    // (GPIO27) Pin na którym jest adresowalna dioda RGB 
//#define G_LED			      26				// (GPIO26) Pin na którym jest kolor zielony diody RGB (62 or 56 Ohm resistor for 3.3V)
//#define B_LED			      25				// (GPIO25) Pin na którym jest kolor niebieski diody RGB (62 62Ohm resistor for 3.3V)

//---------------------------------------------------------------------------------------------------------------------------------------------

// put function declarations here:
void GatherInformation();				// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void BatterySaveMode_Modemsleep();		// Przejście w stan uśpienia (Forced Modem-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Lightsleep();		// Przejście w stan uśpienia (Forced Light-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void PrintData();

// Zmienia sekundy na milisekundy
unsigned long secondsToMilliseconds(unsigned long seconds) {
    return seconds * 1000;
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
	digitalWrite(CD42ActivateTrigger,LOW);							  // Stan wysoki aby nie zanikało napięcie
	digitalWrite(22,LOW);
	delay(10);
	digitalWrite(CD42ActivateTrigger,HIGH);							  // Stan wysoki aby nie zanikało napięcie
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

	WiFi.mode(WIFI_STA);
	WiFi.begin("ECN", "Pecherek1987");
	Serial.println("");
  
	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
	  delay(500);
	  Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
	  request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
	});
  

	// Set Authentication Credentials
	//ElegantOTA.setAuth("myUsername", "myPassword");

	ElegantOTA.begin(&server);    // Start ElegantOTA
	// ElegantOTA callbacks
	ElegantOTA.onStart(onOTAStart);
	ElegantOTA.onProgress(onOTAProgress);
	ElegantOTA.onEnd(onOTAEnd);
  
	server.begin();
	Serial.println("HTTP server started");

	pinMode(MoleDetectionPin, INPUT_PULLUP);				  // Deklaracja pinu z kontaktronem
	pinMode(LED_GPIO, OUTPUT);					              // Pin na którym jest adresowalna dioda RGB 

	pinMode(RELAY_01, OUTPUT);				                  // Deklaracja pinu z przekaźnikiem wyzwalającym eksplozję petardy
	pinMode(CD42ActivateTrigger, OUTPUT);				      // Deklaracja pinu na którym co 20s wystawiany jest stan LOW aby CD42 nie przechodził w stan uśpienia 
	

	pinMode(22, OUTPUT);					          		  // Deklaracja pinu z wbudowaną diodą
	digitalWrite(RELAY_01,HIGH);							  // Wyłącza przekaźnik z petardą
	digitalWrite(22,HIGH);							  		  // Wyłącza zintegrowaną diodę LED

	myLED.begin( LED_GPIO, 1 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
    myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level
    myLED.setPixel( 0, L_GREEN, 1 );    // set the LED colour and show it
	delay(secondsToMilliseconds(1)); 
	myLED.setPixel( 0, L_YELLOW, 1 );    // set the LED colour and show it
	delay(secondsToMilliseconds(1));
	myLED.setPixel( 0, L_RED, 1 );    // set the LED colour and show it
	delay(secondsToMilliseconds(1));
	myLED.setPixel( 0, L_BLUE, 1 );    // set the LED colour and show it
	delay(secondsToMilliseconds(1));
	//myLED.brightness( 0, 1 );           // turn the LED off


	//digitalWrite(22, HIGH);		// Zapala diodę
	//delay(secondsToMilliseconds(2)); 


	// call function f once after d milliseconds
    //timerID = Timer.setTimeout(secondsToMilliseconds(30), BatterySaveMode_Modemsleep);	  // Przejdzie w stan uśpienia za 3min czyli 180sec
	//BatterySaveMode_Lightsleep();
	Timer.setInterval(secondsToMilliseconds(3), PrintData);
	Timer.setInterval(secondsToMilliseconds(20), TriggerCD42);

}

void loop()
{	
	Timer.run();
	if ( BatterySaveMode == false )	// Obsługuje Captive portal tylko jeśli nie jest w stanie oszczędzania energii
	{
		ElegantOTA.loop();
	}


	GatherInformation();			

	if ( DeadMole == true)
	{
		myLED.setPixel( 0, L_BLUE, 1 );    // Zapala niebieską diodę na znak żałoby po kreciku
	}
	else if ( MoleInPlace == false &&  ReadyToFire == true )
	{
		myLED.setPixel( 0, L_RED, 1 );			// Zapala czerwoną diodę (Uzbrojone i gotowe do detonacji)
	}
	else if ( MoleInPlace == false &&  ReadyToFire == false )
	{
		myLED.setPixel( 0, L_GREEN, 1 );		// Zapala zieloną diodę (Prawidłowo uzbrojone)
	}
	else if ( MoleInPlace == true &&  ReadyToFire == false )
	{
		myLED.setPixel( 0, L_YELLOW, 1 );		// Zapala żółtą diodę (Popraw wyzwalacz magnetyczny)
	}
	else if ( MoleInPlace == true &&  ReadyToFire == true )
	{	
		digitalWrite(RELAY_01,LOW);				// Odpala petardę
		delay(secondsToMilliseconds(1)); 		// Czeka 2s
		digitalWrite(RELAY_01,HIGH);			// Wyłącza przekaźnik z petardą
		DeadMole = true;						// Zmienia status kreta na "MARTWY"
	}

}

// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void GatherInformation() {

	if ( analogRead(IgniterVoltage) < 250 )	// Sprawdza czy na zapalniku jest napięcie aby sprawdzić pozycję Rotary Switch oraz stan zapalnika
	{	
		ReadyToFire = false;
	}
	else
	{
		ReadyToFire = true;
		if ( BatterySaveMode == false )	// Czy ESP jest już w stanie oszczędzania energii
		{	
			BatterySaveMode_Modemsleep();
			BatterySaveMode=true;
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

    setCpuFrequencyMhz(160);
	delay(1000);
    Serial.print("CPU Freq: ");
    Serial.println(getCpuFrequencyMhz());
	Serial.println("");

    WiFi.disconnect(true);  		// Disconnect from the network
    WiFi.mode(WIFI_OFF);    		// Switch WiFi off
	BatterySaveMode	= true;			// TRUE jeśli ESP32 jest w trybie oszczędzania baterii, potrzebne do tego aby nie wywoływaćAutoConnect
  	delay(10);  					// it doesn't always go to sleep unless you delay(10); yield() wasn't reliable
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
	Serial.print("Battery, Analog Read = ");
	Serial.println(analogRead(IgniterVoltage));

	Serial.print("ReadyToFire = ");
	Serial.println(ReadyToFire);

	Serial.print("MoleInPlace = ");
	Serial.println(MoleInPlace);

	Serial.print("DeadMole = ");
	Serial.println(DeadMole);

	Serial.println("");
}