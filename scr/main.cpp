#define FIRMWARE_VERSION	"v0.1.0"
#define FIRMWARE_DATE	"27.01.2025"
#include <Arduino.h>


// Biblioteki potrzebne do przejścia w stan uspienia
#include <coredecls.h>         // crc32()
#include <PolledTimeout.h>
//#include <include/WiFiState.h> // WiFiState structure details


//Biblioteki potrzebne dla AutoConnect
#include <ESP8266WiFi.h>                // Replace 'ESP8266WiFi.h' with 'WiFi.h. for ESP32
#include <ESP8266WebServer.h>					  // Replace 'ESP8266WebServer.h'with 'WebServer.h' for ESP32
#include <AutoConnect.h>
ESP8266WebServer			Server;				    // Replace 'ESP8266WebServer' with 'WebServer' for ESP32
AutoConnect			Portal(Server);
//AutoConnectConfig	Config;       		  // Enable autoReconnect supported on v0.9.4
AutoConnectConfig 	Config("Lazienka_ESP32", "12345678");
String viewCredential(PageArgument&);
String delCredential(PageArgument&);
const String  FrmDate = "Current version: " + String(FIRMWARE_VERSION) + " <br> Dated: " + String(FIRMWARE_DATE);
const char* fw_ver = FrmDate.c_str();	   // Display an extra string on the update screen, REF: https://hieromon.github.io/AutoConnect/otabrowser.html#display-an-extra-string-on-the-update-screenenhanced-wv130


#include <SimpleTimer.h>				          // https://github.com/jfturcot/SimpleTimer
#include <TimeLib.h>
SimpleTimer Timer;

#include <RGBLed.h>

//ZMIENNE GLOBALNE
boolean		 ReadyToFire		= false;	// TRUE jeśli jest rotary switch poziom 1 / FALSE jeśli jest rotary switch poziom 2
boolean		 MoleInPlace		= false;	// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 DeadMole			= false;	// TRUE jeśli kret został wykryty / FALSE kret jest poza polem rażenia
boolean		 IgniterReady		= false;	// TRUE jeśli zapalnik jest sprawny (pomiar przewodności) / FALSE jeśli zapalnik jest przerwany (uszkodzony lub po odpaleniu petardy)
volatile int timerID			= -1;		//Przetrzymuje ID Timera

//STAŁE

#define MoleDetectionPin	  14				// D5 (GPIO14) Pin na którym jest kontaktron / AM312 PIR Sensor 
#define IgnitionLevel_1		  5				    // D1 (GPIO5)  Pin na którym jest rotary switch poziom 1
#define IgnitionLevel_2		  4				    // D2 (GPIO4)  Pin na którym jest rotary switch poziom 2
#define IgniterVoltage		  A0				// A0 (ADC0)   Pin pomiaru napięcia zapalnika w celu sprawdzenia czy sprawny
#define RELAY_01			  0				    // D3 (GPIO0)  Pin do przekaźnika inicjującego wybuch petardy
#define R_LED			      2				    // D4 (GPIO2) Pin na którym jest kolor czerwony diody RGB (62 or 68 Ohm resistor for 3.3V)
#define G_LED			      13				// D7 (GPIO13)Pin na którym jest kolor zielony diody RGB (62 or 56 Ohm resistor for 3.3V)
#define B_LED			      12				// D6 (GPIO12) Pin na którym jest kolor niebieski diody RGB (62 62Ohm resistor for 3.3V)
RGBLed led(R_LED, G_LED, B_LED, RGBLed::COMMON_CATHODE);  // Deklaracja diody RGB z biblioteki <RGBLed.h>
//---------------------------------------------------------------------------------------------------------------------------------------------

// put function declarations here:
void GatherInformation();				// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void BatterySaveMode_Modemsleep();		// Przejście w stan uśpienia (Forced Modem-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Lightsleep();		// Przejście w stan uśpienia (Forced Light-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf



void wakeupCallback() {  // unlike ISRs, you can do a print() from a callback function
#ifdef testPoint
  digitalWrite(testPoint, LOW);  // testPoint tracks latency from WAKE_UP_PIN LOW to testPoint LOW
#endif
  Serial.print(F("millis() = ")); // show that RTC / millis() is stopped in Forced Light Sleep
  Serial.println(millis());  // although the CPU may run for up to 1000 mS before fully stopping
  Serial.println(F("Woke from Forced Light Sleep - this is the callback"));
}

void setup() {




  Serial.begin(115200);
	Serial.println("Serial port initiated");
	// Autoconnect
	/*
	Config.hostName 		= "Kretonator_2000";			// Sets host name to SotAp identification
	//Config.apid 			= "Lazienka_ESP22 ";			// SoftAP's SSID.
	//Config.psk 				= "12345678";				// Sets password for SoftAP. The length should be from 8 to up to 63.
	Config.homeUri 			= "/_ac";						// Sets home path of Sketch application
	Config.retainPortal 	= true;                         // Launch the captive portal on-demand at losing WiFi
	Config.autoReconnect 	= true;							      		// Automatically will try to reconnect with the past established access point (BSSID) when the current configured SSID in ESP8266/ESP32 could not be connected.
	Config.ota 				= AC_OTA_BUILTIN;				      		// Specifies to include AutoConnectOTA in the Sketch.
	Config.otaExtraCaption	= fw_ver;						     		// Display an extra string on the update screen, REF: https://hieromon.github.io/AutoConnect/otabrowser.html#display-an-extra-string-on-the-update-screenenhanced-wv130
	Config.menuItems = Config.menuItems | AC_MENUITEM_DELETESSID;	    // https://hieromon.github.io/AutoConnect/apiconfig.html#menuitems
	Portal.config(Config);										        // Don't forget it.
	if (Portal.begin())												    // Starts and behaves captive portal
	{	
		Serial.println("WiFi connected: " + WiFi.localIP().toString());
	}
	*/
	
	pinMode(MoleDetectionPin, INPUT);					      // Deklaracja pinu z kontaktronem
	pinMode(IgnitionLevel_1, INPUT);					      // Deklaracja pinu bezpiecznego uzbrajania
	pinMode(IgnitionLevel_2, INPUT);					      // Deklaracja pinu stanu uzbrojonego, oczekiwanie na kreta i inicjalizacja eksplozji
	pinMode(R_LED, INPUT);					                  // Deklaracja pinu z kolorem czerwonym diody RGB
	pinMode(G_LED, INPUT);					                  // Deklaracja pinu z kolorem zielonym diody RGB
	pinMode(B_LED, INPUT);					                  // Deklaracja pinu z kolorem niebieskim diody RGB
	pinMode(RELAY_01, OUTPUT);				                  // Deklaracja pinu z przekaźnikiem wyzwalającym eksplozję petardy
	
	pinMode(D4, OUTPUT);					                  // Deklaracja pinu z wbudowaną diodą
	digitalWrite(RELAY_01,LOW);								  // Wyłącza przekaźnik z petardą

	// call function f once after d milliseconds
    timerID = Timer.setTimeout(3000, BatterySaveMode_Lightsleep);	  // Przejdzie w stan uśpienia za 3min czyli 180sec

}

void loop()
{	
	Timer.run();
	/*
	if ( Timer.isEnabled(timerID) )		// Portal Auto connect będzie aktywny tylko jeśli ESP nie przejdzie w stan uśpienia co nastąpi po 3min od uruchomienia
	{
			Portal.handleClient();
	}
	*/
	digitalWrite(D4,HIGH);		// zapala diodę
	//Serial.println("Zapala diodę.");
	delay(200); 			    // Czeka 200ms
	//Serial.println("Gasi diodę");
	digitalWrite(D4,LOW);		// Gazi diodę
	delay(200); 			    // Czeka 200ms


	GatherInformation();			

	if ( DeadMole == true)
	{
		led.setColor(RGBLed::BLUE);			// Zapala niebieską diodę na znak żałoby po kreciku
	}
	else if ( MoleInPlace == false &&  ReadyToFire == true )
	{
		led.setColor(RGBLed::RED);			// Zapala czerwoną diodę (Uzbrojone i gotowe do detonacji)
	}
	else if ( MoleInPlace == false &&  ReadyToFire == false )
	{
		led.setColor(RGBLed::GREEN);		// Zapala zieloną diodę (Prawidłowo uzbrojone)
	}
	else if ( MoleInPlace == true &&  ReadyToFire == false )
	{
		led.setColor(RGBLed::YELLOW);		// Zapala żółtą diodę (Popraw wyzwalacz magnetyczny)
	}
	else if ( MoleInPlace == true &&  ReadyToFire == true )
	{	
		digitalWrite(RELAY_01,HIGH);		// Odpala petardę
		delay(2000); 						// Czeka 2s
		digitalWrite(RELAY_01,LOW);			// Wyłącza przekaźnik z petardą
		DeadMole = true;					// Zmienia status kreta na "MARTWY"
	}

}

// Definicja funkcji zbierającej onformacje o KRETONATORZE 2000
void GatherInformation() {

	if ( analogRead(IgniterVoltage) < 500 )	// Sprawdza czy na zapalniku jest napięcie aby sprawdzić pozycję Rotary Switch oraz stan zapalnika
	{	
		ReadyToFire = false;
	}
	else
	{
		ReadyToFire = true;
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


// REF: https://github.com/esp8266/Arduino/issues/6642

// Przejście w stan uśpienia (Forced Modem-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Modemsleep()
{
  WiFi.forceSleepBegin();  // alternate method of Forced Modem Sleep without saving WiFi state
  delay(10);  // it doesn't always go to sleep unless you delay(10); yield() wasn't reliable
  Serial.println("Forced Modem-sleep");
}

// Przejście w stan uśpienia (Forced Light-sleep) REF: https://www.espressif.com/sites/default/files/9b-esp8266-low_power_solutions_en_0.pdf
void BatterySaveMode_Lightsleep()
{
  WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
  delay(10);
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  // gpio_pin_wakeup_enable(GPIO_ID_PIN(MoleDetectionPin), GPIO_PIN_INTR_LOLEVEL);
  Serial.println("Forced Light-sleep");
}
