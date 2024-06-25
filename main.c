#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Define.h"
#include "Ultrasone_sensor.h"

#define DEBOUNCE  _delay_ms(10)

typedef enum {NOODSTOP,
              START,
              SIGNAAL,
              VOLGEN,
              AUTOMATISCH_RIJDEN,
              BOCHT_LINKSOM,
              BOCHT_RECHTSOM,
              BOOMSTOP,
              BOOMRESET
             } AGV;
AGV toestand = START;

int bocht = 0;
int stand = 0;

// Werking motor
// ISR voor PWM motor L
ISR(TIMER4_OVF_vect)    {  PORTH ^= (1 << ENB);  }
ISR(TIMER4_COMPA_vect)  {  PORTH |= (1 << ENB);  }

// ISR voor PWM motor R
ISR(TIMER1_OVF_vect)    {  PORTH ^= (1 << ENA);  }
ISR(TIMER1_COMPA_vect)  {  PORTH |= (1 << ENA);  }

void MOTORL(signed char percentage) {
    if (percentage >= 0 && percentage <= 100) {
        OCR4A = (65536 * percentage) / 100;
    }
}
void MOTORR(signed char percentage) {
    if (percentage >= 0 && percentage <= 100) {
        OCR1A = (65536 * percentage) / 100;
    }
}

void Rechtdoor() {
    PORTB |=  ((1 << IN1) | (1 << IN4)); // AAN
    PORTB &= ~((1 << IN2) | (1 << IN3)); // UIT
}

void HBRUG_UIT() {
    PORTB &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4)); // UIT
}

// Werking LEDS
void Alle_Leds_AAN() {
    PORTH |=  ((1 << LED_TEST)   | (1 << LED_TEST2));
    PORTL |=  ((1 << LED_L_GEEL) | (1 << LED_R_GEEL));
}

void Alle_Leds_UIT() {
    PORTH &= ~((1 << LED_TEST)   | (1 << LED_TEST2));
    PORTL &= ~((1 << LED_L_GEEL) | (1 << LED_R_GEEL));
}

void Leds_Voor_Knipper_AAN_uit() { // AAN uit
    PORTH |=  (1 << LED_TEST);  _delay_ms(500);
    PORTH &= ~(1 << LED_TEST);  _delay_ms(100);
}

void Leds_Voor_Knipper_aan_UIT() { // aan UIT
    PORTH |=  (1 << LED_TEST2);  _delay_ms(100);
    PORTH &= ~(1 << LED_TEST2);  _delay_ms(500);
}

void Leds_Rechts_Knipper_AAN_uit() { // AAN uit
    PORTL |=  (1 << LED_R_GEEL);  _delay_ms(500);
    PORTL &= ~(1 << LED_R_GEEL);  _delay_ms(100);
}

void Leds_Links_Knipper_AAN_uit() { // AAN uit
    PORTL |=  (1 << LED_L_GEEL);  _delay_ms(500);
    PORTL &= ~(1 << LED_L_GEEL);  _delay_ms(100);
}

// functies van de AGV
void Bocht_Maken_Linksom() {
    Rechtdoor();
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(500); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    MOTORL(snelheiduit);        MOTORR(snelheidhard);       _delay_ms(550);  // 90 GRADEN BOCHT MAKEN
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(400); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    MOTORL(snelheiduit);        MOTORR(snelheidhard);       _delay_ms(550);  // 90 GRADEN BOCHT MAKEN
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(800); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    toestand = AUTOMATISCH_RIJDEN;
//    if (stand == 0){
//        toestand = VOLGEN;                                                   // TOESTAND VERANDEREN NAAR VOLGEN
//    } else {
//        toestand = AUTOMATISCH_RIJDEN;                                       // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
//    }
}

void Bocht_Maken_Rechtsom() {
    Rechtdoor();
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(500); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    MOTORL(snelheidhard);       MOTORR(snelheiduit);        _delay_ms(550);  // 90 GRADEN BOCHT MAKEN
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(500); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    MOTORL(snelheidhard);       MOTORR(snelheiduit);        _delay_ms(550);  // 90 GRADEN BOCHT MAKEN
    MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor);  _delay_ms(800); // NOG EEN STUKJE RECHTDOOR RIJDEN.
    if (stand == 0){
        toestand = VOLGEN;                                                   // TOESTAND VERANDEREN NAAR VOLGEN
    } else {
        toestand = AUTOMATISCH_RIJDEN;                                       // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
    }
}

void Pad_Detectie() {
    Rechtdoor();
    // TESTEN VAN INFRAROOD SENSOR LINKS
        if ((PINF & (1 << IR_L)) == 0) {  MOTORL(snelheidrechtdoor);  }         // muur dichtbij
        else if ((PINF & (1 << IR_L)) != 0) {  MOTORL(snelheidbijsturen);  }    // muur weg dus bijsturen andere kant
    // TESTEN VAN INFRAROOD SENSOR RECHTS
        if ((PINF & (1 << IR_R)) == 0) {  MOTORR(snelheidrechtdoor);  }         // muur dichtbij
        else if ((PINF & (1 << IR_R)) != 0) {  MOTORR(snelheidbijsturen);  }    // muur weg dus bijsturen andere kant
    // DETECTIE EINDE RIJSTROOK
        if (((PINF & (1 << IR_R)) && (PINF & (1 << IR_L))) != 0) {              // Voorbij beide balken
            MOTORL(snelheidhard);  MOTORR(snelheidhard);                        // SNELHEID MOTOREN
            if      (bocht == 0) { toestand = BOCHT_LINKSOM; }                  // TOESTAND NAAR BOCHT_LINKSOM
            else if (bocht != 0) { toestand = BOCHT_RECHTSOM;  }}               // TOESTAND NAAR BOOM_RECHTSOM
}

void Mensen_Detectie() {
// TESTEN VAN INFRAROOD SENSOR VOOR
    // WEL MENSEN VOOR AGV AANWEZIG
        if ((PINF & (1 << IR_V)) == 0) {
            PORTH |=  ((1 << LED_TEST) | (1 << LED_TEST2));                         // RODE LEDS OP VOORKANT AAN
            HBRUG_UIT();}                                                           // MOTOREN UIT
    // GEEN MENSEN VOOR AGV AANWEZIG
        else if ((PINF & (1 << IR_V)) != 0) {
            PORTH &= ~((1 << LED_TEST) | (1 << LED_TEST2));                         // RODE LEDS OP VOORKANT UIT
            Rechtdoor();                                                            // MOTOREN AAN,
            MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor); }                // SNELHEID MOTOREN
}

void Bereik_Detectie() {
// TESTEN VAN INFRAROOD SENSOR VOOR & ULTRASOON SENSOR VOOR
    // MENS STAAT WEL IN BEREIK VAN DE AGV;
        if (((PINF & (1 << IR_V)) != 0) && (agv_ultrasoon_voor_midden < 24)) {      // INFRAROODSENSOR ZIET NIKS   AND   ULTRASOONSENSOR WEL   ->  AGV RIJDT WEL
            PORTH &= ~((1 << LED_TEST) | (1 << LED_TEST2));                         // RODE LEDS OP VOORKANT UIT
            Rechtdoor();                                                            // MOTOREN AAN,
            MOTORL(snelheidrechtdoor);  MOTORR(snelheidrechtdoor); }                // SNELHEID MOTOREN
    // MENS STAAT NIET IN BEREIK VAN DE AGV;
        else {                                                                      // INFRAROODSENSOR ZIET WEL    OR    ULTRASOONSENSOR NIET  ->  AGV RIJDT NIET
            PORTH |= ((1 << LED_TEST)| (1 << LED_TEST2));                           // RODE LEDS OP VOORKANT AAN
            HBRUG_UIT(); }                                                          // MOTOREN UIT
}

void Boom_Detectie(){
// DETECTIE BOMEN: BOOM GEDETECTEERD LINKS OF RECHTS
    if (((PINK & (1 << IR_BOOM_links)) == 0) | ((PINK & (1 << IR_BOOM_rechts)) == 0)) {
        HBRUG_UIT();                                                                // MOTOREN UIT
        toestand =  BOOMSTOP;                                                       // TOESTAND NAAR BOOMSTOP
    }
}

// initialisatie pinnen
void init() {
    cli();  // Interrupts uitschakelen tijdens setup

    // Timer PWM motor L
    TCCR4A = (1 << WGM41);  // Fast PWM, TOP=ICR3
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);  // Prescaler = 1
    ICR4 = 65535;  // Set TOP value for 16-bit timer
    TIMSK4 = (1 << OCIE4A) | (1 << TOIE4);
    OCR4A = 0;

    // Timer PWM motor R
    TCCR1A = (1 << WGM11);  // Fast PWM, TOP=ICR1
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // Prescaler = 1
    ICR1 = 65535;  // Set TOP value for 16-bit timer
    TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);
    OCR1A = 0;

    // Invoer pinnen
    DDRK &= ~((1 << START_knop) | (1 << stop_knop) | (1 << toestand_knop)); // // TOEGEVOEGD TOESTAND KNOP, WEGGEHAALD PINNEN VOOR LIMITSWITCHES

    // Uitvoer pinnen
    DDRH |= ((1 << LED_TEST) | (1 << LED_TEST2));
    DDRL |= ((1 << LED_L_GEEL) | (1 << LED_R_GEEL));
    DDRH |= ((1 << ENA) | (1 << ENB));
    DDRB |= ((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4)); // // ANDERE PORT VOOR IN1 IN2 IN3 IN4 DAN ENA & ENB

    // Zet uitvoer pinnen uit
    PORTH &= ~((1 << LED_TEST) | (1 << LED_TEST2));
    PORTL &= ~((1 << LED_L_GEEL) | (1 << LED_R_GEEL));

    // H-brug pinnen LOW
    PORTH &= ~((1 << ENA) | (1 << ENB));
    PORTB &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4)); // // ANDERE PORT VOOR IN1 IN2 IN3 IN4 DAN ENA & ENB

    agv_ultrasoon_init(); // ??? WAT GEBEURT HIER ???

    sei();  // Interrupts inschakelen na setup
}

int main(void) {
    init();

    while (1) {
        // STOP KNOP = INGEDRUKT; ALLE LEDS AAN, MOTOREN UIT
        if ((PINK & (1 << stop_knop)) != 0) {
            Alle_Leds_AAN();
            HBRUG_UIT();
            toestand = NOODSTOP;
        }

        switch (toestand) {
            case NOODSTOP:
            // UIT NOODSTOP SITUATIE GAAN dmv. OPNIEUW INDRUKKEN NOODKNOP
                // STOP KNOP = INGEDRUKT
                if ((PINK & (1 << stop_knop)) != 0) {
                    Alle_Leds_UIT();                                                                // ALLE LEDS UIT
                    toestand = START;                                                               // TOESTAND VERANDEREN NAAR START
                }
            // UIT NOODSTOP SITUATIE GAAN dmv. WISSELEN TOESTAND
                // TOESTAND KNOP = VOLGEN    OR    TOESTAND KNOP = AUTOMATISCH
                if (((PINK & (1 << toestand_knop)) != 0) | ((PINK & (1 << toestand_knop)) == 0)) {  // DUBBEL CONTROLE KNOP INDRUKKEN;
                    if ((PINK & (1 << START_knop)) != 0) {                                          // START KNOP INGEDRUKT,
                        DEBOUNCE;
                        if ((PINK & (1 << START_knop)) == 0) {                                      // START KNOP LOSGELATEN, VERANDEREN TOESTAND
                            DEBOUNCE;
                            Alle_Leds_UIT();                                                        // ALLE LEDS UIT
                            toestand = START;                                                       // TOESTAND VERANDEREN NAAR START
                }}}
            break;

            case START:
                // TOESTAND KNOP = VOLGEN
                if ((PINK & (1 << toestand_knop)) != 0) {
                    if ((PINK & (1 << START_knop)) != 0) {                  // START KNOP INGEDRUKT, LED AAN
                        DEBOUNCE;
                        PORTH |= (1 << LED_TEST2);                          // LED VOOR LINKS/RECHTS AAN
                        if ((PINK & (1 << START_knop)) == 0) {              // START KNOP LOSGELATEN, LED UIT, VERANDEREN TOESTAND
                            DEBOUNCE;
                            PORTH &= ~(1 << LED_TEST2);                     // LED VOOR LINKS/RECHTS UIT
                            toestand = SIGNAAL;                             // TOESTAND VERANDEREN NAAR SIGNAAL
                }}}
                // TOESTAND KNOP = AUTOMATISCH
                else {
                    if ((PINK & (1 << START_knop)) != 0) {                  // START KNOP INGEDRUKT
                        DEBOUNCE;
                        PORTH |= (1 << LED_TEST);                           // LED VOOR LINKS/RECHTS AAN
                        if ((PINK & (1 << START_knop)) == 0) {              // START KNOP LOSGELATEN, LED UIT, VERANDEREN TOESTAND
                            DEBOUNCE;
                            PORTH &= ~(1 << LED_TEST);                      // LED VOOR LINKS/RECHTS UIT
                            toestand = SIGNAAL;                             // TOESTAND VERANDEREN NAAR SIGNAAL
                }}}
            break;

            case SIGNAAL:
                // TOESTAND KNOP = VOLGEN;
                if ((PINK & (1 << toestand_knop)) != 0) {
                    Leds_Voor_Knipper_aan_UIT();                            // LEDS VOOR KNIPPEREN
                    Leds_Voor_Knipper_aan_UIT();                            // LEDS VOOR KNIPPEREN
                    toestand = VOLGEN;                                      // TOESTAND VERANDEREN NAAR VOLGEN
                // TOESTAND KNOP = AUTOMATISCH;
                } else if ((PINK & (1 << toestand_knop)) == 0) {
                    Leds_Voor_Knipper_AAN_uit();                            // LEDS VOOR KNIPPEREN
                    Leds_Voor_Knipper_AAN_uit();                            // LEDS VOOR KNIPPEREN
                    toestand = AUTOMATISCH_RIJDEN;                          // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
                }
            break;

            case VOLGEN:
                stand = 0;
                Bereik_Detectie();
                Pad_Detectie();                                             // TOESTAND NAAR BOCHT_LINKS OF BOCHT_RECHTS
                Boom_Detectie();                                            // TOESTAND VERANDEREN NAAR BOOMSTOP
            break;

            case AUTOMATISCH_RIJDEN:
                PORTH &= ~((1 << LED_TEST)|(1 << LED_TEST2));
                stand = 1;
                Mensen_Detectie();
                Pad_Detectie();                                             // TOESTAND NAAR BOCHT_LINKS OF BOCHT_RECHTS
                Boom_Detectie();                                            // TOESTAND VERANDEREN NAAR BOOMSTOP
            break;

            case BOCHT_LINKSOM:
                PORTH |=  ((1 << LED_TEST)|(1 << LED_TEST2));
                bocht = 1;
                Bocht_Maken_Linksom();                                      // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
            break;

            case BOCHT_RECHTSOM:
                PORTH |=  ((1 << LED_TEST)|(1 << LED_TEST2));
                bocht = 0;
                Bocht_Maken_Rechtsom();                                     // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
            break;

            case BOOMSTOP:
            // BOOM GEDETECTEERD LINKS
                if ((PINK & (1 << IR_BOOM_links)) == 0) {
                    Leds_Links_Knipper_AAN_uit();                           // LED LINKS KORT KNIPPEREN
                    Leds_Links_Knipper_AAN_uit();                           // LED LINKS KORT KNIPPEREN
                    Rechtdoor();                                            // MOTOREN AAN,
                    MOTORL(snelheidrechtdoor); MOTORR(snelheidrechtdoor);   // SNELHEID MOTOREN
                    //_delay_ms(50);                                         // Tijd laten doorrijden
                    toestand = BOOMRESET;                                   // TOESTAND VERANDEREN NAAR BOOMRESET
            // BOOM GEDETECTEERD RECHTS
                } else if ((PINK & (1 << IR_BOOM_rechts)) == 0) {
                    Leds_Rechts_Knipper_AAN_uit();                          // LED RECHTS KORT KNIPPEREN
                    Leds_Rechts_Knipper_AAN_uit();                          // LED RECHTS KORT KNIPPEREN
                    Rechtdoor();                                            // MOTOREN AAN,
                    MOTORL(snelheidrechtdoor); MOTORR(snelheidrechtdoor);   // SNELHEID MOTOREN
                    //_delay_ms(50);                                         // Tijd laten doorrijden
                    toestand = BOOMRESET;                                   // TOESTAND VERANDEREN NAAR BOOMRESET
                }
            break;

            case BOOMRESET:
            // GEEN BOMEN GEDETECTEERD
                if(((PINK & (1 << IR_BOOM_rechts)) != 0) && ((PINK & (1 << IR_BOOM_links)) != 0)) {
                    if (stand == 0){
                        toestand = VOLGEN;                                                   // TOESTAND VERANDEREN NAAR VOLGEN
                    } else {
                        toestand = AUTOMATISCH_RIJDEN;                                       // TOESTAND VERANDEREN NAAR AUTOMATISCH_RIJDEN
                    }
                }
            break;
        } // SLUITEN SWITCH
    } // SLUITEN WHILE LOOP
    return 0;
} // SLUITEN MAIN LOOP
