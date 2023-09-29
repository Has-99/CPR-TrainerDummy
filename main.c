#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LCD_Dir DDRB
#define LCD_Port PORTB
#define RS PB0
#define EN PB1

const unsigned char EYE_LED = PC0;
const unsigned char COMPRESSION_SUCCESS_LED = PD2;
const unsigned char TILT_SUCCESS_LED = PD1;
const unsigned char PINCH_SUCCESS_LED = PD0;
const unsigned char ACTION_BUTTON = PA1;
const unsigned char PRESSURE_SENSOR = PA0;
const unsigned char TILT_SENSOR = PD6;
const unsigned char PINCH_BUTTON = PA2;

const unsigned short COMPRESSION_TIME_THERSHOLD = 20000;
const unsigned short COMPRESSION_PRESSURE_THRESHOLD = 300;
const unsigned short COMPRESSION_COUNT_THRESHOLD = 30;

const unsigned short PRESSURE_SENSOR_ADC_CHANNEL = 0;

void LCD_Command(unsigned char cmnd) {
    LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
    LCD_Port &= ~(1 << RS);                       /* RS=0, command reg. */
    LCD_Port |= (1 << EN);                        /* Enable pulse */
    _delay_us(1);
    LCD_Port &= ~(1 << EN);

    _delay_us(200);

    LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4); /* sending lower nibble */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);
    _delay_ms(2);
}

void LCD_Char(unsigned char data) {
    LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
    LCD_Port |= (1 << RS);                        /* RS=1, data reg. */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);

    _delay_us(200);

    LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);
    _delay_ms(2);
}

void LCD_Init(void) {
    LCD_Dir = 0xFF; /* Make LCD port direction as o/p */
    _delay_ms(20);  /* LCD Power ON delay always >15ms */

    LCD_Command(0x02); /* send for 4 bit initialization of LCD  */
    LCD_Command(0x28); /* 2 line, 5*7 matrix in 4-bit mode */
    LCD_Command(0x0c); /* Display on cursor off*/
    LCD_Command(0x06); /* Increment cursor (shift cursor to right)*/
    LCD_Command(0x01); /* Clear display screen*/
    _delay_ms(2);
}

void LCD_String(char *str) {
    int i;
    for (i = 0; str[i] != 0; i++) /* Send each char of string till the NULL */
    {
        LCD_Char(str[i]);
    }
}

void LCD_String_xy(char row, char pos, char *str) {
    if (row == 0 && pos < 16)
        LCD_Command((pos & 0x0F) | 0x80); /* Command of first row and required position<16 */
    else if (row == 1 && pos < 16)
        LCD_Command((pos & 0x0F) | 0xC0); /* Command of first row and required position<16 */
    LCD_String(str);                      /* Call LCD string function */
}

void LCD_Clear(void) {
    LCD_Command(0x01); /* Clear display */
    _delay_ms(2);
    LCD_Command(0x80); /* Cursor at home position */
}

void LCD_Integer(unsigned int number) {
    char num2Str[256];
    sprintf(num2Str,"%d", number);
    LCD_String(num2Str);
}

unsigned long curTime;

unsigned long timer2OverflowCount = 0;
unsigned long timer2TotalTickCount = 0;

ISR(TIMER2_OVF_vect) { //Timer2's counter has overflowed 
    timer2OverflowCount++;
}

unsigned long micros2() {
    return (unsigned long) timer2OverflowCount*256UL + TCNT2;
}

unsigned long millis2() {
    return micros2()/1000;
}

void initTimer() {
    // TIMER2=================
    TIMSK |= (1 << TOIE2); // Enable timer2 overflow interrupt
    sei(); // Enable global interrupts
    TCCR2 |= (1 << CS20); // Start timer2 with no pre-scaler
}

void initADC() {
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Set prescaller to 128 (bits 2:0 = 111)
    ADMUX |= (1<<REFS0); //Selecting internal reference voltage
    ADCSRA |= (1 << ADEN); //Enable ADC (bit 7 = 1)
}

unsigned short readADC(unsigned char channel) {
    ADMUX = (ADMUX & ~0b00001111) | (channel & 0b00001111); //Copy channel bits to 3:0 position

    ADCSRA |= (1 << ADSC); //Start taking reading (bit 6 = 1)
    while (((ADCSRA>>ADSC) & 1) == 1) {} //Wait until bit 6 = 0

    return ADC;
}

void reset() {
    DDRC |= (1<<EYE_LED); PORTC &= ~(1<<EYE_LED); //pinMode(EYE_LED, OUTPUT);digitalWrite(EYE_LED, LOW);
    DDRD |= (1<<COMPRESSION_SUCCESS_LED); PORTD &= ~(1<<COMPRESSION_SUCCESS_LED); //pinMode(COMPRESSION_SUCCESS_LED, OUTPUT); digitalWrite(COMPRESSION_SUCCESS_LED, LOW);
    DDRD |= (1<<TILT_SUCCESS_LED); PORTD &= ~(1<<TILT_SUCCESS_LED); //pinMode(TILT_SUCCESS_LED, OUTPUT); digitalWrite(TILT_SUCCESS_LED, LOW);
    DDRD |= (1<<PINCH_SUCCESS_LED); PORTD &= ~(1<<PINCH_SUCCESS_LED); //pinMode(PINCH_SUCCESS_LED, OUTPUT); digitalWrite(PINCH_SUCCESS_LED, LOW);
    DDRA &= ~(1<<ACTION_BUTTON); PORTA |= (1<<ACTION_BUTTON); //pinMode(ACTION_BUTTON, INPUT_PULLUP);
    DDRA &= ~(1<<PRESSURE_SENSOR); // pinMode(PRESSURE_SENSOR, INPUT);
    DDRD &= ~(1<<TILT_SENSOR); //pinMode(TILT_SENSOR, INPUT);
    DDRA &= ~(1<<PINCH_BUTTON); PORTA |= (1<<PINCH_BUTTON); //pinMode(PINCH_BUTTON, INPUT_PULLUP);
}

int practiceCompression(unsigned long startMillis) {
    unsigned short numCompressions = 0;

    while (1) {
        curTime = millis2();

        if (curTime - startMillis > COMPRESSION_TIME_THERSHOLD) {
            // CASE: Timeout
            // Serial.println("COMPRESSION PRACTICE: Unsuccessful BECAUSE: Took more than " + String(COMPRESSION_TIME_THERSHOLD / 1000) + "s");
            LCD_Clear();
            LCD_String("CMPRESN: Unsuccess");
            LCD_Command(0xC0);
            LCD_String("Time>");
            LCD_Integer(COMPRESSION_TIME_THERSHOLD / 1000);
            LCD_String("s");
            PORTC &= ~(1<<EYE_LED); //digitalWrite(EYE_LED, LOW);
            PORTD &= ~(1<<COMPRESSION_SUCCESS_LED); //digitalWrite(COMPRESSION_SUCCESS_LED, LOW);

            return 0;
        } else {
            // CASE: Still within time
            if (numCompressions < COMPRESSION_COUNT_THRESHOLD) {
                // CASE: Need more compressions
                if (readADC(PRESSURE_SENSOR_ADC_CHANNEL) > COMPRESSION_PRESSURE_THRESHOLD) {
                    // CASE: Compression detected
                    numCompressions++;
                    _delay_ms(100);
                }
            } else {
                // CASE: Exceeded required compressions
                // Serial.println("COMPRESSION PRACTICE: Successful WITH: " + (String)numCompressions + " compressions WITHIN: " + String((curTime - startMillis) / 1000) + "s");
                LCD_Clear();
                LCD_String("CMPRESN: Success");
                LCD_Command(0xC0);
                LCD_String("Time = ");
                LCD_Integer((curTime - startMillis) / 1000);
                LCD_String(", Count = ");
                LCD_Integer(numCompressions);
                PORTC |= (1<<EYE_LED); //digitalWrite(EYE_LED, HIGH);
                PORTD |= (1<<COMPRESSION_SUCCESS_LED); //digitalWrite(COMPRESSION_SUCCESS_LED, HIGH);

                return 1;
            }
        }
    }
}

int practiceVentilation() {
    int nosePinchSuccess = 0;

    while (1) { // Wait until head tilt and nose pinch is correct
        int tiltSuccess = ((PIND>>~TILT_SENSOR)&1);//!digitalRead(TILT_SENSOR);

        if (tiltSuccess == 0) { //digitalWrite(TILT_SUCCESS_LED, tiltSuccess);
            PORTD |= (1<<TILT_SUCCESS_LED);
        } else {
            PORTD &= ~(1<<TILT_SUCCESS_LED);
        }

        if (nosePinchSuccess) { //digitalWrite(PINCH_SUCCESS_LED, nosePinchSuccess);
            PORTD |= (1<<PINCH_SUCCESS_LED);
        } else {
            PORTD &= ~(1<<PINCH_SUCCESS_LED);
        }

        if (((PINA>>PINCH_BUTTON)&1)==0 /*!digitalRead(PINCH_BUTTON)*/) {
            nosePinchSuccess = 1;
        }

        if (tiltSuccess && nosePinchSuccess) {
            // Serial.println("VENTILATION PRACTICE: Successful");
            LCD_Clear();
            LCD_String("VNTLATION: Successful");
            return 1;
        }
    }
}

int main() {
    //LCD===================
    LCD_Init();

    //ADC===================
    initADC();

    //TIMER=================
    initTimer();

    while(1) {
        reset();

        // Serial.println("VENTILATION PRACTICE: Press button to start");
        
		LCD_Clear();
        LCD_String("VNTLATION:");
        LCD_Command(0xC0);
        LCD_String("Press to start");
		
		
	
		
        while (((PINA>>ACTION_BUTTON)&1)==1 /*digitalRead(ACTION_BUTTON) == HIGH*/) {} // Wait until user presses action button
        // Serial.println("VENTILATION PRACTICE: Started");
        LCD_Clear();
        LCD_String("VNTLATION:");
        LCD_Command(0xC0);
        LCD_String("Started");

        if (practiceVentilation() == 1) {
		
			LCD_Clear();
            LCD_String("VNTLATION: Successful");
            
            // CASE: Ventilation practice is successful
            // Start compression practice
            // Serial.println("COMPRESSION PRACTICE: Press button to start");
            LCD_Clear();
            LCD_String("COMPRESN:");
            LCD_Command(0xC0);
            LCD_String("Press to start");
            while (((PINA>>ACTION_BUTTON)&1)==1 /*digitalRead(ACTION_BUTTON) == HIGH*/) {} // Wait until user presses action button

            // Serial.println("COMPRESSION PRACTICE: Started");
            LCD_Clear();
            LCD_String("COMPRESN:");
            LCD_Command(0xC0);
            LCD_String("Started");

            if (practiceCompression(millis2()) == 1) {
                // CASE: Compression practice is successful
                // Greet user and offer start again
                // Serial.println("TRAINING: Successful. Press button to start again");
                LCD_Clear();
                LCD_String("TRAIN: Success");
                LCD_Command(0xC0);
                LCD_String("Press to restart");
                while (((PINA>>ACTION_BUTTON)&1)==1 /*digitalRead(ACTION_BUTTON) == HIGH*/) {} // Wait until user presses action button
            } else {
                // CASE: Compression practice is unsuccessful
                // Serial.println("TRAINING: Unsuccessful BECAUSE: Compression practice failed. Press button to start again");
                LCD_Clear();
                LCD_String("TRAIN: Cmpresn faild");
                LCD_Command(0xC0);
                LCD_String("Press to restart");
                while (((PINA>>ACTION_BUTTON)&1)==1 /*digitalRead(ACTION_BUTTON) == HIGH*/) {} // Wait until user presses action button
            }
        }

        _delay_ms(500);
    }
}

