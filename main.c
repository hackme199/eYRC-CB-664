/*
*
* Team Id: eYRC#664
* Author List: Divyansh Tripathi, Anuj Pratap Singh, Anurag Raj, Shikhar Verma
* Filename:
* Theme: Construct-o-Bot
* Functions: instruction, forward_wls()
* Global Variables: k --> used as switch variable character

					c --> active when white area is detected, specifically declared for black & white, zig-zag part,
						  takes values from 0 to 30
						  
					p --> active when white area is detected, specifically declared for black & white, zig-zag part,
					takes values from 0 to 55	 
					
					ADC_Conversion(unsigned char) --> defines channel number of white line sensor and converts input from it to digital values
					
					ADC_Value --> stores input values from white line sensor
					
					sharp --> stores sharp sensor's reading
					
					sharp2 --> stores 2nd sharp sensor's reading
					
					distance --> stores distance of obstacle from sharp
					
					value ,value2 --> Stores the Analog value of sharp
					
					Shaft_count --> stores number of pulses of encoder
*
*/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> 
#include "lcd.c"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, sharp2, distance, adc_reading;
unsigned int value, value2;
unsigned char flag = 0;
volatile unsigned long int Shaft_count=0;

int k=0,c=0,p=0;


/*
*
* Function Name: lcd_port_config
* Input: void
* Output: void
* Logic: initializes lcd ports 
* Example Call: lcd_port_config()
*
*/
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
*
* Function Name: adc_pin_config
* Input: void
* Output: void
* Logic: initializes analog to digital conversion ports
* Example Call:adc_port_config()
*
*/
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/*
*
* Function Name: motion_pin_config
* Input: void
* Output: void
* Logic: Function to configure ports to enable robot's motion
* Example Call: motion_pin_config()
*
*/
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
*
* Function Name: encoder_pin_config
* Input: void
* Output: void
* Logic: Function to configure ports for encoder of motor
* Example Call: encoder_pin_config()
*
*/
void encoder_pin_config()
{
	DDRD=0xFE;  //PIN 0 set as input
	PORTD=0x01; //pull up enabled on PIN 0
	
	EICRA=0x03;   //RISING EDGE
	EIMSK=0x01;   //INT0 declared

	sei();  //turns on interrupt
}

/*
*
* Function Name: peri_init
* Input: void
* Output: void
* Logic: Function to configure ports for gripper and buzzer
* Example Call: peri_init()
*
*/
void peri_init (void)
{
	DDRG = DDRG | 0x0F;    //Gripper
	PORTG = PORTG & 0xF0;
	DDRH = DDRH | 0x01;    //Buzzer
	PORTH = PORTH | 0x01; 
}

/*
*
* Function Name: servo1_pin_config
* Input: void
* Output: void
* Logic: Configures pin for servo motor 1 operation
* Example Call: servo1_pin_config()
*
*/
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
*
* Function Name: servo2_pin_config
* Input: void
* Output: void
* Logic: Configures pin for servo motor 2 operation
* Example Call: servo2_pin_config()
*
*/
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*
*
* Function Name: port_init
* Input: void
* Output: void
* Logic: initializes all ports of board
* Example Call: port_init()
*
*/
//Function to Initialize PORTS
void port_init()
{
  lcd_port_config();
  adc_pin_config();
  encoder_pin_config();
  motion_pin_config();  
  servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
  servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
  peri_init();
}

/*
*
* Function Name: timer1_init
* Input: void
* Output: void
* Logic: //TIMER1 initialization in 10 bit fast PWM mode
         //prescalar:256
         // WGM: 7) PWM 10bit fast, TOP=0x03FF
         // actual value: 42.187Hz
* Example Call: timer1_init(void)
*
*/

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
*
* Function Name: timer5_init
* Input: void
* Output: void
* Logic: // Timer 5 initialized in PWM mode for velocity control
         // Prescalar:256
         // PWM 8bit fast, TOP=0x00FF
         // Timer Frequency:225.000Hz
* Example Call: timer5_init(void)
*
*/

void timer5_init()
{
  TCCR5B = 0x00;  //Stop
  TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;  //Output compare register high value for Left Motor
  OCR5AL = 0xFF;  //Output compare register low value for Left Motor
  OCR5BH = 0x00;  //Output compare register high value for Right Motor
  OCR5BL = 0xFF;  //Output compare register low value for Right Motor
  OCR5CH = 0x00;  //Output compare register high value for Motor C1
  OCR5CL = 0xFF;  //Output compare register low value for Motor C1
  TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
             For Overriding normal port functionality to OCRnA outputs.
              {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
  
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=20)
}

/*
*
* Function Name: adc_init
* Input: void
* Output: void
* Logic: sets up adc ports
* Example Call: adc_init()
*
*/

void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;    //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
*
* Function Name: adc_init
* Input: unsigned char Ch
* Output: unsigned char a
* Logic: Function For ADC Conversion
* Example Call: ADC_Conversion(1)
*
*/

unsigned char ADC_Conversion(unsigned char Ch) 
{
  unsigned char a;
  if(Ch>7)
  {
    ADCSRB = 0x08;
  }
  Ch = Ch & 0x07;        
  ADMUX= 0x20| Ch;         
  ADCSRA = ADCSRA | 0x40;    //Set start conversion bit
  while((ADCSRA&0x10)==0);  //Wait for conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}


/*
*
* Function Name: print_sensor
* Input: char row, char coloumn,unsigned char channel
* Output: void
* Logic: Function To Print Sensor Values At Desired Row And Column Location on LCD
* Example Call: lcd_print(2,1,k,3)
*
*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
  
  ADC_Value = ADC_Conversion(channel);
  lcd_print(row, coloumn, ADC_Value, 3);
}

/*
*
* Function Name: Sharp_GP2D12_estimation
* Input: adc reading of sharp
* Output: void
* Logic:This Function calculates the actual distance in millimeters(mm) from the input
        analog value of Sharp Sensor.
* Example Call: Sharp_GP2D12_estimation(sharp);
*
*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/*
*
* Function Name: velocity
* Input: unsigned char left_motor, unsigned char right_motor
* Output: void
* Logic:Function for velocity control
* Example Call: velocity(100,100)
*
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}

/*
*
* Function Name: motion_set
* Input: unsigned char Direction
* Output: void
* Logic:Function used for setting motor's direction
* Example Call: motion_set(0x0A)
*
*/
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;     // removing upper nibble for the protection
 PortARestore = PORTA;     // reading the PORTA original status
 PortARestore &= 0xF0;     // making lower direction nibble to 0
 PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
 PORTA = PortARestore;     // executing the command
}

/*
*
* Function Name: servo_1
* Input: unsigned char degrees
* Output: void
* Logic:Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
* Example Call: servo1(63)
*
*/
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

/*
*
* Function Name: servo_2
* Input: unsigned char degrees
* Output: void
* Logic:Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
* Example Call: servo1(63)
*
*/
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees * 0.512) + 34.56;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

/*
*
* Function Name: forward
* Input: void
* Output: void
* Logic: moves robot forward
* Example Call: forward() 
*
*/
void forward (void) 
{
  motion_set (0x06);
}

/*
*
* Function Name: right
* Input: void
* Output: void
* Logic: moves robot right
* Example Call: right()
*
*/

void right()
{
  motion_set(0x0A);
}

/*
*
* Function Name: left
* Input: void
* Output: void
* Logic: moves robot left
* Example Call: left()
*
*/
void left()
{
  motion_set(0x05);
}

/*
*
* Function Name: stop
* Input: void
* Output: void
* Logic: stops the robot 
* Example Call: stop()
*
*/
void stop (void)
{
  motion_set (0x00);
}

/*
*
* Function Name: up
* Input: int degree
* Output: void
* Logic: moves gripper by specified angle
* Example Call: up(65)
*
*/
void up(int degree)
{
	servo_2 (degree);
	_delay_ms(1000);
}

/*
*
* Function Name: down
* Input: int degree
* Output: void
* Logic: moves gripper by specified angle
* Example Call: down(65)
*
*/
void down (int degree)
{
	servo_2 (degree);
	_delay_ms(1000);
}

/*
*
* Function Name: grip
* Input: int degree
* Output: void
* Logic: grips the object
* Example Call: grip(65)
*
*/
void grip (int degree) // grip the object
{
	servo_1 (degree);
	_delay_ms(1000);
}

/*
*
* Function Name: ungrip
* Input: int degree
* Output: void
* Logic: ungrips the object
* Example Call: grip(65)
*
*/
void ungrip (int degree) // grip the object
{
	servo_1 (degree);
	_delay_ms(1000);
}

/*
*
* Function Name: pick
* Input: void
* Output: void
* Logic: picks objects
* Example Call: pick()
*
*/
void pick()
{
	stop();
	_delay_ms(50);
	up(180);
	ungrip(70);
	_delay_ms(50);
	down(102);
	_delay_ms(105);
	grip(107);
	_delay_ms(200);
	up(180);
}

/*
*
* Function Name: place
* Input: void
* Output: void
* Logic: places objects
* Example Call: place()
*
*/
void place()
{
	stop();
	_delay_ms(50);
	down(165);
	_delay_ms(50);
	ungrip(70);
	_delay_ms(50);
	up(180);
	
}

/*
*
* Function Name: init_devices
* Input: void
* Output: void
* Logic : initializes all ports
* Example Call: init_devices (void)
*
*/
void init_devices (void)
{
   cli(); //Clears the global interrupts
  port_init();
  adc_init();
  timer1_init();
  timer5_init(); 
  sei();   //Enables the global interrupts
}

/*
*
* Function Name: buzzer
* Input: void
* Output: void
* Logic : initializes buzzer
* Example Call: buzzer(void)
*
*/
void buzzer(unsigned char status)
{
	DDRH=0x01;
	PORTH=status;	
}

/*
*
* Function Name: buzzerON
* Input: void
* Output: void
* Logic : turns on buzzer
* Example Call: buzzerON(void)
*
*/
void buzzerON()
{
	unsigned char n=0x01;
	buzzer(n);
}

/*
*
* Function Name: buzzerOFF
* Input: void
* Output: void
* Logic : turns off buzzer
* Example Call: buzzerOFF(void)
*
*/
void buzzerOFF()
{
	unsigned char n=0x00;
	buzzer(n);
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic : turns the bot right until black line is encountered
* Example Call: right_turn_wls()
*
*/
void right_turn_wls()
{
	while(1)
	{
		if(ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F)
		{
			right();
			velocity(100,100);
		}
		break;
	}
}

/*
*
* Function Name: right_turn_bls
* Input: void
* Output: void
* Logic : turns the bot right until white line is encountered
* Example Call: right_turn_bls()
*
*/
void right_turn_bls()
{
	while(1)
	{
		if(ADC_Conversion(2)>0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)>0x0F)
		{
			right();
			velocity(100,100);
		}
		break;
	}
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic : turns the bot left until black line is encountered
* Example Call: left_turn_wls()
*
*/
void left_turn_wls()
{
	while(1)
	{
		if(ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F)
		{
			left();
			velocity(100,100);
		}
		break;
	}
}

/*
*
* Function Name: left_turn_bls
* Input: void
* Output: void
* Logic : turns the bot left until black line is encountered
* Example Call: left_turn_bls()
*
*/
void left_turn_bls()
{
	while(1)
	{
		if(ADC_Conversion(2)>0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)>0x0F)
		{
			left();
			velocity(150,150);
		}
		break;
	}
}

/*
*
* Function Name: wallfollow
* Input: void
* Output: void
* Logic : makes the bot follow path between two walls
* Example Call: wallfollow()
*
*/
void wallfollow(void)
{	
	while((ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F))
	{
		sharp = ADC_Conversion(9);
		sharp2 = ADC_Conversion(10);				    //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);
		value2 = Sharp_GP2D12_estimation(sharp2);	    //Stores Distance calculated in a variable "value".
		
	
		if((value>100) && (value2>100))
		{
			forward();
			velocity(100,100);
		}
		
		if((value<160) && (value2>140))
		{
			forward();
			velocity(100,0);
		}
		if((value>140) && (value2<160))
		{
			forward();
			velocity(0,100);
		}
	}
}

/*
*
* Function Name: zigzag
* Input: void
* Output: void
* Logic: Uses logic to guide the bot on the zizzag dashed line in the path
* Example Call: zigzag(); //bot will pass through dash line
*
*/
void zigzag(void)
{
	if(ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14 && ADC_Conversion(3)<0x14)
	{	  while (ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14 && ADC_Conversion(3)<0x14)
		{
			c += 1;
			lcd_print(2,5,c,2);
			left();
			velocity(100, 100);

			while(ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14 && ADC_Conversion(3)<0x14 && c>30)
			{   p+=1;
				lcd_print(2,8,p,2);
				right();
				velocity(100, 100);
				
				while (ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14 && ADC_Conversion(3)<0x14 && p>55)
				{   
					while(p>=20)
					 {	p=p-1;
						 lcd_print(2,8,p,2);
					 left();
					 velocity(100, 100);
					 }
					p=0;
					c=0;
					
				}
			}
		}
	}
}

/*
*
* Function Name: wnodecount
* Input: void
* Output: void
* Logic: Guides bot on white nodes by giving intstruction to turn left or right
* Example Call: wnodecount() //turn left on node 'C'
*
*/

void wnodecount(void)
{
	//if ((ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14 && ADC_Conversion(3)<0x14)||(ADC_Conversion(2)<0x14 && ADC_Conversion(1)<0x14) || (ADC_Conversion(2)<0x14 && ADC_Conversion(3)<0x14))
	//{
		while(((ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F)||(ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(3)<0x0F)))
		{	forward();
			velocity(100, 100);
			_delay_ms(20);
		}
	//}
	k = k + 1;
	lcd_print(2,1,k,3);
	
	instruction(9,'F');
	instruction(9,'R');
	instruction(9,'C');
	instruction(9,'w');
	instruction(22,'F');
	instruction(22,'L');
	instruction(22,'C');
	instruction(22,'w');
//	instruction(22,'k');
}

/*
*
* Function Name: nodecount
* Input: void
* Output: void
* Logic: Uses logic to count nodes in terms of alphabet when it passed through a node
* Example Call: nodecount(); //count node when called
*
*/
void nodecount(void)
{
	    if ((ADC_Conversion(2)>0x96 && ADC_Conversion(1)>0x96 && ADC_Conversion(3)>0x96) || (ADC_Conversion(2)>0x96 && ADC_Conversion(1)>0x96) || (ADC_Conversion(2)>0x96 && ADC_Conversion(3)>0x96))
	    {
		    while ((ADC_Conversion(2)>0x96 && ADC_Conversion(1)>0x96 && ADC_Conversion(3)>0x96) || (ADC_Conversion(2)>0x96 && ADC_Conversion(1)>0x96) || (ADC_Conversion(2)>0x96 && ADC_Conversion(3)>0x96))
		    {
			    forward();
			    velocity(100,100);
				 _delay_ms(20);
		    }
      
		    k = k + 1;
			
		}
		lcd_print(2,1,k,3);
		
}

/*
*
* Function Name: forward_cm
* Input: void
* Output: void
* Logic: Uses white line sensors to go forward by the distance specified in cm
* Example Call: forward_cm(2); //Goes forward by 2 cm
*
*/
void forward_cm(unsigned long int dist)
{
	float reqd_count;
	
	
	reqd_count=dist*24.54;
	reqd_count=(unsigned long int)reqd_count;
	
	while(1)
	{   
		//lcd_print(2,13,Shaft_count,3);
		forward();
		velocity(100,100);
		if (Shaft_count>reqd_count)
		{
			break;
		}
	}
	
	
	
	//stop();
	//_delay_ms(100);
}

/*
*
* Function Name: right_cm
* Input: void
* Output: void
* Logic: Uses white line sensors to go right by the distance specified in cm
* Example Call: right_cm(2); //Goes right by 2 cm
*
*/
void right_cm(int dist)
{
	float reqd_count;

	
	reqd_count=dist*24.54;
	reqd_count=(unsigned long int)reqd_count;
	
	while(1)
	{   
		//lcd_print(2,13,Shaft_count,3);
		right();
		velocity(120,120);
		if (Shaft_count>reqd_count)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	
	
}

/*
*
* Function Name: left_cm
* Input:
* Output: void
* Logic: Uses white line sensors to go left by the distance specified in cm
* Example Call: left_cm(2); //Goes left by 2 cm
*
*/

void left_cm(int dist)
{
	float reqd_count=0;
	
	
	reqd_count=dist*24.54;
	reqd_count=(unsigned long int)reqd_count;
	
	while(1)
	{   
		//lcd_print(2,13,Shaft_count,3);
		left();
		velocity(120,120);
		if (Shaft_count>reqd_count)
		{
			stop();
			_delay_ms(100);
			break;
		}
	}
	
	
}

/*
*
* Function Name: blacknwhite
* Input: void
* Output: void
* Logic: Uses logic to guide the bot on the black and white inverted path between nodes
* Example Call: blacknwhite(); //bot will pass through inverted line
*
*/

void blacknwhite(void)
{
	//while ((ADC_Conversion(2)<0x14 && ADC_Conversion(1)>0x14 && ADC_Conversion(3)>0x14))
	//{
		
		if (ADC_Conversion(2)<0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)>0x0F) // forward -> when M white
		{
			c = 0;
			p = 0;
			forward();
			velocity(100, 100);
		//	_delay_ms(10);
		}
		if ((ADC_Conversion(2)>0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)>0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F  && ADC_Conversion(3)>0x0F)) //left -> when L white
		{
			c = 0;
			p = 0;
			//left();
			forward();
			velocity(40, 100);
			//_delay_ms(2);

		}
		if ((ADC_Conversion(2)>0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)<0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)<0x0F)) //right -> when R white
		{
			c = 0;
			p = 0;
			//right();
			forward();
			velocity(100, 40);
			//_delay_ms(2);
		}

		if ((ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(3)<0x0F))
		{
			wnodecount();
		}
		

		if (ADC_Conversion(2)>0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F)
		{
			 k=k+1;
		}
		
		
}

/*
*
* Function Name: instruction
* Input: node, ch
* Output: void
* Logic: Uses logic to guide bot on nodes by giving intstruction to turn left or right
* Example Call: instruction('C','L'); //turn left on node 'C'
*
*/

void instruction(int node, char ch)
{
	if (k == node)
	{  
		switch (ch)
		{
			case 'L':Shaft_count=0;left_cm(12); break;  // turn left 90 degree
			case 'R':Shaft_count=0;right_cm(18); break;  // turn right 90 degree
			case 'r':right_turn_wls(); break;
			case 'l':left_turn_wls(); break;
			case 'F':Shaft_count=0;forward_cm(11); break;//_delay_ms(800); break;
			case 'P':pick(); break;//_delay_ms(640); right(); _delay_ms(950); break;
			case 'C':place(); break;//_delay_ms(640); right(); _delay_ms(950); break;
			//  case 'P':pick(); _delay_ms(640); right(); _delay_ms(950); break;
			//  case 'C':place(); _delay_ms(640); right(); _delay_ms(950); break;
			//  case 'c':place(); _delay_ms(170); right(); _delay_ms(1640); break;
			//  case 'p':place(); _delay_ms(170); soft_right(); _delay_ms(1640); break;
			case 'w':Shaft_count=0; left_cm(18); break;
			case 'T':left(); velocity(100,100); _delay_ms(2000); break;  // turn right 180 degree
			case 'S':stop();  _delay_ms(1000); break;
			case 'x':right();velocity(100,100);_delay_ms(1300);break;
		    case 'y':left();velocity(100,100);_delay_ms(1300);break;
			case 'k':right_turn_bls();velocity(100,100);break;
			case 'j':left_turn_bls();velocity(100,100);break;
		}
	}
}


/*
*
* Function Name: main
* Input: void
* Output: int
* Logic: contains all executable statements (main body)
* Example Call: main()
*/
int main()
{
  init_devices();
  lcd_set_4bit();
  lcd_init();
  
  while(1)
  {	 
    print_sensor(1,1,3);  //Prints value of White Line Sensor1
    print_sensor(1,5,2);  //Prints Value of White Line Sensor2
    print_sensor(1,9,1);  //Prints Value of White Line Sensor3
	
	while (k==8 || k==9 || k==21 || k==22)//&& (ADC_Conversion(2)<0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)>0x0F))
	blacknwhite();
	
	if(k==29 || k==32 || k==39 || k==42 || k==45 || k==52 || k==55) //|| k==30|| k==33 || k==38 || k==45 || k==48 || k==53)
	wallfollow();
	if(k==26)
    zigzag();
    if (ADC_Conversion(2)>0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)<0x0F)
    {
      c = 0;
	  p=0;
      forward();
      velocity(90,100);
      //_delay_ms(10);
    }
	
    if ((ADC_Conversion(2)>0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)<0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(1)>0x0F && ADC_Conversion(3)<0x0F))
    {
      c = 0;
      p=0;
      forward();
      velocity(50,100);
    }

    if ((ADC_Conversion(2)>0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)>0x0F) || (ADC_Conversion(2)<0x0F && ADC_Conversion(1)<0x0F && ADC_Conversion(3)>0x0F))
    {
      c = 0;
      p=0;
	  forward();
      velocity(100,50);
	  //right();
	  //velocity(100,100);
      //_delay_ms(10);
    }
			
//	if(k==4)
//	wallfollow();

	
    if((ADC_Conversion(2)>0x64 && ADC_Conversion(1)>0x64 && ADC_Conversion(3)>0x64) || (ADC_Conversion(2)>0x64 && ADC_Conversion(1)>0x64) || (ADC_Conversion(2)>0x64 && ADC_Conversion(3)>0x64))
    {  
			nodecount();
			instruction(1,'F');
			instruction(1,'r');
			
			instruction(2,'F');
			instruction(2,'r');
			
     		instruction(3,'F');
			instruction(3,'L');
			instruction(3,'l');		
			instruction(3,'P');
//			instruction(3,'x');
			instruction(3,'R');
			instruction(3,'r');
		
	        instruction(8,'F');
			instruction(8,'R');
			instruction(8,'k');
			
			instruction(11,'F');
			instruction(11,'r');
			
			instruction(16,'F');
			instruction(16,'L');
			instruction(16,'l');
			instruction(16,'P');
			instruction(16,'L');
			instruction(16,'l');
			
			instruction(21,'F');
			instruction(21,'L');
			instruction(21,'l');

			instruction(24,'F');
			instruction(24,'r');
		
			instruction(25,'F');	
			instruction(25,'L');
			instruction(25,'l');
			instruction(25,'P');
			instruction(25,'R');
			
			instruction(26,'F');
			instruction(26,'R');
			instruction(26,'r');
			
			instruction(27,'F');
			instruction(27,'C');
			instruction(27,'L');
			instruction(27,'l');
		
		    instruction(28,'F');
			instruction(28,'R');
			instruction(28,'r');
			instruction(28,'P');
			instruction(28,'L');
			instruction(28,'l');
			
			instruction(29,'F');
			instruction(29,'L');
			instruction(29,'l');
			
			instruction(30,'F');
			instruction(30,'C');
			instruction(30,'R');
			instruction(30,'r');
			
			instruction(31,'F');
			instruction(31,'R');
			instruction(31,'r');
			instruction(31,'P');
			instruction(31,'R');
			instruction(31,'r');
			
			instruction(32,'F');
			instruction(32,'L');
			instruction(32,'l');
			
			instruction(33,'F');
			instruction(33,'R');
			instruction(33,'r');
			
			instruction(35,'F');
			instruction(35,'L');
			instruction(35,'l');
			instruction(35,'C');
			instruction(35,'R');
			instruction(35,'r');
			
			instruction(36,'F');
			instruction(36,'L');
			instruction(36,'l');
			instruction(36,'P');
			instruction(36,'L');
			instruction(36,'l');
			
			instruction(39,'F');
			instruction(39,'L');
			instruction(39,'l');
			
			instruction(40,'F');
			instruction(40,'C');
			instruction(40,'L');
			instruction(40,'l');
			
			instruction(41,'F');
			instruction(41,'R');
			instruction(41,'r');
			instruction(41,'P');
			instruction(41,'R');
			instruction(41,'r');
			
			instruction(42,'F');
			instruction(42,'R');
			instruction(42,'r');
			
			instruction(43,'F');
			instruction(43,'C');	
			instruction(43,'L');
			instruction(43,'l');
			
			instruction(44,'F');
			instruction(44,'L');
			instruction(44,'l');
			instruction(44,'P');
			instruction(44,'L');
			instruction(44,'l');
			
			instruction(45,'F');
			instruction(45,'R');
			instruction(45,'r');
			
			instruction(46,'F');
			instruction(46,'L');
			instruction(46,'l');
			
			instruction(48,'F');
			instruction(48,'R');
			instruction(48,'r');
			instruction(48,'C');
			instruction(48,'L');
			instruction(48,'l');
			
			instruction(49,'F');
			instruction(49,'L');
			instruction(49,'l');
			instruction(49,'P');
			instruction(49,'L');
			instruction(49,'l');
			
			instruction(52,'F');
			instruction(52,'R');
			instruction(52,'R');
			
			instruction(53,'F');
			instruction(53,'C');
			instruction(53,'R');
			instruction(53,'R');
			
			instruction(54,'F');
			instruction(54,'R');
			instruction(54,'r');
			instruction(54,'P');
			instruction(54,'R');
			instruction(54,'r');
			
			instruction(55,'F');
			instruction(55,'L');
			instruction(55,'l');
			
			instruction(56,'F');
			instruction(56,'L');
			instruction(56,'l');
			
			instruction(58,'F');
			instruction(58,'R');
			instruction(58,'r');
			instruction(58,'C');
			
	} 
	
	
  }
 }
 
  
 ISR(INT0_vect)
 {
	 Shaft_count++;
 }
 