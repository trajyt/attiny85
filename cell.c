// начало адресации 10 (0x0a)
//ДЛЯ МАРКИРОВАННЫХ: ПЕРЕД КАЖДОЙ ПРОШИВКОЙ СТИРАТЬ КАМЕНЬ
//fuse d7 ec
#include <tiny44a.h>
#include <interrupt.h>
#include <eeprom.h>
#include <delay.h>
#include <stdio.h>
//#include <math.h>
#include <stdint.h>
#include <sleep.h>


const unsigned char ID = 14;

typedef struct
{
  uint8_t CRC_OK: 1;
  uint8_t transmit_ok : 1;
  uint8_t receive_ok : 1;
  uint8_t ADC_EN: 1;
  uint8_t sleep_en: 1;
  //uint8_t write_ee: 1;

} BIT;


register BIT flags         @2;
unsigned char bitcount        @3;
unsigned int  byte          @4;
unsigned char  temperature   @6;
unsigned char  voltage         @7;
//           @8;
unsigned char Vmax           @10;
unsigned char sec             @11;
unsigned int k               @12;

volatile unsigned long  av;
unsigned int  time;
//volatile unsigned char transmit_ok = 0;
//volatile unsigned char receive_ok  = 0;
//volatile unsigned char ADC_EN = 0;
//volatile unsigned char CRC_OK = 0;
//volatile unsigned char sleep_en = 0;

volatile unsigned char c[3];

unsigned int k1;
unsigned char k2;
volatile unsigned char remain; //
volatile unsigned char calib = 0;
unsigned int v_real = 0;



//для 23 ячейки порт ADC_1 выгорел используется порт_3
//#define ADC_1   (1<<REFS1) | (0<<REFS0) | (0<<MUX5) | (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (1<<MUX0)

#define ADC_1   (1<<REFS1) | (0<<REFS0) | (0<<MUX5) | (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (1<<MUX0)
#define ADC_2  0b00000010
//#define ADC_8  0b10100010    // внутренния температура
#define VOLTAGE               1
#define TEMPERATURE           2


#define STOP_PWM    TCCR0A&=~(1<<COM0B1)
#define START_PWM   TCCR0A|= (1<<COM0B1) //|(1<<COM0B0)



/*
*  Ниже задаются константы, определяющие скорость передачи данных (бодрейт)
*  расчет BAUD_DIV осуществляется следующим образом:
*  BAUD_DIV = (CPU_CLOCK / DIV) / BAUD_RATE
*  где CPU_CLOCK - тактовая частота контроллера, BAUD_RATE - желаемая скорость UART,
*  а DIV - значение делителя частоты таймера, задающееся в регистре TCCR0B.
*  Например, делитель на 8, скорость порта 9600 бод:
*  BAUD_DIV = (4 000 000 / 1) / 9600 = 415 .
    BAUD_DIV = (4 000 000 / 8) / 9600 = 52
*/

//#define T_DIV    0x01  // DIV = 1
#define T_DIV    0x02  // DIV = 8
//#define T_DIV    0x03  // DIV = 64
#define BAUD_DIV  51//  Скорость = 9600 бод, ДОПОЛНИТЕЛЬНО КОРРЕКТИРУЕТСЯ



unsigned int ADC_read(unsigned char value);
void uart_send(unsigned char u);



void main(void)
{


 // Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif


  MCUCR = (0<<ISC01)|(1<<ISC00);

  DDRA|=(1<<DDA7);//шим Т0 РА7
  //DDRA|=(1<<0);////для отладки

  //DDRA |= (1 << 6);
  //PORTA|= (1 << 6);

/*
  DDRA|= (1 << 0);
  DDRA|= (1 << 3);
  DDRA|= (1 << 4);
  DDRA|= (1 << 5);
  DDRB|= (1 << 0);
  DDRB|= (1 << 1);
  */


// Timer/Counter 1 initialization
  OCR1A = BAUD_DIV;      // Задаем значение регистра OCR0A в соответствии с бодрейтом
  TCCR1B = T_DIV;// Задаем скорость счета таймера в соответствии с делителем
  TCCR1B |= (1 << WGM12);    // Режим таймера CTC (очистка TCNT0 по достижению OCR0A)

// Timer/Counter 1 Interrupt(s) initialization
  //TIMSK1 |= (1 << OCIE1A); //разрешаем прерывания Т1

// External Interrupt(s) initialization
// Interrupt on any change on pins PCINT9: On
  PCMSK1=(1<<PCINT10);

  GIMSK=(0<<INT0) | (1<<PCIE1) | (0<<PCIE0);
  GIFR|=(1<<PCIF1); //очищаем флаг прерывания

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 500,000 kHz
// Mode: Ph. correct PWM top=OCR0A
// OC0A output: Disconnected
// OC0B output: Non-Inverted PWM
// Timer Period: 0,5 ms
// Output Pulse(s):
// OC0B Period: 0,5 ms Width: 0,248 ms
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (1<<WGM00);
TCCR0B=(1<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x7D;
// OC0B=62  это 50%

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);


// ADC initialization
// ADC Clock frequency: 125,000 kHz
// ADC Voltage Reference: 1.1V, AREF discon.
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On, ADC6: On, ADC7: On
//ADMUX = ADC_diff;
ADCSRA=(1<<ADEN) | (1<<ADSC) | (0<<ADATE) | (1<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);
//ADCSRB|= (1<<ADLAR); //при использовании ADCH

//Analog Comparator Disable
ACSR|=(1<<7);





//процедура калибровки

//DDRA|=(1<<DDA3);
DDRA|=(1<<DDA4);
DDRA|=(1<<DDA5);
//PORTA|= (1 << 3); // ставим 1
PORTA|= (1 << 4); // ставим 1
PORTA|= (1 << 5); // ставим 1


  //  Vmax = eeprom_read_byte((uint8_t*)6);
//    delay_ms(5);
//    if(Vmax > 250) {Vmax = 250;}


if(!(PINA & (1 << 4)) && (PINA & (1 << 5)) )      //0 1 sck(-) mi(+)
 {
   v_real = 330;
   calib = 1;
   Vmax = 189;//160+,,,=349 пороги для шим
   remain = 160;  //вычитаемый остаток
   k = 650;
   eeprom_write_byte ((uint8_t *)6, Vmax);
    delay_ms(10);
   eeprom_write_byte ((uint8_t *)9, remain);
    delay_ms(10);

  }
else if( (PINA & (1 << 4)) && !(PINA & (1 << 5)))  //1 0          +
 {
   v_real = 400;
   calib = 1;
   Vmax = 235;//180+235=415
   remain = 180;
  }
else if( !(PINA & (1 << 4)) && !(PINA & (1 << 5))) //0 0      -+
 {
   v_real = 240;
   calib = 1;
   Vmax = 105;//160+100=265
   remain = 160;
  }


 if(!calib)
  {
//   unsigned char i;
//      ADMUX = ADC_1;
//   //delay_ms(10);
//   for(av=0,i=0;i<64;++i)
//   {
//    ADCSRA |= 0x40;                      // start new A/D conversion
//    while (!(ADCSRA & 0x10));        // wait until ADC is ready
//    ADCSRA |= 0x10;
//    av = av + ADCW;
//   }
   //av = av >> 7; //511
   //av = av << 2; //множим на 4
   //k = av - v_real;
   //k1 = k >> 8;
   //k2 = k % 255;//k - (k1<<8)
   //eeprom_write_byte ((uint8_t *)2, k1); //запись в 2адрес
    //delay_ms(10);
   //eeprom_write_byte ((uint8_t *)3, k2); //запись в 3адрес
   // delay_ms(10);
//   eeprom_write_byte ((uint8_t *)6, Vmax);
//    delay_ms(10);
//   eeprom_write_byte ((uint8_t *)9, remain);
//    delay_ms(10);
//  }
// else
//  {
    k1 = eeprom_read_byte((uint8_t*)2); // коэф в первом адресе
    delay_ms(10);
    k2 = eeprom_read_byte((uint8_t*)3); // коэф в первом адресе
    delay_ms(10);
    k = (k1*10)+k2;

    Vmax = eeprom_read_byte((uint8_t*)6);
    delay_ms(10);
    remain = eeprom_read_byte((uint8_t*)9);
    delay_ms(10);
  }


do{}while(!(PINA & (1 << 5)) || !(PINA & (1 << 6)) );


//PORTA&= ~(1 << 3);
//PORTA&= ~(1 << 4);
//PORTA&= ~(1 << 5);
//DDRA&=~(1<<DDA3);
//DDRA&=~(1<<DDA4);
//DDRA&=~(1<<DDA5);
//START_PWM;
//OCR0B=20;

SREG|= (1<<7);


delay_ms(50);
voltage = ADC_read(VOLTAGE);
temperature = ADC_read(TEMPERATURE);

  //while(1)
  for(;;)

   {

               //START_PWM;
        //OCR0B=62;
      if(flags.ADC_EN)//((flag & (1<<ADC_EN)))
      {
           //PORTA|=(1<<2); //для отладки
         STOP_PWM;
         //delay_ms(5);
		 voltage = ADC_read(VOLTAGE);

         temperature = ADC_read(TEMPERATURE);
         ///START_PWM;

        if(voltage < Vmax) {if(OCR0B>3) OCR0B-=3;}

	    if(voltage > Vmax+1) {if(OCR0B<100) OCR0B+=2; }

		if(OCR0B<5) STOP_PWM;
		else START_PWM;


       //PORTA&=~(1<<2); //для отладки
       flags.ADC_EN = 0;//flag &= ~(1<<ADC_EN);///
       GIMSK |= (1<<PCIE1);//  разрешаем прерывание
       GIFR |= (1<<PCIF1); //очищаем флаг прерывания

      }


     if(flags.CRC_OK)
       {
      //_delay_us(100); //чтоб плата успевала прием данных
          uart_send(ID);
           while (flags.transmit_ok);  //_delay_us(900);  // 800 это минимум с учетом всех задержек
          uart_send(voltage);
           while (flags.transmit_ok);  //_delay_us(900);
          uart_send(voltage);
           while (flags.transmit_ok);  //_delay_us(900);
          uart_send(temperature);
           while (flags.transmit_ok);  //_delay_us(900);
          uart_send(temperature);
           while (flags.transmit_ok);  //_delay_us(900);
          flags.CRC_OK = 0;
         GIMSK |= (1<<PCIE1);//
         GIFR |= (1<<PCIF1); //очищаем флаг прерывания//

        //flags.ADC_EN = 1;

      }

      if(flags.sleep_en)
        {

         MCUCR|=(1<<SM1)|(0<<SM0);  //Power-Down Mode

         GIFR |= (1<<INTF0);
         GIMSK |= (1<<INT0);
         sleep_enable();
        /* Enter sleep mode */
         #asm("sleep");//после этой команды уходим в сон, просыпаемся на этом месте
         sleep_disable();
        flags.sleep_en = 0;
      }

   }


}





interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{

 //PORTA|=(1<<0); //для отладки


 if(calib)
  {
    static unsigned int time1 = 0;
    time1++;

    if(!(PINA & (1 << 4))) //sck(4) -
     { time1 = 0;
      // DDRC |= (1<<5); //LED вкл
      delay_ms(20);
      // DDRC &= ~(1<<5); //LED выкл
      if(!(PINA & (1 << 4))) //sck(4) -
         {
          while(!(PINA & (1 << 4)));//ожидание отжатия кнопки
           k++;
         }
      }
    else if(!(PINA & (1 << 5))) // mi +
     {  time1 = 0;
      // DDRC |= (1<<5); //LED вкл
       delay_ms(20);
      // DDRC &= ~(1<<5); //LED выкл
       if(!(PINA & (1 << 5))) //  +
         {
          while(!(PINA & (1 << 5)));//ожидание отжатия кнопки
           k--;
         }
      }
    //else if(!(PINA & (1 << 3))) //  011  сохраняем
    else if(time1 > 65500) //30cek
     {
      //delay_ms(10);
      //if(!(PINA & (1 << 3))) //
        // {
        // while(!(PINA & (1 << 3)));//ожидание отжатия кнопки
         k1 = k /10;
         k2 = k % 10;
         eeprom_write_byte ((uint8_t *)2, k1); //запись в 2адрес
          delay_ms(10);
         eeprom_write_byte ((uint8_t *)3, k2); //запись в 3адрес
          //delay_ms(10);
         //eeprom_write_byte ((uint8_t *)6, Vmax);
         // delay_ms(50);
         //eeprom_write_byte ((uint8_t *)9, remain);
         // delay_ms(50);
         calib = 0;
       //}
      }

  }


 time++;


   if(time>20000) // на случай прекращения запросов,включаем ацп через 10 сек
    {
     sec++;
     time = 0;
     flags.ADC_EN = 1;//flag |= (1<<ADC_EN); ///

    if(sec>30) //больше 5 мин никто не спрашивает, спать
    {
     sec = 0;
     if(voltage < Vmax)  { STOP_PWM;  flags.sleep_en = 1; }
    }


  }

//PORTA&=~(1<<0); //для отладки


}

//для пробуждения из сна
interrupt [EXT_INT0] void ext_int0_isr(void)
{
//MCUCR&=~(1<<SM1)|(1<<SM0);
GIMSK &= ~(1<<INT0);

}


////////////////ДАЛЕЕ ОТНОСИТСЯ К ЮАРТ//////////////////////////

//Прерывание PCINT.
//Срабатывает по любому фронту!!! испульса на входе PCINT9, используется для отслеживания начала приема информации.


interrupt [PC_INT1] void pin_change_isr0(void)
{
    GIMSK &= ~(1<<PCIE1);          // Запрещаем прерывание

    sec = 0;
    delay_us(40); //10*4 -40мкс задержка 1сек/9600бод/2 = 52мкс
    //TIMSK0 &=~(1<<TOIE0);
      byte = 0;

    flags.receive_ok = 1;//flag |= (1<<RX_EN);
      bitcount = 9;

    TCNT1=0; //корректировка счетчика
    TIMSK1|= (1 << OCIE1A); //разрешаем прерывания Т1
}


/*
*  Прерывание таймера по сравнению с регистром OCR1A.
*  Это прерывание используется для отправки и приема данных. 1 стартовый, 8 бит данных и 1 стоповый, итого 10
*/

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
//PORTA|=(1<<0); //для отладки

 if(bitcount > 0)
  {
    bitcount--;

    if(flags.receive_ok) // прием  (flag & (1<<RX_EN))
    {

     if(PINB & (1 << 2)) // Проверяем в каком состоянии вход RXD
      {
        byte |= (1 << (8-bitcount)); // записываем биты 0-7 в число поразрядно согласно счетчику
      }

     if(!bitcount)
      {
         byte >>= 1; //сдвигаем вправо ( убираем бит) убираем стартовый бит
         //rx_byte = rx_byte|0x01; // инверсия бита если нужно
         flags.receive_ok = 0; //flag &= ~(1<<RX_EN);
         TIMSK1 &= ~(1 << OCIE1A); //запрещаем прерывания Т1

        // c[0] = c[1]; //ид
        // c[1] = c[2]; //код
         c[0] = byte; //напр

       if(c[0] == ID)//&&(c[1] == 150))  //если запросы на другую плату и данные отправлены,
       {

        flags.CRC_OK = 1;
//        c[0] = 0;
//        c[1] = 0;
//        c[2] = 0;

        return;

       }
//       else if((c[0] == ID)&&(c[1] == 200))
//       {
//         flags.write_ee = 1;
//              c[0] = 0;
//              c[1] = 0;
//              v_real = c[2];
//              c[2] = 0;
//
//         return;
//       }
       else
       {
        flags.ADC_EN = 1;//   flag |= (1<<ADC_EN);  //           можно и ацп обработать
       }



      }

     //PORTA&=~(1<<2); //для отладки
    }


    else if(flags.transmit_ok)//передача
    {
     //PORTB|=(1<<2); //для отладки
     if(byte & 1)  //проверям бит 0 или 1
      //{PORTA|= (1 << 6);DDRA &=~(1 << 6); }   //1
       DDRA &=~(1 << 6);
     else
      //{PORTA&= ~(1 << 6); DDRA |= (1 << 6); }   //0
       DDRA |= (1 << 6);

     byte >>= 1;    // Двигаем вправо

       if(!bitcount)
       {
        //PORTA|= (1 << 6);//DDRA &=~(1 << 6);
        TIMSK1 &= ~(1 << OCIE1A);
        flags.transmit_ok = 0;//flag &= (~(TX_EN)); //
        //TIMSK0 |= (1<<TOIE0);
       }


    }

  }
//PORTA&=~(1<<0); //для отладки

}

/*
*  Функция передачи байта по UART.
*  записывает в 8 бит переменной tx_byte байт для передачи вместе со стартовым битом, Задает счетчик бит = 10.
*   После этого они немедленно начнут передаваться.
*/

void uart_send(unsigned char u)
{
    GIMSK &= ~(1<<PCIE1);
    byte = u;
    //byte=0;
  byte <<= 1;          // сдвигаем влево, добавляем cтартовый бит
  byte |= (1 << 9); //  стоповый в 1
    bitcount = 10;          // Задаем счетчик бит, 1 стартовый 8 бит данных 1 стоповый
  flags.receive_ok = 0;//flag &= ~(1<<RX_EN);
  flags.transmit_ok = 1;
  TCNT1=0;
    TIMSK1 |= (1 << OCIE1A);
}



unsigned int ADC_read(unsigned char value)
{
  unsigned char i;

  //unsigned char ae;

  switch (value)
  {
    case 1:
        ADMUX = ADC_1;
        break;
    case 2:
        ADMUX = ADC_2;
        break;
   }


    //Calculate a average out of the next 8 A/D conversions
   for(av=0,i=0;i<64;++i)
    {
        ADCSRA |= 0x40;                      // start new A/D conversion
        while (!(ADCSRA & 0x10));        // wait until ADC is ready
        ADCSRA |= 0x10;

		   av = av + ADCW;

     }
    //av = av >> 6; //1023


   if(value == VOLTAGE)
   {
     //av = av >> 7; //511
	  av = av << 2;   //*4

//      if(flags.write_ee)
//      {
//        k = av - (v_real + remain);
//           //k1 = k >> 8;
//           //k2 = k % 255;//k - (k1<<8)
//           eeprom_write_byte ((uint8_t *)2, k); //запись в 2адрес
//            //delay_ms(10);
//           //eeprom_write_byte ((uint8_t *)3, k2); //запись в 3адрес
//            //delay_ms(10);
//           //eeprom_write_byte ((uint8_t *)6, Vmax);
//            //delay_ms(10);
//           //eeprom_write_byte ((uint8_t *)9, remain);
//            //delay_ms(10);
//         }

	  av = av/k;

	    av = av - remain; //минус 1.60в       миниально возможное 1.55в 1.60+255(0xFF) = 4.20max

   }
   else
   {
     av = av >> 8; //255

 // ввиду плавающего напряжения датчика, расчет адс_т = адс_т * t, где t поправочный коэф от напряжения,
 // соотв.чем больше напряжение тем меньше коэф
	  //av = av * ((256-ae)*3/4 + 80) ;
	  //av = av / 780;
	  //if(av>255) av=254;
   }

    ADCSRA&= (~(4));                     // clear ADC interrupt flag

    return(av);
}



/*
void Stable_ADC(void)                     // loop until you have a stable value
{
  unsigned int V[4];
  unsigned char i;
  unsigned int Vmax, Vmin;

    //Loop until the ADC value is stable. (Vmax <= (Vmin+1))
    for (Vmax=10,Vmin=0;Vmax > (Vmin+1);)
    {
        ADCSRA |= 0x40;                      // Start a new A/D conversion
        while (!(ADCSRA & (1<<4)));        // wait until ADC is ready
        V[3] = V[2];
        V[2] = V[1];
        V[1] = V[0];
        V[0] = ADCH;
        //Save the max and min voltage
        for (i=0;i<=3;i++)
        {
            if (V[i] > Vmax)
                Vmax=V[i];
            if (V[i] < Vmin)
              Vmin=V[i];
        }

        Vmin = V[0];                          // Vmin is the lower VOLTAGE
        Vmax = V[0];                          // Vmax is the higher VOLTAGE

    }
}

*/









