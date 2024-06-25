//2022 ROBERTO OLIVEIRA frobertoliveira@gmail.com
//FIRMWARE V1 SOFTSTARTER TRIFASICO 220V e neutro ARDUINO NANO

// Include library

#include <TimerOne.h>
#include <MsTimer2.h>
#include <DFRobot_LCD.h>                // LCD SIMULAÇÃO
#include <LiquidCrystal_Software_I2C.h> // LCD REAL
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#define enderecoI2CLCDRobot 0x3E //endereço i2c do LCD simulação 0x3E decimal 62 LCD REAL 0x27 decimal 39
#define enderecoI2CLCDliquid 0x27 //endereço i2c do LCD simulação 0x3E decimal 62 LCD REAL 0x27 decimal 39
#define colunasLCD     16   //numero de colunas do LCD
#define linhasLCD      2    //numero de linhas LCD
#define SDA            A4   // SDA i2c arduino nano
#define SCL            A5   // SCL i2c arduino nano

// LCD REAL
// Usage: LiquidCrystal_I2C lcd(ADDRESS, COLUMNS, ROWS, SDA_PIN, SCL_PIN);
//LiquidCrystal_I2C lcd( enderecoI2CLCDliquid, colunasLCD, linhasLCD, SDA, SCL ); 
// Just SDA and SCL pins are needed to find the addresses

// LCD SIMULAÇÃO
DFRobot_LCD lcd(colunasLCD ,linhasLCD, enderecoI2CLCDRobot, 0);//lcd(colunasLCD,linhasLCD,endereço, rgb);

#define BNTstart   7 //entrada digital Botao start
#define BNTstop    6 //entrada digital Botao stop
#define BNTcima 5 //entrada digital Botao aumenta
#define BNTmenu    4 //entrada digital Botao menu

#define zeroR 8 //entrada digital detecta passagem por zero na fase R 
#define zeroS 9 //entrada digital detecta passagem por zero na fase S 
#define zeroT 10 //entrada digital detecta passagem por zero na fase T

#define TriacU A0  //saida digital aciona triac da fase U PORTB0
#define TriacV A1  //saida digital aciona triac da fase V PORTB1
#define TriacW A2 //saida digital aciona triac da fase W PORTB2

#define Rele A3 //saida digital aciona rele de bypass PORTB3

#define TEMPOMAXATRASO 70 // constante de tempo maximo de atraso do disparo do triac 40 vezes 100 microsegundos
#define TEMPOMINATRASO 5// constante de tempo minimo de atraso do disparo do triac 5 vezes 100 microsegundos

#define TEMPOMAXRAMPASUBIDA 10
#define TEMPOMINRAMPASUBIDA 2

#define TEMPOMAXRAMPADESCIDA 10
#define TEMPOMINRAMPADESCIDA 2

#define TEMPOPULSOTRIAC 2

volatile byte last_PINB = 0;

volatile bool lastZeroR = 0; //valor anterior da entrada digital do zerocrossing em cada fase
volatile bool lastZeroS = 0;
volatile bool lastZeroT = 0;

volatile bool habilitaAtrasoU = 0; //libera contagem do tempo de atraso p cada fase
volatile bool habilitaAtrasoV = 0;
volatile bool habilitaAtrasoW = 0;

volatile byte contTempoAtrasoU = 0; //contador do tempo de atraso p cada fase
volatile byte contTempoAtrasoV = 0;
volatile byte contTempoAtrasoW = 0;

volatile byte setpointTempoAtrasoU = 0; //setpoint do tempo de atraso p cada fase MIN 5 MAX 40 VEZES 100 MICROSEGUNDOS
volatile byte setpointTempoAtrasoV = 0;
volatile byte setpointTempoAtrasoW = 0;

volatile bool habilitaPulsoU = 0; // libera contagem do tempo de gatilho de cada triac
volatile bool habilitaPulsoV = 0;
volatile bool habilitaPulsoW = 0;

volatile byte contTempoPulsoU = 0; //contador do tempo de gatilho p cada fase
volatile byte contTempoPulsoV = 0;
volatile byte contTempoPulsoW = 0;

volatile byte setpointTempoPulsoU = 2; //setpoint do tempo de gatilho p cada fase
volatile byte setpointTempoPulsoV = 2;
volatile byte setpointTempoPulsoW = 2;

volatile bool habilitaRampaSubida   = 0;
volatile bool habilitaRampaDescida  = 0;

volatile unsigned int tempoRampaSubida  = 0;
volatile unsigned int tempoRampaDescida = TEMPOMAXRAMPADESCIDA;

volatile byte contRampaSubida  = 10;
volatile byte contRampaDescida = 10;

byte tela_atual = 0 ,tela_anterior = 0;

byte posicao_cursor = 0;

bool botao_start = 1;
bool botao_stop = 1;
bool botao_cima = 1;
bool botao_menu = 1;

bool estado_anterior_start = 1,estado_anterior_stop = 1, estado_anterior_cima = 1, estado_anterior_menu = 1;
bool estado_atual_start = 1, estado_atual_stop = 1, estado_atual_cima = 1, estado_atual_menu = 1;
bool habilita_leitura_start = 0, habilita_leitura_stop = 0, habilita_leitura_cima = 0, habilita_leitura_menu = 0;
bool repeticao_cima = 0;
bool cronometro_start = 0, cronometro_stop = 0, cronometro_cima = 0;

unsigned long tempo_debounce_start = 0UL, tempo_debounce_stop = 0UL, tempo_debounce_cima = 0UL;
unsigned long tempo_repeticao_cima = 0UL;
unsigned long tempo_estabiliza_start = 0UL, tempo_estabiliza_stop = 0UL, tempo_estabiliza_cima = 0UL, tempo_estabiliza_menu = 0UL;

unsigned long tempo_reflesh_display = 0UL;

unsigned long tempo_transcorrido = 0UL;

byte hora_dezena = 0;
byte hora_unidade = 0;
byte horas = 0;
byte minuto_dezena = 0;
byte minuto_unidade = 0;
byte minutos = 0;
byte segundo_dezena = 0;
byte segundo_unidade = 0;
byte segundos = 0;


byte rampaSubidaUnidade = 0;
byte rampaDescidaUnidade = 0;

int rampaSubida_display  = 0;
int rampaDescida_display = 0;

unsigned int pulsos_entrada = 0;

unsigned long periodo_saida = 0UL;

const float RAZAO_RPM = 0.00, DIVISOES_ENCODER = 60.00;

float porcentagem_rpm = 0.00;

//*********************************************************************************************
void LigaTriacU()
{
  bitSet( PORTC, 0 );
  //PORTB = PORTC | 1 << 0;//BITWISE
}
void DesligaTriacU()
{
  bitClear( PORTC, 0 );
  //PORTB = ( PORTC & ~( 1 << 0 ) );//BITWISE
}

void LigaTriacV()
{
  bitSet( PORTC, 1 );
  //PORTB = PORTC | 1 << 1;//BITWISE
}
void DesligaTriacV()
{
  bitClear( PORTC, 1 );
  //PORTB = ( PORTC & ~( 1 << 1 ) );//BITWISE
}

void LigaTriacW()
{
  bitSet( PORTC, 2 );
  //PORTB = PORTC | 1 << 2;//BITWISE
}
void DesligaTriacW()
{
  bitClear( PORTC, 2 );
  //PORTB = ( PORTC & ~( 1 << 2 ) );//BITWISE
}

void LigaRele()
{
  bitSet( PORTC, 3 );
  //PORTB = PORTC | 1 << 3;//BITWISE
}
void DesligaRele()
{
  bitClear( PORTC, 3 );
  //PORTB = ( PORTC & ~( 1 << 3 ) );//BITWISE
}

//****************************************************************************************************
bool LeBTNstart()
{
  return ( PIND&(1<<7) );//digitalRead(START);
}

bool LeBTNstop()
{
  return ( PIND&(1<<6) );//digitalRead(BTNstop);
}

bool LeBTNaumenta()
{
  return ( PIND&(1<<5) );//digitalRead(BTNaumenta);
}

bool LeBTNmenu()
{
  return ( PIND&(1<<4) );//digitalRead(BTNmenu);
}

bool LeZeroR()
{
  return bitRead( PORTB, 0 );//( PINB&(1<<0) );//bitRead( PORTB, 0 );
}

bool LeZeroS()
{
  return bitRead( PORTB, 1 );//( PINB&(1<<1) );//bitRead( PORTB, 1 );
}

bool LeZeroT()
{
  return bitRead( PORTB, 2 );//( PINB&(1<<2) );//bitRead( PORTB, 2 );
}

//****************************************************************************************************
void TesteSaidas()
{
  LigaTriacU();
  delay(1000);
  LigaTriacV();
  delay(1000);
  LigaTriacW();
  delay(1000);
  LigaRele();
  delay(1000);

  DesligaTriacU();
  delay(1000);
  DesligaTriacV();
  delay(1000);
  DesligaTriacW();
  delay(1000);
  DesligaRele();
  delay(1000);
}

//******************************************************************************************
void TesteBotoes()
{
  if ( LeBTNstart() == 0 )
  {
    LigaTriacU();
  }
  else
  {
    DesligaTriacU();
  }

  if ( LeBTNstop() == 0 )
  {
    LigaTriacV();
  }
  else
  {
    DesligaTriacV();
  }
  if ( LeBTNaumenta() == 0 )
  {
    LigaTriacW();
  }
  else
  {
    DesligaTriacW();
  }
  if ( LeBTNmenu() == 0 )
  {
    LigaRele();
  }
  else
  {
    DesligaRele();
  }
}

//**********************************************************************************************
void TesteEntradas()
{
  if ( LeZeroR() == 0 )
  {
    LigaTriacU();
  }
  else
  {
    DesligaTriacU();
  }
  if ( LeZeroS() == 0 )
  {
    LigaTriacV();
  }
  else
  {
    DesligaTriacV();
  }
  if ( LeZeroT() == 0 )
  {
    LigaTriacW();
  }
  else
  {
    DesligaTriacW();
  }
}

//*********************************************************************************************

ISR (PCINT0_vect)  // interrupção dos pinos de zerocrossing de cada fase
{
  // code to execute
 
  uint8_t changed_bits = 0;
  changed_bits = PINB ^ last_PINB;
  last_PINB = PINB;

/*
  bool changedZeroR = 0;
  bool changedZeroS = 0;
  bool changedZeroT = 0;
  bool ZeroAUX = 0;

  
  ZeroAUX = LeZeroR();
  changedZeroR = ( ZeroAUX ^ lastZeroR );
  lastZeroR = ZeroAUX;

  ZeroAUX = LeZeroS();
  changedZeroS = ( ZeroAUX ^ lastZeroS );
  lastZeroS = ZeroAUX;

  ZeroAUX = LeZeroT();
  changedZeroT = ( ZeroAUX ^ lastZeroT );
  lastZeroT = ZeroAUX;
  
  if ( changedZeroR )
  {
      if ( lastZeroR ) 
      {        // D8 mudou de LOW para HIGH;
        contTempoAtrasoU = setpointTempoAtrasoU;
        contTempoPulsoU = setpointTempoPulsoU;
        habilitaAtrasoU = 1;
        
      }
      else 
      {        // D8 mudou de HIGH para LOW;        
      }
  }
  if ( changedZeroS )
  {
      if ( lastZeroS ) 
      {          // D9 mudou de LOW para HIGH;
        contTempoAtrasoV = setpointTempoAtrasoV;
        contTempoPulsoV = setpointTempoPulsoV;
        habilitaAtrasoV = 1;
      }
      else 
      {        // D9 mudou de HIGH para LOW;        
      }
  }
  if ( changedZeroT )
  {
      if ( lastZeroT )
      {          // D10 mudou de LOW para HIGH;  
        contTempoAtrasoW = setpointTempoAtrasoW;
        contTempoPulsoW = setpointTempoPulsoW;     
        habilitaAtrasoW = 1;
      }
      else
      {        // D10 mudou de HIGH para LOW;        
      }
  }

*/
  
  if (changed_bits & (1 << PINB0))
  {
      if (PINB & (1 << PINB0)) 
      {
        // D8 mudou de LOW para HIGH;
        contTempoAtrasoU = setpointTempoAtrasoU;
        contTempoPulsoU = setpointTempoPulsoU;
        habilitaAtrasoU = 1;

        //DesligaTriacU();
      }
      else 
      {
        // D8 mudou de HIGH para LOW;
      }
  }
  if (changed_bits & (1 << PINB1))
  {
      if (PINB & (1 << PINB1)) 
      {
          // D9 mudou de LOW para HIGH;
        contTempoAtrasoV = setpointTempoAtrasoV;
        contTempoPulsoV = setpointTempoPulsoV;
        habilitaAtrasoV = 1;

        //DesligaTriacV();
      }
      else 
      {
        // D9 mudou de HIGH para LOW;
      }
  }
  if (changed_bits & (1 << PINB2))
  {
      if (PINB & (1 << PINB2))
      {
          // D10 mudou de LOW para HIGH;
        contTempoAtrasoW = setpointTempoAtrasoW;
        contTempoPulsoW = setpointTempoPulsoW;     
        habilitaAtrasoW = 1;
      }
      else 
      {
        // D10 mudou de HIGH para LOW;
      }
  }  
}

//*************************************************************************************

void InterrupcaoTimer1()
{
  if ( habilitaPulsoU )
  {    
    //LigaTriacU();
    
    if ( contTempoPulsoU > 0 )
    {
      contTempoPulsoU--;      
    }
    else 
    {      
      DesligaTriacU();
       
      habilitaPulsoU = 0;
    }
  }
  if ( habilitaAtrasoU )
  {
    if ( contTempoAtrasoU > 0 )
    {
      contTempoAtrasoU--;      
    }
    else
    {
      habilitaAtrasoU = 0;               
      habilitaPulsoU = 1;
    
      LigaTriacU();
    }
  }
  
//#############################################
  if ( habilitaPulsoV )
  {
    //LigaTriacU();
    if ( contTempoPulsoV > 0 )
    {
      contTempoPulsoV--;              
    }
    else
    {
      DesligaTriacV();  
          
      habilitaPulsoV = 0;
    }
  }
  
  if ( habilitaAtrasoV )
  {
    if ( contTempoAtrasoV > 0 )
    {
      contTempoAtrasoV--;      
    }
    else
    {
      habilitaAtrasoV = 0;               
      habilitaPulsoV = 1;
    
      LigaTriacV();
    }
  }
  
//#############################################
  if ( habilitaPulsoW )
  {
    if ( contTempoPulsoW > 0 )
    {
      contTempoPulsoW--;              
    }
    else
    {
      DesligaTriacW();      
      habilitaPulsoW = 0;
    }
  }
  
  if ( habilitaAtrasoW )
  {        
    if ( contTempoAtrasoW > 0 )
    {
      contTempoAtrasoW--;      
    }
    else
    {
      habilitaAtrasoW = 0;               
      habilitaPulsoW = 1;
    
      LigaTriacW();
    }
  }
  
}

//****************************************************************************************
//ISR(TIMER1_COMPA_vect)
//{
 // flagRele = !flagRele;
  //digitalWrite( Rele, flagRele );
//}
//******************************************************************************************

void RampaSubida( byte tempo )
{
  unsigned int auxTempo = 0;
    
  auxTempo = tempo*1000/( TEMPOMAXATRASO - TEMPOMINATRASO );
  //Serial.println(auxTempo);
  
  habilitaRampaDescida = 0;
  habilitaRampaSubida  = 1;

  setpointTempoAtrasoU = TEMPOMAXATRASO;
  setpointTempoAtrasoV = TEMPOMAXATRASO;
  setpointTempoAtrasoW = TEMPOMAXATRASO;

  contRampaSubida = TEMPOMAXATRASO;

  DesligaRele();
  DesligaTriacU();
  DesligaTriacV();
  DesligaTriacW();

  MsTimer2::set( auxTempo, InterrupcaoTimer2 ); // ms period
  MsTimer2::start();
  Timer1.start();
}

void RampaDescida( byte tempo )
{
  byte auxTempo = 0;
  auxTempo = tempo*1000/(TEMPOMAXATRASO - TEMPOMINATRASO );
  //Serial.println(auxTempo);
  
  habilitaRampaSubida  = 0;
  habilitaRampaDescida = 1;

  setpointTempoAtrasoU = TEMPOMINATRASO;
  setpointTempoAtrasoV = TEMPOMINATRASO;
  setpointTempoAtrasoW = TEMPOMINATRASO;

  contRampaDescida = TEMPOMINATRASO;

  LigaTriacU();
  LigaTriacV();
  LigaTriacW();
      
  MsTimer2::set( auxTempo, InterrupcaoTimer2 ); // ms period
  MsTimer2::start();
  Timer1.start();

  DesligaRele();
}

void InterrupcaoTimer2()
{   
   if ( habilitaRampaSubida )
   {      
/*      if ( setpointTempoAtrasoU > TEMPOMINATRASO)
      {
        setpointTempoAtrasoU--;
      }
      if ( setpointTempoAtrasoV > TEMPOMINATRASO)
      {
        setpointTempoAtrasoV--;
      }
      if ( setpointTempoAtrasoW > TEMPOMINATRASO)
      {
        setpointTempoAtrasoW--;
      }
      contRampaSubida--;
*/    if ( contRampaSubida > TEMPOMINATRASO )
      {
        contRampaSubida--;
        setpointTempoAtrasoU--;
        setpointTempoAtrasoV--;
        setpointTempoAtrasoW--;        
      }
      else
      {
        habilitaRampaSubida = 0;
        
        LigaRele();
        
        unsigned int aux1 = 0;
        unsigned int aux2 = 0;
        while ( aux1 > 60000 )
        {
          aux1++;
          while ( aux2 > 60000 )
          {
            aux2++;
          }
        }
                
        Timer1.stop();
        MsTimer2::stop();
        
        DesligaTriacU();
        DesligaTriacV();
        DesligaTriacW();
      }     
   }   
   
   if ( habilitaRampaDescida )
   {     
/*      
      if ( setpointTempoAtrasoU < TEMPOMAXATRASO )
      {
        setpointTempoAtrasoU++;
      }
      if ( setpointTempoAtrasoV < TEMPOMAXATRASO )
      {
        setpointTempoAtrasoV++;
      }
      if ( setpointTempoAtrasoW < TEMPOMAXATRASO )
      {
        setpointTempoAtrasoW++;
      }
*/      
      if ( contRampaDescida < TEMPOMAXATRASO )
      {
        contRampaDescida++;

        setpointTempoAtrasoU++;
        setpointTempoAtrasoV++;
        setpointTempoAtrasoW++;
        
      }
      else
      {      
        MsTimer2::stop();
        Timer1.stop();
        
        DesligaTriacU();
        DesligaTriacV();
        DesligaTriacW();
        
        habilitaRampaDescida = 0;
      }        
   }   
}

void Tela0()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("   Softstarter");

  lcd.setCursor(0, 1);            // selecionando coluna 0 e linha 1
  lcd.print( "   Versao V1.0" );       // Print da mensagem
}

void Tela1()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print(" INICIAR/CONFIG.");

  lcd.setCursor(0, 1);
  lcd.print( "MEN CIMA STO STA" );

}

void Tela2()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("  RAMPA SUBIDA");

  lcd.setCursor(14, 1);
  lcd.print(rampaSubidaUnidade);

  lcd.setCursor(15, 1);
  lcd.print( "s" );

  posicao_cursor = 14;
}

void Tela3()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("  RAMPA DESCIDA");

  lcd.setCursor(14, 1);
  lcd.print(rampaDescidaUnidade);

  lcd.setCursor(15, 1);
  lcd.print( "s" );

  posicao_cursor = 14;
}

void Tela4()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("   ACELERANDO");
}

void Tela5()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("     BYPASS");

  lcd.setCursor(0, 1);
  lcd.print("    COMPLETO");
}

void Tela6()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("  DESACELERANDO");
}

void Tela7()
{
  lcd.noCursor();

  lcd.clear();                    // limpa tela

  lcd.setCursor(0, 0);
  lcd.print("    FRENAGEM");

  lcd.setCursor(0, 1);
  lcd.print("    COMPLETA");
}

void LeEntradas()
{
  
}
void AcaoMenu()
{
  
}

void AtualizaMenu()
{
  switch (tela_atual)
  {
    case 0:
      Tela0();
      break;
    case 1:
      Tela1();
      break;
    case 2:
      Tela2();
      break;
    case 3:
      Tela3();
      break;
    case 4:
      Tela4();
      break;
    case 5:
      Tela5();
      break;
    case 6:
      Tela6();
      break;
    case 7:
      Tela7();
      break;
  }
}

void RefleshDisplay()
{
  if ( millis() - tempo_reflesh_display >= 500 )
  {
    tempo_reflesh_display = millis();
    AtualizaMenu();
  }
}

void setup() {
  // put your setup code here, to run once:
  
  pinMode( BNTstart,   INPUT_PULLUP );
  pinMode( BNTstop,    INPUT_PULLUP );
  pinMode( BNTcima, INPUT_PULLUP );
  pinMode( BNTmenu,    INPUT_PULLUP );

  pinMode( zeroR, INPUT_PULLUP );
  pinMode( zeroS, INPUT_PULLUP );
  pinMode( zeroT, INPUT_PULLUP );

  pinMode( TriacU, OUTPUT );
  pinMode( TriacV, OUTPUT );
  pinMode( TriacW, OUTPUT );
  pinMode( Rele,   OUTPUT );

  DesligaTriacU();
  DesligaTriacV();
  DesligaTriacW();
  DesligaRele();

  //Serial.begin(9600);
  
  Timer1.initialize( 100*2 );// inicializa timer1
  Timer1.attachInterrupt( InterrupcaoTimer1 ); // habilita interrupção do timer1
  Timer1.stop();  // liga o timer1

  lastZeroR = LeZeroR();
  lastZeroS = LeZeroS();
  lastZeroT = LeZeroT();
  last_PINB = PINB;

  PCICR |= B00000001; // We activate the interrupts of the PB port
  PCMSK0 |= B00000111; //We activate the interrupts on pin D10,D9,D8  

  //PCICR |= (1 << PCIE0);
  //PCMSK0 |= (1 << PCINT0);  

  // Configuração do TIMER1
  //TCCR1A = 0;                //confira timer para operação normal
  //TCCR1B = 0;                //limpa registrador
  //TCNT1  = 0;                //zera temporizado
 
  //OCR1A = 0x3D09;            // carrega registrador de comparação: 16MHz/1024/1Hz = 15625 = 0X3D09
  //TCCR1B |= (1 << WGM12)|(1<<CS10)|(1 << CS12);   // modo CTC, prescaler de 1024: CS12 = 1 e CS10 = 1  
  //TIMSK1 |= (1 << OCIE1A);  // habilita interrupção por igualdade de comparação  
  
  // Configure Timer 1 interrupt
  // F_clock = 16e6 Hz, prescaler = 64, Fs = 100 Hz
  //TCCR1A = 0;
  //TCCR1B = 1<<WGM12 | 0<<CS12 | 1<<CS11 | 1<<CS10;
  //TCNT1 = 0;          // reset Timer 1 counter
  // OCR1A = ((F_clock / prescaler) / Fs) - 1 = 2499
  //OCR1A = 2499;       // Set sampling frequency Fs = 100 Hz
  //TIMSK1 = 1<<OCIE1A; // Enable Timer 1 interrupt
  
  //sei();

  setpointTempoPulsoU = TEMPOPULSOTRIAC;
  setpointTempoPulsoV = TEMPOPULSOTRIAC;
  setpointTempoPulsoW = TEMPOPULSOTRIAC;

  habilita_leitura_start = 1;
  habilita_leitura_cima  = 1;
  habilita_leitura_stop  = 1;
  habilita_leitura_menu  = 1;

  lcd.init(); // LCD initialization
  lcd.display();
//  lcd.backlight();
 
  tela_atual = 0;

  AtualizaMenu();  

  tela_atual = 1;
  tela_anterior = 0;

  tempo_reflesh_display = millis();
/*
  setpointTempoAtrasoU = TEMPOMAXATRASO -10;
  setpointTempoAtrasoV = TEMPOMAXATRASO -10;
  setpointTempoAtrasoW = TEMPOMAXATRASO -10;

  Timer1.start();
  delay(2000);

  Timer1.stop();

  setpointTempoAtrasoU = TEMPOMAXATRASO -20;
  setpointTempoAtrasoV = TEMPOMAXATRASO -20;
  setpointTempoAtrasoW = TEMPOMAXATRASO -20;

  Timer1.start();
  delay(2000);

  Timer1.stop();

  setpointTempoAtrasoU = TEMPOMAXATRASO -30;
  setpointTempoAtrasoV = TEMPOMAXATRASO -30;
  setpointTempoAtrasoW = TEMPOMAXATRASO -30;

  Timer1.start();
  delay(2000);

  Timer1.stop();

  setpointTempoAtrasoU = TEMPOMAXATRASO -40;
  setpointTempoAtrasoV = TEMPOMAXATRASO -40;
  setpointTempoAtrasoW = TEMPOMAXATRASO -40;

  Timer1.start();
  delay(2000);

  Timer1.stop();

  setpointTempoAtrasoU = TEMPOMAXATRASO -50;
  setpointTempoAtrasoV = TEMPOMAXATRASO -50;
  setpointTempoAtrasoW = TEMPOMAXATRASO -50;

  Timer1.start();
  delay(2000);

  Timer1.stop();

  setpointTempoAtrasoU = TEMPOMAXATRASO -60;
  setpointTempoAtrasoV = TEMPOMAXATRASO -60;
  setpointTempoAtrasoW = TEMPOMAXATRASO -60;

  Timer1.start();
  delay(5000);

  Timer1.stop();

  DesligaTriacU();
  DesligaTriacV();
  DesligaTriacW();

*/

  digitalWrite(13, HIGH);
 // RampaSubida( 5 );
}

void loop() {
  // put your main code here, to run repeatedly:
  
  LeEntradas();
  AcaoMenu();
  RefleshDisplay();
  
  RampaSubida( 7 );
  delay(10000);

  RampaDescida( 5 );
  delay(10000);

  //TesteSaidas();
  //TesteEntradas();
  //TesteBotoes();
  //TesteLCD();
  
  
  //digitalWrite(A3, HIGH);//LigaRele();
  //delay(100);
  //digitalWrite(A3, LOW);//DesligaRele();
  //delay(100);
  
}

//Serial.print(resultArray[i], HEX);                // Print hexadecimal value
//Serial.println("Valid addresses:");

//#include "DFRobot_LCD.h"
//DFRobot_LCD lcd(16,2);  //16 characters and 2 lines of show

//lcd.display(); // Turn on the display:
//lcd.noDisplay();// Turn off the display:

//lcd.rightToLeft(); go right for the next letter
//lcd.leftToRight(); // go left for the next letter
//lcd.home(); // start again at 0

//lcd.setCursor(0, 1);linha coluna
//lcd.cursor(); // turn on the cursor:
//lcd.noCursor(); // Turn off the cursor:
//lcd.blink(); // Turn on the blinking cursor:
//lcd.stopBlink();// Turn off the blinking cursor:

//lcd.autoscroll(); // set the show to automatically scroll:
//lcd.noAutoscroll(); // turn off automatic scrolling
//lcd.scrollDisplayRight(); scroll one position right:

//lcd.print("Hello, world!");
//lcd.write(thisLetter);

//lcd.customSymbol(0, heart); // create a new character
/*
 * byte heart[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000
}; */
        
