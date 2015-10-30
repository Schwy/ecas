/************************************************************************/
/* SIL2 squelette tp2 communications CAN  sur ARCOMGX533                */
/* sous RTAI 3.4                                                        */
/* a completer pour ajouter COM et sortie analogique voie 2             */       
/* KOCIK R.                                                             */
/*         Driver ADC fonctionnel                                       */
/*        Driver DAC fonctionnel                                        */
/************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>

//#include "3712.h"

MODULE_LICENSE("GPL");

/* define pour tache periodique */
#define STACK_SIZE      2000
#define TICK_PERIOD     1000000    //1 ms
#define PERIODE_CONTROL 10000000	 //10ms
#define N_BOUCLE         10000000
#define NUMERO             1
#define PRIORITE        1

/* define pour DAC */
#define PI 3.1415

// DRIVER ADC
#define BASE_ADC_0 0x0320
#define BASE_ADC_1 (BASE_ADC_0+1)
#define BASE_ADC_2 (BASE_ADC_0+2)
#define BASE_ADC_3 (BASE_ADC_0+3)
#define BASE_ADC_8 (BASE_ADC_0+8)

// DRIVER DAC
#define BASE_DAC_0 0x0300
#define BASE_DAC_1 (BASE_DAC_0+1)
#define BASE_DAC_2 (BASE_DAC_0+2)
#define BASE_DAC_3 (BASE_DAC_0+3)
#define BASE_DAC_8 (BASE_DAC_0+8)

#define MAX_ANGL        	0.305    //(+35° = 0.61 rad)
#define MAX_POS            	0.6
#define CHANNEL_ANGL        0
#define CHANNEL_POS       	1

// EXTREMES ARCOM
//Banc 2 à revérifier
#define BANC        3        //CHOISIR ENTRE 2,3,4,5 ou autre(défaut) *************************


#if (BANC == 2)
	#define MIDDLE		2007
	#define COEFF_ANGL		0.000376543
    #define BC_PMn		367
    #define BC_PMx		3839
    #define BC_AMn		1195
    #define BC_AMx		2823
#elif (BANC == 3)
	#define MIDDLE		2059 //56mV
	#define COEFF_ANGL	0.000361803
    #define BC_PMn		400
    #define BC_PMx		3890
    #define BC_AMn		1200
    #define BC_AMx		2825//2910
#elif (BANC == 4)
	#define MIDDLE		2007
	#define COEFF_ANGL	0.000361803
    #define BC_PMn		5
    #define BC_PMx		4095
    #define BC_AMn		1291
    #define BC_AMx		2760
#elif (BANC == 5)
	#define MIDDLE		2007
	#define COEFF_ANGL	0.000361803
    #define BC_PMn		0
    #define BC_PMx		4095
    #define BC_AMn		1235
    #define BC_AMx		2800
#elif (BANC == 0)		//VALEURS 0-4095
	#define MIDDLE		2047
	#define COEFF_ANGL	0.000361803
    #define BC_PMn		0
    #define BC_PMx		4095
    #define BC_AMn		0
    #define BC_AMx		4095
#else
	#define MIDDLE		2007
	#define COEFF_ANGL	0.000361803
    #define BC_PMn		380
    #define BC_PMx		3820
    #define BC_AMn		1200
    #define BC_AMx		2820
#endif

/* RT_TASK */
static RT_TASK tache_main;
//static int temps=0;

/*** DEFINE GLOBAL VAR ***/
float commande;
float Adc[4][4];
float Bdc[4][2];
float Cdc[4];
float x[4];
float x_new[4];
float y[2];
int line;
int h;

float vit_pos;
float vit_ang;
float pos_save;
float ang_save;

/*********PROTOTYPES DAC************/
int		init_3712           (void);
void	set_DA              (int canal, float value);
int		dac_convert_pos     (float value);

/*********PROTOTYPES ADC************/
void	init_3718           (void);
void	trigger             (void);
void	set_canal           (int canal);
void	ad_range_select     (int canal, int range);
int		adc_read_value      (void);
float	adc_convert_pos     (int value);
float	adc_convert_angle   (int value);
int		adc_read_eoc        (void);

/** PROTOTYPE AUTRE FONCTION **/
void	init_matrix         (void);
int		test_value          (float valueA, float valueB);
void	affichage_float     (float val);
void	verif_position      (int value);
void	verif_angle         (int value);

void	read_ADC			(void);
void	write_DAC			(void);
void	calc_matrix			(void);

void main_task(long arg)
{
	while(1)
	{
		read_ADC();
		read_ADC();
		//printk("pos*1000= %d", (int)(y[1][0]*1000));
		//printk(" | angl*1000= %d\t", (int)(y[0][0]*1000));
		calc_matrix();
		write_DAC();
		rt_task_wait_period();
	}
}

void calc_matrix()
{
	//data.x=data.Adc*data.x+data.Bdc*data.y;
	//data.u=-data.Cdc*data.x;
	vit_ang = ((y[0] - ang_save)/PERIODE_CONTROL)*0.0000000001;
	ang_save = y[0];
	vit_pos = ((y[1] - pos_save)/PERIODE_CONTROL)*0.0000000001;
	pos_save = y[1];
	/*
	x[0]=y[0];
	x[1]=y[1];
	x[2]=vit_pos;
	x[3]=vit_ang;
	*/
	x_new[0]= Adc[0][0]*x[0] + Adc[0][1]*x[1] + Adc[0][2]*x[2] + Adc[0][3]*x[3] + Bdc[0][0]*y[0] + Bdc[0][1]*y[1];
	x_new[1]= Adc[1][0]*x[0] + Adc[1][1]*x[1] + Adc[1][2]*x[2] + Adc[1][3]*x[3] + Bdc[1][0]*y[0] + Bdc[1][1]*y[1];
	x_new[2]= Adc[2][0]*x[0] + Adc[2][1]*x[1] + Adc[2][2]*x[2] + Adc[2][3]*x[3] + Bdc[2][0]*y[0] + Bdc[2][1]*y[1];
	x_new[3]= Adc[3][0]*x[0] + Adc[3][1]*x[1] + Adc[3][2]*x[2] + Adc[3][3]*x[3] + Bdc[3][0]*y[0] + Bdc[3][1]*y[1];

	commande =  - Cdc[0]*(x_new[0])
				- Cdc[1]*(x_new[1])
				- Cdc[2]*(x_new[2])
				- Cdc[3]*(x_new[3]);

	x[0]=(x_new[0]);
	x[1]=(x_new[1]);
	x[2]=(x_new[2]);
	x[3]=(x_new[3]);
	commande = commande * 4;
	if((int)(commande*1000) > 10000)//Commande >10.00
	{
		commande = 10.0;		//dans l'ecriture de la commande le float peut prendre jusqu'a 10 pour ecrire 0 !
	}
	else if((int)(commande*1000) < -10000)//Commande <-10.00
	{
		commande = -10.0;
	}
}

/* write_DAC
 * Routine de l'actionneur:
 */
void write_DAC()
{
	set_DA(0, commande);                // on ecrit dans le canal 0, la "commande"
	//printk("%d\ta:%d\t\t",(int)(commande*1000),(int)(y[0]*10000));
}

/*
 *read_ADC
 * Routine de lecture des capteurs.
 */

void read_ADC()
{
    int     value, chan;
	trigger();
	while(adc_read_eoc() != 1);                            // Attente de fin d'acquisition
	value         = adc_read_value();                        //recuperation du (canal + valeur) concaténés
	chan         = value & 0x0F;                            //recuperation du canal
	value         = value >> 4 ;                            //recuperation de la valeur lue (0-4095)
	//printk("Cnl %d (0/4095): %d",chan,value);
	if(chan == 1 )            // on se trouve dans le canal de lecture de la position (canal 1)
	{
		y[1] = adc_convert_pos(value);
	}
	else if(chan == 0 )        // on se trouve dans le canal de lecture de l angle (canal 0)
	{
		y[0] = -adc_convert_angle(value);
	}
	//printk("Position*100= %d, Angle*100(rad)= %d\n\n",(int)(pos*100),(int)(angl*100));
}

/**************************************************************************\
|****************************** - RT STUFF - ******************************|
\**************************************************************************/
static int tpcan_init(void)
{
    int ierr1;
    RTIME now;
    printk("\n\n\n\ninside tpcan_init\n");
    rt_set_oneshot_mode();

    start_rt_timer(nano2count(TICK_PERIOD));
    now = rt_get_time();

    init_matrix();		// initialisation des matrices
	init_3718();		// init de l'adc
	printk("BC_PMn = %d \n BC_PMx = %d \n BC_AMn = %d \n BC_AMx = %d \n", BC_PMn, BC_PMx, BC_AMn, BC_AMx );
	
    //TACHE ADC
    ierr1 = rt_task_init(&tache_main,main_task,0,STACK_SIZE, PRIORITE, 1, 0);
    rt_task_make_periodic(&tache_main, now, nano2count(PERIODE_CONTROL));
    
    printk("exiting tpcan_init\n");
    /* creation taches périodiques */

    return(0);
}

static void tpcan_exit(void)
{
    //printk("inside tpcan_exit\n");
    stop_rt_timer();
    rt_task_delete(&tache_main);
    //printk("EXIT\n");
}
module_init(tpcan_init);
module_exit(tpcan_exit);

/**************************************************************************\
|***************************** - DRIVER ADC - *****************************|
\**************************************************************************/
/*
    Pour lire une valeur de l'ADC (une fois init):
    trigger() -> adc_read_eoc() -> adc_read_value()
*/

void init_3718(void)
{    //Pour lire canal 0, mettre 0x00. Pour lire canaux 0 et 1 alternativement, mettre 0x10
    //set_canal(0x00);
    ad_range_select(0x00,8);            // Canal 0, +/-10V
    ad_range_select(0x11,8);            // Canal 1, +/-10V
    set_canal(0x10);                    // Scan sur canaux 0 et 1
}

void trigger(void)
{
    // Trigger the A/D conversion by writing to the A/D low byte register (BASE+0) with any value.
    outb((char)0xFF,BASE_ADC_0);
}

void set_canal(int canal)
{
    outb((char)canal, BASE_ADC_2);                    // selectionne le canal    
}

void ad_range_select(int canal, int range)
{
    // Set the input range for each A/D channel. (p26/30))
    // base+1 D3 to D0 = b1000
    outb((char)canal, BASE_ADC_2);                    // selectionne le canal
    outb((char)range, BASE_ADC_1);                    // met le range
}

// ajout de cette fonction pour permettre une interruption quand fin de de conversion
int adc_read_eoc(void)
{    
	if((inb(BASE_ADC_8) & 0x80) == 0x80)
	{return 1;}
	else{return 0;}
	/*
    int eoc = 0;
    int temp = 0;
    eoc = inb(BASE_ADC_8);                            // EOC: end of convertion p41/37
    if((eoc & 0x80) == 0x80){temp = 1;}                // verif fin conversion
    else{ temp = 0;}
    return temp;
	*/
}

// lecture de la valeur une fois que la conversion est finie
int adc_read_value(void)
{
    int lsb    = 0;
    int msb    = 0;    
    int res	= 0;
    int chan =0;
    
    chan = inb(BASE_ADC_0);                         // recuperation du LSB
    msb = inb(BASE_ADC_1);                            // recuparation du MSB
    lsb = chan >> 4;                            // recuperation LSB
    chan = chan & 0x0F;                            // recuperation canal de la valeur lue
    
    // Convertion de la valeur + concatenation du canal dans les 4 bits de poids faible
    res = (((msb << 4) + lsb)<< 4 )+ chan;                             

    return res;
}

// suppression de la substitution de valeur quand depassement de la valeur max ou min
float adc_convert_pos(int value)
{    
    return ( ( ( (value-(BC_PMn)) * 1.2) / ( (BC_PMx)-(BC_PMn) ) )- 0.6); // verif OK (by nico)
}

//La valeur de l'angle n'est plus calculée à partir des valeurs extrêmes, mais du milieu (Antoine, 28/10) 
float adc_convert_angle(int value)
{    
   	return (value-MIDDLE)*COEFF_ANGL;
	//return ( ( ((value -(BC_AMn)) *MAX_ANGL*2.0) / (BC_AMx-(BC_AMn)) )- (MAX_ANGL)); // verif OK (by nico)
}


/**************************************************************************\
|***************************** - DRIVER DAC - *****************************|
\**************************************************************************/

int init_3712(void)
{
    return 0;                //No problem init
}

/*
 * set_DA
 * Function output: send in the channel "canal" the voltage "value" in volt (+-10V).
 * 8 LSB d'abord, 8 MSB après. Buffer garde resultat et n'envoie que quand MSB est écrit
 * Valeur à rentrer de -10 a 10
 */
void set_DA(int canal, float value)
{
    int lsb = 0;
    int msb = 0;
    int value_n = 0;
    value_n = (int)((value +10.0) * 4095.0 / 20.0);             // CONVERION IN NUMERIC
    lsb = value_n & 0x00FF;                            //Recuperation du LSB
    msb = value_n >> 8;                            //Recuperation du MSB
    outb(lsb,BASE_DAC_0+2*canal);
    outb(msb,BASE_DAC_1+2*canal);
}

/**************************************************************************\
|***************************** - LIB MATRIX - *****************************|
\**************************************************************************/
void init_matrix(void)
{
    printk("init_matrix : begin initialization\n");
    
    //Matrice un seul pendule    
    Adc[0][0]= 0.6196;    Adc[0][1]= 0.0968;    Adc[0][2]=-0.0008;    Adc[0][3]= 0.0086;
    Adc[1][0]= 0.0971;    Adc[1][1]= 0.7038;    Adc[1][2]= 0.0107;    Adc[1][3]= 0.0012;
    Adc[2][0]= 1.8124;    Adc[2][1]=-1.7997;    Adc[2][2]= 1.1306;    Adc[2][3]= 0.2351;
    Adc[3][0]=-3.8837;    Adc[3][1]= 0.8724;    Adc[3][2]=-0.1546;    Adc[3][3]= 0.7222;

    Bdc[0][0]= 0.3762;    Bdc[0][1]=-0.0973;
    Bdc[1][0]=-0.0931;    Bdc[1][1]= 0.2966;
    Bdc[2][0]=-1.0133;    Bdc[2][1]= 1.8954;
    Bdc[3][0]= 3.0534;    Bdc[3][1]=-0.9858;

    Cdc[0]=-80.3092;    Cdc[1]=-9.6237;    Cdc[2]=-14.1215;    Cdc[3]=-23.6260;
      
 	
    //Matrice deux pendules
/*    
	Adc[0][0]= 0.6300;    Adc[0][1]=-0.1206;    Adc[0][2]=-0.0008;    Adc[0][3]= 0.0086;
    Adc[1][0]=-0.0953;    Adc[1][1]= 0.6935;    Adc[1][2]= 0.0107;    Adc[1][3]= 0.0012;
    Adc[2][0]=-0.2896;    Adc[2][1]=-1.9184;    Adc[2][2]= 1.1306;    Adc[2][3]= 0.2351;
    Adc[3][0]=-3.9680;    Adc[3][1]=-1.7733;    Adc[3][2]=-0.1546;    Adc[3][3]= 0.7222;

    Bdc[0][0]=0.3658;    Bdc[0][1]=0.1200;
    Bdc[1][0]=0.0993;    Bdc[1][1]=0.3070;
    Bdc[2][0]=1.0887;    Bdc[2][1]=2.0141;
    Bdc[3][0]=3.1377;    Bdc[3][1]=1.6599;

    Cdc[0]=-80.3092;    Cdc[1]=-9.6237;    Cdc[2]=-14.1215;    Cdc[3]=-23.6260;
	*/ 

    y[0]=0;
    y[1]=0;

    x[0]=0;
    x[1]=0;
    x[2]=0;
    x[3]=0;

    printk("init_matrix : end initialization\n");
}




void affichage_float(float val)
{
    if(val>=0)
    {
        int decimal = (int)(val*1000)-((int)val)*1000;
        if(decimal>100)
        printk("%d,%d", (int)val,  (int)(val*1000)-((int)val)*1000 );
        else if(decimal > 10 )
        printk("%d,0%d", (int)val,  (int)(val*1000)-((int)val)*1000 );
        else
        printk("%d,00%d", (int)val,  (int)(val*1000)-((int)val)*1000 );
    }
    else
    {
        int decimal = (int)((-val)*1000)-((int)(val))*1000;
        if(decimal>100)
            printk("-%d,%d", (int)(-val),  (int)((-val)*1000)-((int)(val))*1000 );
        else if(decimal > 10 )
            printk("-%d,0%d", (int)(-val),  (int)((-val)*1000)-((int)(val))*1000 );
        else
            printk("-%d,00%d", (int)(-val),  (int)((-val)*1000)-((int)(val))*1000 );
    }
}


void verif_angle(int value)
{
    float check=0.0;
    int min=BC_AMn;
    int max=BC_AMx;
    check = (((value-min)*MAX_ANGL*2.0)/(max-min))-MAX_ANGL;
    if((check) > MAX_ANGL)
    {
        printk("verif angle : valeur trop haute ! Vérifier valeurs extremes du banc.\n");
    }
    else if((check) < -MAX_ANGL)
    {
        printk("verif angle : valeur trop basse ! Vérifier valeurs extremes du banc.\n");
    }
}

void verif_position(int value)
{
    float check;
    int min=BC_PMn;
    int max=BC_PMx;

    check = ((((value-min)*1.2)/(max-min))-0.6);
    if((check) > MAX_ANGL)
    {
        printk("convertion position : valeur trop haute ! Vérifier valeurs extremes du banc.\n");
    }
    else if((check) < -MAX_ANGL)
    {
        printk("convertion position : valeur trop basse ! Vérifier valeurs extremes du banc.\n");
    }
}
