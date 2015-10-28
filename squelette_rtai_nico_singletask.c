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
#define PERIODE_CONTROL 25000000	 //25ms
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
    #define BC_PMn        367
    #define BC_PMx        3839
    #define BC_AMn        1195
    #define BC_AMx        2823
#elif (BANC == 3)
    #define BC_PMn        400
    #define BC_PMx        3890
    #define BC_AMn        1200
    #define BC_AMx        2825//2910
#elif (BANC == 4)
    #define BC_PMn        5
    #define BC_PMx        4095
    #define BC_AMn        1291
    #define BC_AMx        2760
#elif (BANC == 5)
    #define BC_PMn        0
    #define BC_PMx        4095
    #define BC_AMn        1235
    #define BC_AMx        2800
#elif (BANC == 0)
    #define BC_PMn        0
    #define BC_PMx        4095
    #define BC_AMn        0
    #define BC_AMx        4095
#else
    #define BC_PMn        380
    #define BC_PMx        3820
    #define BC_AMn        1200
    #define BC_AMx        2820
#endif

/* RT_TASK */
static RT_TASK tache_main;
static int temps=0;

/*** DEFINE GLOBAL VAR ***/
float commande;
float Adc[4][4];
float Bdc[4][2];
float Cdc[1][4];
float x[4][1];
float x_save[4][1];
float y[2][1];
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


// probleme au niveau des données de lecture (sans mouvement du pendule varie jusqu'a 200 (int lue)
// ceci est apparue suite a la modification des fonction de conversion (ancienne version toujours commantee)
// changement des erreurs a la con, des define et de l'initialisation des matrices
// ATTENTION, affichage float fait des erreurs sur : 0.0011 au lieu de 0.011

void main_task(long arg)
{
	while(1)
	{
		read_ADC();
		read_ADC();
		//printk("pos= ");affichage_float(y[1][0]);printk(" | angl= ");affichage_float(y[0][0]);
		//printk("pos*1000= %d", (int)(y[1][0]*1000));
		//printk(" | angl*1000= %d\t", (int)(y[0][0]*1000));
		calc_matrix();
		write_DAC();
		//printk("\n");
		rt_task_wait_period();
	}
}

void calc_matrix()
{
        //printk("test matrix : get in\n");
       	//y[0][0] = 0.01;	//Angle
		//y[1][0] = 0.0;	//Position

  		//data.x=data.Adc*data.x+data.Bdc*data.y;
  		//data.u=-data.Cdc*data.x;
		vit_ang = (y[0][0] - ang_save)/PERIODE_CONTROL;
		ang_save = y[0][0];
		vit_pos = (y[1][0] - pos_save)/PERIODE_CONTROL;
		pos_save = y[1][0];



		/*
		x[0][0]=y[0][0];
	 	x[0][1]=y[1][0];
	 	x[0][2]=vit_ang;
	 	x[0][3]=vit_pos;
		*/
		x_save[0][0]= Adc[0][0]*x[0][0] + Adc[0][1]*x[1][0] + Adc[0][2]*x[2][0] + Adc[0][3]*x[3][0] + Bdc[0][0]*y[0][0] + Bdc[0][1]*y[1][0];
		x_save[1][0]= Adc[1][0]*x[0][0] + Adc[1][1]*x[1][0] + Adc[1][2]*x[2][0] + Adc[1][3]*x[3][0] + Bdc[1][0]*y[0][0] + Bdc[1][1]*y[1][0];
		x_save[2][0]= Adc[2][0]*x[0][0] + Adc[2][1]*x[1][0] + Adc[2][2]*x[2][0] + Adc[2][3]*x[3][0] + Bdc[2][0]*y[0][0] + Bdc[2][1]*y[1][0];
		x_save[3][0]= Adc[3][0]*x[0][0] + Adc[3][1]*x[1][0] + Adc[3][2]*x[2][0] + Adc[3][3]*x[3][0] + Bdc[3][0]*y[0][0] + Bdc[3][1]*y[1][0];

		commande =  - Cdc[0][0]*(x_save[0][0])
				 	- Cdc[0][1]*(x_save[1][0])
				 	- Cdc[0][2]*(x_save[2][0])
				 	- Cdc[0][3]*(x_save[3][0]);

		/*
		x[0][0]=(x_save[0][0]);
	 	x[0][1]=(x_save[1][0]);
	 	x[0][2]=(x_save[2][0]);
	 	x[0][3]=(x_save[3][0]);
		*/
		/*
		commande =  - Cdc[0][0]*(Adc[0][0]*x[0][0] + Adc[0][1]*x[1][0] + Adc[0][2]*x[2][0] + Adc[0][3]*x[3][0] +Bdc[0][0]*y[0][0] +Bdc[0][1]*y[1][0])
				 	- Cdc[0][1]*(Adc[1][0]*x[0][0] + Adc[1][1]*x[1][0] + Adc[1][2]*x[2][0] + Adc[1][3]*x[3][0] +Bdc[1][0]*y[0][0] +Bdc[1][1]*y[1][0])
				 	- Cdc[0][2]*(Adc[2][0]*x[0][0] + Adc[2][1]*x[1][0] + Adc[2][2]*x[2][0] + Adc[2][3]*x[3][0] +Bdc[2][0]*y[0][0] +Bdc[2][1]*y[1][0])
				 	- Cdc[0][3]*(Adc[3][0]*x[0][0] + Adc[3][1]*x[1][0] + Adc[3][2]*x[2][0] + Adc[3][3]*x[3][0] +Bdc[3][0]*y[0][0] +Bdc[3][1]*y[1][0]);
		*/
		 ////printk("Com*100: %d | ", (int)(commande*100));affichage_float(commande);//printk("\n");
            if((int)(commande*1000) > 10000)//Commande >10.00
            {
                commande = 10.0;		//dans l'ecriture de la commande le float peut prendre jusqu'a 10 pour ecrire 0 !
                //printk("(c>10)");
            }
            else if((int)(commande*1000) < -10000)//Commande <-10.00
            {
                commande = -10.0;
                // printk("(c<-10)");
            }
        //else printk("(-10<c<10)\n");

        //printk("test matrix : get out\n\n");
}

/* write_DAC
 * Routine de l'actionneur:
 * a terme: renvoi la valeur calculée par le processeur déporter.
 * Test a effectué: envoyé une commande proportionnel a l'angle du pendule
 * (permet de commander le moteur en direct)
 * !! ATTENTION ajouter un limite sur les bornes. (ex si val> born -> STOP)
 */
void write_DAC()
{
    //printk("write_DAC : START\n");
  
        //printk("write_DAC : get in\n");
        //printk("\nwrite_DAC : Ecriture de commande * 1000 =  %d \n", (int)(commande*1000));
        set_DA(0, commande);                // on ecrit dans le canal 0, la "commande"
        //printk("\nwrite_DAC : u*100 = %d\n",(int)(commande*100));
        //rt_task_wait_period();
        //printk("write_DAC : get out\n\n");
    
    //printk("write_DAC : END\n");
}

/*
 *read_ADC
 * Routine de lecture des capteurs.
 */

void read_ADC()
{
    int     value, chan;
    //printk("read_ADC : START\n");
    //init_3718();
      
        //printk("read_ADC : get in\n");
        trigger();
        while(adc_read_eoc() != 1);                            // Attente de fin d'acquisition
        value         = adc_read_value();                        //recuperation du (canal + valeur) concaténés
        chan         = value & 0x0F;                            //recuperation du canal
        value         = value >> 4 ;                            //recuperation de la valeur lue (0-4095)

        //printk("Cnl %d (0/4095): %d",chan,value);

    
        if(chan == 1 )            // on se trouve dans le canal de lecture de la position (canal 1)
        {
            //pos = adc_convert_pos(value);
            //verif_position(value);
            y[1][0] = adc_convert_pos(value);
            //printk("Position*100 (-60/60) = %d (value = %d)\n",(int)(pos*100),(int)pos);
        }
        else if(chan == 0 )        // on se trouve dans le canal de lecture de l angle (canal 0)
        {
            //angl = adc_convert_angle(value);
            //verif_angle(value);
            //y[0][0] = (-1)* adc_convert_angle(value);
			y[0][0] = -adc_convert_angle(value);		// plus logique sans moins d'apres la valeur retournee de l'angle qui sera deja negative si c'est la valeur la plus petite
            //printk("Angle*100 (-61/61)= %d (value = %d)\n",(int)(angl*100),(int)angl);
        }

        //printk("Position*100= %d, Angle*100(rad)= %d\n\n",(int)(pos*100),(int)(angl*100));
       
        //printk("read_ADC : get out\n\n");
 
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
    int eoc = 0;
    int temp = 0;
    eoc = inb(BASE_ADC_8);                            // EOC: end of convertion p41/37
    if((eoc & 0x80) == 0x80){temp = 1;}                // verif fin conversion
    else{ temp = 0;}
    return temp;
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
    //float check;
    //int min=BC_PMn;
    //int max=BC_PMx;
    //check = ((((value-(BC_PMn))*1.2)/((BC_PMx)-(BC_PMn)))- 0.6);
	//verif_position(value);
    return ( ( ( (value-(BC_PMn)) * 1.2) / ( (BC_PMx)-(BC_PMn) ) )- 0.6); // verif OK (by nico)
}

// suppression de la substitution de valeur quand depassement de la valeur max ou min
float adc_convert_angle(int value)
{    
    //printk("convert angle : get in\n");
    //printk("convert angle : get out\n");
	//verif_angle(value);
   
	return ( ( ((value -(BC_AMn)) *MAX_ANGL*2.0) / (BC_AMx-(BC_AMn)) )- (MAX_ANGL)); // verif OK (by nico)
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
    //printk("Valeur en bits : %d\n",value);
    //printk("Valeur MSB : %d\n",msb);
    //printk("Valeur LSB : %d\n",lsb);
    /* WRITE OUTPUT */
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

    Cdc[0][0]=-80.3092;    Cdc[0][1]=-9.6237;    Cdc[0][2]=-14.1215;    Cdc[0][3]=-23.6260;
       
 	/*
    //Matrice deux pendules (Allah vérifier)
    Adc[0][0]= 0.6300;    Adc[0][1]=-0.1206;    Adc[0][2]=-0.0008;    Adc[0][3]= 0.0086;
    Adc[1][0]=-0.0953;    Adc[1][1]= 0.6935;    Adc[1][2]= 0.0107;    Adc[1][3]= 0.0012;
    Adc[2][0]=-0.2896;    Adc[2][1]=-1.9184;    Adc[2][2]= 1.1306;    Adc[2][3]= 0.2351;
    Adc[3][0]=-3.9680;    Adc[3][1]=-1.7733;    Adc[3][2]=-0.1546;    Adc[3][3]= 0.7222;

    Bdc[0][0]=0.3658;    Bdc[0][1]=0.1200;
    Bdc[1][0]=0.0993;    Bdc[1][1]=0.3070;
    Bdc[2][0]=1.0887;    Bdc[2][1]=2.0141;
    Bdc[3][0]=3.1377;    Bdc[3][1]=1.6599;

    Cdc[0][0]=-80.3092;    Cdc[0][1]=-9.6237;    Cdc[0][2]=-14.1215;    Cdc[0][3]=-23.6260;
	*/ 

    y[0][0]=0;
    y[1][0]=0;

    x[0][0]=0;
    x[1][0]=0;
    x[2][0]=0;
    x[3][0]=0;

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
    //printk("verif angle : get in\n");
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


